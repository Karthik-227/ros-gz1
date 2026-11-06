"""Arm, take off and land SITL vehicles.

Convenience wrappers so an engagement can be set up without MAVProxy.

Four entry points:

``takeoff`` / ``land``            one vehicle, addressed by ``--url``
``takeoff_both`` / ``land_both``  both engagement vehicles in one command

``takeoff_both`` launches them *together*: every vehicle is armed first, then
the takeoff command goes to all of them, and only then does it wait for the
climbs. Arming and launching one vehicle at a time would leave the second on
the ground for the whole of the first one's climb, which is both slow and a
different initial condition from the one the guidance run assumes.
"""

from __future__ import annotations

import argparse

from .mavlink_io import VehicleError, connect


def _common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--url', default='udp:127.0.0.1:14550',
                        help='MAVLink connection string '
                             '(default udp:127.0.0.1:14550)')
    parser.add_argument('--name', default='veh', help='label used in logs')


def main(argv=None) -> int:
    """Arm in GUIDED and climb to ``--alt``."""
    parser = argparse.ArgumentParser(
        prog='takeoff', description='Arm a SITL copter and take off.')
    _common(parser)
    parser.add_argument('--alt', type=float, default=50.0,
                        help='target altitude above home, m (default 50)')
    args = parser.parse_args(argv)

    veh = connect(args.url, args.name)
    try:
        veh.set_mode('GUIDED')
        veh.arm()
        veh.takeoff(args.alt)
    except VehicleError as exc:
        print(f'error: {exc}')
        return 1
    finally:
        veh.close()
    return 0


def main_land(argv=None) -> int:
    """Land (or RTL) and wait for disarm."""
    parser = argparse.ArgumentParser(
        prog='land', description='Land a SITL copter and wait for disarm.')
    _common(parser)
    parser.add_argument('--rtl', action='store_true',
                        help='return to launch instead of landing in place')
    args = parser.parse_args(argv)

    veh = connect(args.url, args.name)
    try:
        if args.rtl:
            veh.set_mode('RTL')
        veh.land_and_wait()
    except VehicleError as exc:
        print(f'error: {exc}')
        return 1
    finally:
        veh.close()
    return 0


# --------------------------------------------------------------------------- #
# Both vehicles at once
# --------------------------------------------------------------------------- #

def _fleet_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--pursuer-url', default='udp:127.0.0.1:14550',
                        help='pursuer MAVLink link (SITL instance 0)')
    parser.add_argument('--target-url', default='udp:127.0.0.1:14560',
                        help='target MAVLink link (SITL instance 1)')


def _connect_fleet(args, fleet: list) -> list:
    """Connect both vehicles, appending each one as it comes up.

    Appends in place rather than returning a fresh list so that a failure on
    the second link still leaves the first in the caller's list, where the
    ``finally`` block will close it.
    """
    for url, name in ((args.pursuer_url, 'pursuer'),
                      (args.target_url, 'target')):
        fleet.append(connect(url, name))
    return fleet


def main_both(argv=None) -> int:
    """Arm both engagement vehicles in GUIDED and launch them together."""
    parser = argparse.ArgumentParser(
        prog='takeoff_both',
        description='Arm and take off both SITL copters on one command.')
    _fleet_args(parser)
    parser.add_argument('--alt', type=float, default=50.0,
                        help='target altitude above home, m (default 50)')
    parser.add_argument('--prompt', action='store_true',
                        help='arm both, then wait for Enter before launching '
                             '(handy for starting a recording on the mark)')
    parser.add_argument('--timeout', type=float, default=90.0,
                        help='per-vehicle climb timeout, s (default 90)')
    args = parser.parse_args(argv)

    fleet: list = []
    try:
        _connect_fleet(args, fleet)

        # Phase 1: every vehicle into GUIDED and armed. Doing this for all of
        # them before any takeoff means a pre-arm failure is reported with
        # nothing in the air.
        for veh in fleet:
            veh.set_mode('GUIDED')
        for veh in fleet:
            veh.arm()

        if args.prompt:
            input('[run] both armed. Press Enter to take off to '
                  f'{args.alt:g} m ...')

        # Phase 2: command every takeoff, then wait. Order matters -- see the
        # module docstring.
        for veh in fleet:
            veh.takeoff_async(args.alt)
            print(f'[{veh.name}] takeoff commanded to {args.alt:g} m')
        for veh in fleet:
            veh.wait_altitude(args.alt, timeout=args.timeout)

        alts = ', '.join(f'{veh.name} {veh.altitude:.1f} m' for veh in fleet)
        print(f'[run] both airborne: {alts}')
    except VehicleError as exc:
        print(f'error: {exc}')
        return 1
    except KeyboardInterrupt:
        print('\n[run] interrupted')
        return 1
    finally:
        for veh in fleet:
            veh.close()
    return 0


def main_land_both(argv=None) -> int:
    """Land both engagement vehicles and wait for them to disarm."""
    parser = argparse.ArgumentParser(
        prog='land_both',
        description='Land both SITL copters and wait for disarm.')
    _fleet_args(parser)
    parser.add_argument('--rtl', action='store_true',
                        help='return to launch instead of landing in place')
    args = parser.parse_args(argv)

    fleet: list = []
    rc = 0
    try:
        _connect_fleet(args, fleet)
        for veh in fleet:
            veh.set_mode('RTL' if args.rtl else 'LAND')
        for veh in fleet:
            # Land each one even if an earlier vehicle timed out: leaving a
            # copter hovering because its partner misbehaved is worse than
            # reporting two failures.
            try:
                veh.land_and_wait()
            except VehicleError as exc:
                print(f'error: {exc}')
                rc = 1
    except VehicleError as exc:
        print(f'error: {exc}')
        return 1
    except KeyboardInterrupt:
        print('\n[run] interrupted')
        return 1
    finally:
        for veh in fleet:
            veh.close()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
