"""Thin pymavlink wrapper for driving one ArduCopter SITL instance.

Everything here speaks **NED in metres**, matching
:mod:`guidance_sim.engagement`, so no frame conversion happens between the
telemetry and the guidance maths. ``LOCAL_POSITION_NED`` already carries
``x, y, z`` in metres and ``vx, vy, vz`` in m/s as floats (unlike
``GLOBAL_POSITION_INT``, which is centimetres), so the fields are used as-is.

Sending an acceleration setpoint
--------------------------------
``SET_POSITION_TARGET_LOCAL_NED`` with position, velocity, yaw and yaw-rate all
masked off routes to ``ModeGuided::set_accel_NED_mss()`` in ArduCopter, which
feeds ``AC_PosControl::input_accel_NE_m()`` -- a genuine acceleration input,
with velocity and position free-running as the controller integrates it.
Any mask combination ArduCopter does not explicitly recognise silently falls
through to ``hold_position()``, so :data:`ACCEL_TYPEMASK` is not negotiable.

The setpoint must be **resent continuously**. ArduCopter zeroes both the
velocity and acceleration targets and brakes to a hover if nothing arrives
within ``GUID_TIMEOUT`` (3 s by default). Run the loop at 50 Hz.
"""

from __future__ import annotations

import math
import time

import numpy as np
from pymavlink import mavutil

#: Ignore position (bits 0-2), velocity (bits 3-5), yaw (bit 10) and yaw rate
#: (bit 11), leaving only the acceleration fields active. 7|56|1024|2048.
ACCEL_TYPEMASK = 0b110000111111          # 3135 == 0xC3F

#: Ignore position (bits 0-2), acceleration (bits 6-8), yaw and yaw rate,
#: leaving the velocity fields active. Used to fly the target on a set course.
VELOCITY_TYPEMASK = 0b110111000111       # 3527

#: ``LOCAL_POSITION_NED``; requested at a high rate because the guidance law
#: differentiates nothing but still needs fresh relative geometry each tick.
MSG_LOCAL_POSITION_NED = 32


class VehicleError(RuntimeError):
    """Raised when a vehicle does not reach a commanded state in time."""


class Vehicle:
    """One SITL instance, addressed over MAVLink.

    ``url`` is a pymavlink connection string, e.g. ``udp:127.0.0.1:14550``.
    """

    def __init__(self, url: str, name: str = 'veh', rate_hz: float = 50.0):
        self.url = url
        self.name = name
        self.master = mavutil.mavlink_connection(url)
        self._pos = np.zeros(3)
        self._vel = np.zeros(3)
        self._have_state = False
        self._armed = False
        self._mode = ''
        self._rate_hz = rate_hz

    # --- connection -----------------------------------------------------

    def wait_heartbeat(self, timeout: float = 60.0) -> 'Vehicle':
        print(f'[{self.name}] waiting for heartbeat on {self.url} ...')
        hb = self.master.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise VehicleError(f'[{self.name}] no heartbeat from {self.url}')
        print(f'[{self.name}] heartbeat: system {self.master.target_system} '
              f'component {self.master.target_component}')
        self.request_streams(self._rate_hz)
        return self

    def request_streams(self, rate_hz: float) -> None:
        """Ask for LOCAL_POSITION_NED fast enough for a 50 Hz guidance loop."""
        interval_us = int(1e6 / max(rate_hz, 1.0))
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            MSG_LOCAL_POSITION_NED, interval_us, 0, 0, 0, 0, 0)

    # --- telemetry ------------------------------------------------------

    def poll(self) -> None:
        """Drain the link without blocking and cache the latest state.

        Called every tick of the guidance loop; it must never block, or the
        loop rate collapses and ArduCopter times the setpoint out.
        """
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == 'LOCAL_POSITION_NED':
                self._pos = np.array([msg.x, msg.y, msg.z], dtype=float)
                self._vel = np.array([msg.vx, msg.vy, msg.vz], dtype=float)
                self._have_state = True
            elif kind == 'HEARTBEAT':
                self._armed = bool(msg.base_mode
                                   & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._mode = mavutil.mode_string_v10(msg)

    def wait_state(self, timeout: float = 30.0) -> 'Vehicle':
        """Block until at least one LOCAL_POSITION_NED has arrived."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll()
            if self._have_state:
                return self
            time.sleep(0.01)
        raise VehicleError(f'[{self.name}] no LOCAL_POSITION_NED in {timeout}s')

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        """Latest ``(position, velocity)`` in NED metres and m/s."""
        return self._pos.copy(), self._vel.copy()

    @property
    def altitude(self) -> float:
        """Height above the local origin, i.e. ``-z``."""
        return -float(self._pos[2])

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def mode(self) -> str:
        return self._mode

    # --- commands -------------------------------------------------------

    def set_param(self, name: str, value: float,
                  ptype: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32) -> None:
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            name.encode('ascii'), float(value), ptype)

    def set_mode(self, mode: str, timeout: float = 10.0) -> None:
        mapping = self.master.mode_mapping() or {}
        if mode not in mapping:
            raise VehicleError(f'[{self.name}] unknown mode {mode!r}')
        self.master.set_mode(mapping[mode])
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll()
            if self._mode == mode:
                print(f'[{self.name}] mode {mode}')
                return
            time.sleep(0.05)
        raise VehicleError(f'[{self.name}] mode {mode} not confirmed')

    def _command_long(self, command: int, *params: float) -> None:
        args = list(params) + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            command, 0, *args)

    def arm(self, timeout: float = 30.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               1.0)
            for _ in range(20):
                self.poll()
                if self._armed:
                    print(f'[{self.name}] armed')
                    return
                time.sleep(0.05)
        raise VehicleError(f'[{self.name}] failed to arm (pre-arm checks?)')

    def disarm(self, force: bool = False, timeout: float = 20.0) -> None:
        deadline = time.time() + timeout
        magic = 21196.0 if force else 0.0
        while time.time() < deadline:
            self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               0.0, magic)
            for _ in range(20):
                self.poll()
                if not self._armed:
                    print(f'[{self.name}] disarmed')
                    return
                time.sleep(0.05)
        raise VehicleError(f'[{self.name}] failed to disarm')

    def takeoff_async(self, alt: float) -> None:
        """Command a takeoff and return immediately, without waiting.

        Split out from :meth:`takeoff` so several vehicles can be launched
        together: issue this to every vehicle first, then wait on all of them
        with :meth:`wait_altitude`. Waiting on each in turn instead would make
        the second copter sit on the ground until the first had finished
        climbing.
        """
        self._command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                           0, 0, 0, 0, 0, 0, float(alt))

    def wait_altitude(self, alt: float, timeout: float = 90.0,
                      tol: float = 0.5) -> None:
        """Block until this vehicle is within ``tol`` of ``alt`` metres."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll()
            if self.altitude >= alt - tol:
                print(f'[{self.name}] at {self.altitude:.1f} m')
                return
            time.sleep(0.05)
        raise VehicleError(f'[{self.name}] takeoff to {alt} m timed out at '
                           f'{self.altitude:.1f} m')

    def takeoff(self, alt: float, timeout: float = 90.0,
                tol: float = 0.5) -> None:
        """Climb to ``alt`` metres above the local origin and settle."""
        self.takeoff_async(alt)
        self.wait_altitude(alt, timeout=timeout, tol=tol)

    def land_and_wait(self, timeout: float = 120.0) -> None:
        self.set_mode('LAND')
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll()
            if not self._armed:
                print(f'[{self.name}] landed and disarmed')
                return
            time.sleep(0.1)
        raise VehicleError(f'[{self.name}] did not land within {timeout}s')

    # --- setpoints ------------------------------------------------------

    def send_accel_ned(self, a_ned) -> None:
        """Send one acceleration setpoint. Must be repeated at >= 10 Hz."""
        a = np.asarray(a_ned, dtype=float)
        self.master.mav.set_position_target_local_ned_send(
            0,                                        # time_boot_ms
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            ACCEL_TYPEMASK,
            0.0, 0.0, 0.0,                            # position (ignored)
            0.0, 0.0, 0.0,                            # velocity (ignored)
            float(a[0]), float(a[1]), float(a[2]),    # acceleration
            0.0, 0.0)                                 # yaw, yaw rate

    def send_velocity_ned(self, v_ned) -> None:
        """Send one velocity setpoint. Used to fly the target's course."""
        v = np.asarray(v_ned, dtype=float)
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            VELOCITY_TYPEMASK,
            0.0, 0.0, 0.0,
            float(v[0]), float(v[1]), float(v[2]),
            0.0, 0.0, 0.0,
            0.0, 0.0)

    def close(self) -> None:
        self.master.close()


def connect(url: str, name: str, alt_check: bool = True,
            rate_hz: float = 50.0) -> Vehicle:
    """Connect, wait for a heartbeat, and wait for usable telemetry."""
    veh = Vehicle(url, name, rate_hz=rate_hz).wait_heartbeat()
    if alt_check:
        veh.wait_state()
    return veh


def heading_to_velocity(speed: float, heading_rad: float) -> np.ndarray:
    """NED velocity for a horizontal course, using the package's bearing
    convention (from North, positive towards East)."""
    return np.array([speed * math.cos(heading_rad),
                     speed * math.sin(heading_rad), 0.0])
