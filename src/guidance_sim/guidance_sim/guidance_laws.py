"""A library of guidance laws, each returning a lateral acceleration command.

Every law is a subclass of :class:`GuidanceLaw` and implements
``command(eng, dt)``, returning a **NED acceleration vector** in m/s^2. The
signed planar latax that gets plotted is recovered with
:meth:`~guidance_sim.engagement.Engagement.latax_of`, i.e. the component along
the pursuer's right-hand normal.

The laws are written in the vector form so the same code runs for a planar
engagement and for a three-dimensional one. For the planar case each reduces to
the scalar expression quoted in its docstring, with ``lamdot`` the LOS rate and
``Vc`` the closing velocity.

Implemented
-----------
=========================  ===============================================
``pure_pursuit``           ``a = K Vp (lam - gam)``
``deviated_pursuit``       ``a = K Vp (lam - gam - delta)``
``los`` (CLOS/beam rider)  station-referenced, drives pursuer onto the beam
``pn`` / ``tpn``           ``a = N Vc lamdot``, applied perpendicular to LOS
``ppn``                    ``a = N Vp lamdot``, perpendicular to velocity
``apn``                    ``a = N Vc lamdot + (N/2) a_T_perp``
``zem``                    ``a = N * ZEM_perp / t_go^2`` (= PN for N = 3)
=========================  ===============================================

All laws clip to ``a_max`` (the latax limit) so that capturability studies see
a realistic acceleration bound rather than an unbounded command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields

import numpy as np

from .engagement import Engagement, right_normal, unit, wrap_pi


def _clip_norm(v: np.ndarray, limit: float) -> np.ndarray:
    """Scale ``v`` down so its norm does not exceed ``limit``."""
    n = float(np.linalg.norm(v))
    if limit is None or limit <= 0 or n <= limit or n < 1e-12:
        return v
    return v * (limit / n)


@dataclass
class GuidanceLaw:
    """Base class. ``a_max`` is the latax limit in m/s^2.

    The default is ``g * tan(30 deg) = 5.66``, which is what an ArduCopter
    quad can actually deliver at the stock ``ANGLE_MAX`` of 30 degrees. A law
    that saturates here would saturate on the real airframe too, which is the
    whole point of imposing the bound.
    """

    a_max: float = 5.66
    #: Human-readable name, set by subclasses.
    name: str = 'base'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        """Return the commanded acceleration in NED, before clipping."""
        raise NotImplementedError

    def __call__(self, eng: Engagement, dt: float) -> np.ndarray:
        """Return the clipped NED acceleration command."""
        return _clip_norm(np.asarray(self.command(eng, dt), dtype=float),
                          self.a_max)

    def reset(self) -> None:
        """Clear any internal state. Called at the start of a run."""

    def extra_log(self) -> dict:
        """Law-specific values to add to the CSV log."""
        return {}


# --------------------------------------------------------------------------- #
# Pursuit family
# --------------------------------------------------------------------------- #

@dataclass
class PurePursuit(GuidanceLaw):
    """Pure pursuit: hold the velocity vector along the line of sight.

    Velocity pursuit is a kinematic constraint rather than an acceleration
    law, so it is realised here as a proportional heading regulator::

        a = K * Vp * (lam - gam)

    which drives the heading error ``eps`` to zero. Capturability requires a
    speed advantage (``nu = Vt/Vp < 1``); the latax demand grows without bound
    as the pursuer closes on a manoeuvring target from behind, which is the
    classic tail-chase problem.
    """

    k: float = 2.0
    name: str = 'pure_pursuit'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        latax = self.k * eng.Vp * eng.eps
        return latax * right_normal(eng.v_p)


@dataclass
class DeviatedPursuit(PurePursuit):
    """Deviated (fixed-lead) pursuit: hold a constant lead angle ``delta``.

        a = K * Vp * (lam - gam - delta)

    Choosing ``delta`` equal to the collision-triangle lead angle turns this
    into a constant-bearing course against a non-manoeuvring target, which
    removes the tail-chase latax blow-up of pure pursuit.
    """

    delta: float = math.radians(20.0)
    name: str = 'deviated_pursuit'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        err = wrap_pi(eng.eps - self.delta)
        latax = self.k * eng.Vp * err
        return latax * right_normal(eng.v_p)


@dataclass
class ConstantBearing(PurePursuit):
    """Constant-bearing course: steer onto the instantaneous collision triangle.

    The lead angle is recomputed every step from the target's current velocity
    instead of being fixed, so this is deviated pursuit with a self-adjusting
    ``delta``. Against a constant-velocity target it flies a straight line.
    """

    name: str = 'constant_bearing'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        alpha_p_star = eng.collision_heading()
        if alpha_p_star is None:
            # No collision triangle exists (target outruns us); fall back to
            # pointing straight at it.
            return self.k * eng.Vp * eng.eps * right_normal(eng.v_p)
        err = wrap_pi(alpha_p_star - eng.alpha_p)
        return self.k * eng.Vp * err * right_normal(eng.v_p)


# --------------------------------------------------------------------------- #
# Proportional navigation family
# --------------------------------------------------------------------------- #

@dataclass
class ProportionalNavigation(GuidanceLaw):
    """True proportional navigation (TPN): ``a = N * Vc * lamdot``.

    The command is applied perpendicular to the **line of sight**. In vector
    form, with ``omega`` the LOS angular velocity and ``r_hat`` the LOS unit
    vector::

        a = N * Vc * (omega x r_hat)

    Nulling ``lamdot`` holds the LOS direction fixed in inertial space, which
    is exactly the collision-course condition. Effective navigation ratios of
    ``N = 3`` to ``5`` are the usual choice: ``N > 2`` is needed for
    capturability, and large ``N`` amplifies seeker noise.
    """

    N: float = 3.0
    name: str = 'pn'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        if eng.R < 1e-6:
            return np.zeros(3)
        return self.N * eng.Vc * np.cross(eng.omega, unit(eng.r))


@dataclass
class PurePN(ProportionalNavigation):
    """Pure proportional navigation (PPN): ``a = N * Vp * lamdot``.

    Uses the pursuer's own speed instead of the closing velocity, and applies
    the command perpendicular to the **velocity vector** rather than the LOS::

        a = N * Vp * (omega x v_hat)

    This is what an airframe that can only turn, rather than accelerate
    sideways, actually delivers. Against a non-manoeuvring target PPN and TPN
    behave almost identically near the collision course, where ``Vc`` and
    ``Vp`` differ only by the target's closing contribution; they diverge in a
    tail chase, where ``Vc`` collapses and TPN's gain collapses with it while
    PPN keeps turning.
    """

    name: str = 'ppn'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        if eng.Vp < 1e-6:
            return np.zeros(3)
        return self.N * eng.Vp * np.cross(eng.omega, unit(eng.v_p))


@dataclass
class AugmentedPN(ProportionalNavigation):
    """Augmented proportional navigation (APN)::

        a = N * Vc * lamdot + (N / 2) * a_T_perp

    The extra term feeds the target's measured lateral acceleration forward, so
    the pursuer anticipates a manoeuvre instead of only reacting to the LOS
    rate it produces. Plain PN leaves a steady-state miss against a constantly
    accelerating target; the ``N/2`` feed-forward is exactly the term that
    cancels it, which is why APN is the optimal-control solution for a
    constant-manoeuvre target. It needs an estimate of ``a_T``, which is the
    practical obstacle -- here it is handed over exactly via ``eng.a_t``.
    """

    name: str = 'apn'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        if eng.R < 1e-6:
            return np.zeros(3)
        a_pn = self.N * eng.Vc * np.cross(eng.omega, unit(eng.r))
        return a_pn + 0.5 * self.N * eng.a_t_perp_los


@dataclass
class ZEMGuidance(ProportionalNavigation):
    """Zero-effort-miss guidance::

        a = N * ZEM_perp / t_go^2

    ``ZEM`` is the miss that would occur if neither vehicle accelerated again;
    nulling it is the whole job. Substituting the planar identity
    ``|ZEM_perp| = R^2 * lamdot / Vc`` together with ``t_go = R / Vc`` turns
    this into ``N * Vc * lamdot``, so **ZEM guidance with N = 3 is exactly
    PN with N = 3** -- the two classical laws are one law in different
    coordinates. Writing it this way makes the generalisation obvious: swap in
    a better ``t_go`` or a predicted ``ZEM`` and the same formula handles drag,
    gravity or a manoeuvring target.
    """

    name: str = 'zem'

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        # An opening engagement has no finite t_go, so there is no zero-effort
        # miss to null. Turn back towards the target until closure is restored.
        if not math.isfinite(eng.t_go):
            return _pursuit_fallback(eng)
        t_go = max(eng.t_go, 1e-3)
        return self.N * eng.zem_perp / t_go**2


# --------------------------------------------------------------------------- #
# Line-of-sight (command-to-LOS / beam rider)
# --------------------------------------------------------------------------- #

@dataclass
class LOSGuidance(GuidanceLaw):
    """Command to line of sight (CLOS), a.k.a. beam riding.

    A ground station at ``station`` watches the target; the beam is the
    station-to-target line, and the pursuer's only job is to stay on it::

        e     = component of (p_P - station) perpendicular to the beam
        a     = -(kp * e + kd * de/dt)

    Unlike the pursuit and PN families this law is *station referenced*: the
    pursuer needs no seeker of its own, only its position relative to the beam,
    which is why it was the first practical guidance scheme. The cost is that
    the required latax grows with range -- a small angular beam error is a
    large cross-track distance far out -- so CLOS is a short-range law, and it
    degrades badly against a target that manoeuvres across the beam.

    The derivative term is a plain finite difference, so this law carries
    state and :meth:`reset` must be called between runs.
    """

    station: np.ndarray = field(default_factory=lambda: np.zeros(3))
    kp: float = 1.0
    kd: float = 2.5
    name: str = 'los'
    _e_prev: np.ndarray | None = field(default=None, repr=False, compare=False)
    _xtrack: float = field(default=0.0, repr=False, compare=False)

    def reset(self) -> None:
        self._e_prev = None
        self._xtrack = 0.0

    def command(self, eng: Engagement, dt: float) -> np.ndarray:
        station = np.asarray(self.station, dtype=float)
        beam = unit(eng.p_t - station)
        if not np.any(beam):
            # Target sitting on the station: no beam to ride.
            return _pursuit_fallback(eng)

        d = eng.p_p - station
        e = d - (d @ beam) * beam
        self._xtrack = float(np.linalg.norm(e))

        if self._e_prev is None or dt <= 0.0:
            e_dot = np.zeros(3)
        else:
            e_dot = (e - self._e_prev) / dt
        self._e_prev = e

        return -(self.kp * e + self.kd * e_dot)

    def extra_log(self) -> dict:
        return {'los_xtrack': self._xtrack}


# --------------------------------------------------------------------------- #
# Registry
# --------------------------------------------------------------------------- #

def _pursuit_fallback(eng: Engagement, k: float = 2.0) -> np.ndarray:
    """Heading regulator towards the target, used when a law is undefined.

    Several laws have a degenerate case (no closure, no beam). Rather than
    coasting, fall back to turning towards the target so the engagement can
    recover instead of ending the run with a meaningless zero command.
    """
    return k * eng.Vp * eng.eps * right_normal(eng.v_p)


#: Selectable laws, keyed by the name used on the command line.
LAWS: dict[str, type[GuidanceLaw]] = {
    'pure_pursuit': PurePursuit,
    'deviated_pursuit': DeviatedPursuit,
    'constant_bearing': ConstantBearing,
    'pn': ProportionalNavigation,
    'tpn': ProportionalNavigation,
    'ppn': PurePN,
    'apn': AugmentedPN,
    'zem': ZEMGuidance,
    'los': LOSGuidance,
}


def make_law(name: str, **kwargs) -> GuidanceLaw:
    """Build a law by name, ignoring keyword arguments it does not accept.

    This lets one CLI expose ``--N``, ``--k`` and ``--delta`` at once and hand
    the whole lot to whichever law was selected; a law simply uses the knobs
    that apply to it and ignores the rest.
    """
    try:
        cls = LAWS[name]
    except KeyError:
        raise KeyError(
            f'unknown guidance law {name!r}; choose from '
            f'{", ".join(sorted(LAWS))}') from None
    allowed = {f.name for f in fields(cls) if not f.name.startswith('_')}
    accepted = {k: v for k, v in kwargs.items()
                if k in allowed and v is not None}
    law = cls(**accepted)
    law.reset()
    return law
