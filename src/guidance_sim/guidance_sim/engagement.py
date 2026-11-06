"""Engagement geometry for a two-body pursuit problem.

Frame and sign conventions (used consistently everywhere in this package)
------------------------------------------------------------------------
Inertial frame is **NED**: x = North, y = East, z = Down. This is the frame
MAVLink reports in (``LOCAL_POSITION_NED``), so no extra conversion is needed
between the flight code and the guidance maths.

* Line-of-sight angle ``lam`` (lambda, sometimes sigma in the notes) is the
  bearing of the target from the pursuer, measured from North and positive
  towards East (i.e. positive clockwise seen from above)::

      lam = atan2(dE, dN)

* Pursuer heading ``alpha_p`` uses the same convention, and the target's is
  ``alpha_t``::

      alpha_p = atan2(vE, vN)

* Lateral acceleration ``latax`` is signed, positive to the pursuer's
  **right**. With the bearing convention above this gives the clean kinematic
  relation::

      gamdot = latax / V

* Angular rates about the Down axis are positive for a clockwise (viewed from
  above) rotation, so the Down component of the LOS angular velocity vector
  equals ``lamdot`` directly. This is why the 3D vector formulation below drops
  straight out of the planar one.

Core quantities
---------------
With ``r = p_T - p_P`` and ``v_rel = v_T - v_P``:

===================  ====================================================
``R``                range, ``|r|``
``Rdot``             range rate, ``r_hat . v_rel``
``Vc``               closing velocity, ``-Rdot``
``omega``            LOS angular velocity vector, ``(r x v_rel) / R^2``
``lamdot``           planar LOS rate, Down component of ``omega``
``t_go``             time-to-go estimate, ``R / Vc``
``zem``              zero-effort miss vector, ``r + v_rel * t_go``
===================  ====================================================

``zem`` is the miss that would occur if neither vehicle accelerated from now
on; its component perpendicular to the LOS has magnitude ``R^2 * lamdot / Vc``
in the planar case, which is the identity that makes ZEM guidance and
proportional navigation with ``N = 3`` the same law.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

#: Down axis of the NED frame; positive rotation about it is clockwise
#: when viewed from above, matching the bearing convention for ``lam``.
DOWN = np.array([0.0, 0.0, 1.0])


def wrap_pi(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def unit(v: np.ndarray) -> np.ndarray:
    """Unit vector along ``v``; the zero vector maps to itself."""
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


def bearing(v: np.ndarray) -> float:
    """Bearing of a NED vector's horizontal part, from North towards East."""
    return math.atan2(v[1], v[0])


def right_normal(v: np.ndarray) -> np.ndarray:
    """Horizontal unit vector 90 deg to the right of ``v`` (NED).

    Used to turn a signed planar latax into a NED acceleration vector, and to
    recover the signed latax from one.
    """
    h = np.array([v[0], v[1], 0.0])
    n = float(np.linalg.norm(h))
    if n < 1e-12:
        return np.zeros(3)
    return np.array([-h[1], h[0], 0.0]) / n


@dataclass
class Engagement:
    """Instantaneous engagement geometry between a pursuer and a target.

    All vectors are NED and in SI units. Construct with :meth:`from_states`.
    """

    # --- raw states -----------------------------------------------------
    p_p: np.ndarray                 # pursuer position
    v_p: np.ndarray                 # pursuer velocity
    p_t: np.ndarray                 # target position
    v_t: np.ndarray                 # target velocity
    a_t: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # --- derived geometry (filled by from_states) -----------------------
    r: np.ndarray = field(default_factory=lambda: np.zeros(3))
    v_rel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    R: float = 0.0
    Rdot: float = 0.0
    Vc: float = 0.0
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))
    lam: float = 0.0
    lamdot: float = 0.0
    alpha_p: float = 0.0            # pursuer heading
    alpha_t: float = 0.0            # target heading
    Vp: float = 0.0
    Vt: float = 0.0
    eps: float = 0.0                # heading error / lead angle, lam - alpha_p
    t_go: float = float('inf')
    zem: np.ndarray = field(default_factory=lambda: np.zeros(3))
    zem_perp: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_states(cls, p_p, v_p, p_t, v_t, a_t=None) -> 'Engagement':
        """Build an :class:`Engagement` from pursuer and target states."""
        p_p = np.asarray(p_p, dtype=float)
        v_p = np.asarray(v_p, dtype=float)
        p_t = np.asarray(p_t, dtype=float)
        v_t = np.asarray(v_t, dtype=float)
        a_t = np.zeros(3) if a_t is None else np.asarray(a_t, dtype=float)

        e = cls(p_p=p_p, v_p=v_p, p_t=p_t, v_t=v_t, a_t=a_t)
        e.r = p_t - p_p
        e.v_rel = v_t - v_p
        e.R = float(np.linalg.norm(e.r))
        r_hat = unit(e.r)

        e.Rdot = float(r_hat @ e.v_rel)
        e.Vc = -e.Rdot
        # LOS angular velocity. Guard against R -> 0 at intercept so the logs
        # stay finite rather than blowing up on the last sample.
        e.omega = (np.cross(e.r, e.v_rel) / e.R**2) if e.R > 1e-6 else np.zeros(3)
        e.lamdot = float(e.omega @ DOWN)

        e.lam = bearing(e.r)
        e.alpha_p = bearing(v_p)
        e.alpha_t = bearing(v_t)
        e.Vp = float(np.linalg.norm(v_p))
        e.Vt = float(np.linalg.norm(v_t))
        e.eps = wrap_pi(e.lam - e.alpha_p)

        # Time-to-go. Only meaningful while actually closing; an opening
        # engagement has no finite intercept under the zero-effort assumption.
        e.t_go = e.R / e.Vc if e.Vc > 1e-6 else float('inf')
        if math.isfinite(e.t_go):
            e.zem = e.r + e.v_rel * e.t_go
            e.zem_perp = e.zem - (e.zem @ r_hat) * r_hat
        return e

    # --- convenience ----------------------------------------------------

    @property
    def speed_ratio(self) -> float:
        """Target-to-pursuer speed ratio ``nu = Vt / Vp``.

        The classical capturability conditions are stated in terms of this:
        pure pursuit only captures for ``nu < 1``, whereas proportional
        navigation can capture a faster target given enough latax authority.
        """
        return self.Vt / self.Vp if self.Vp > 1e-6 else float('inf')

    @property
    def a_t_perp_los(self) -> np.ndarray:
        """Target acceleration component perpendicular to the LOS."""
        r_hat = unit(self.r)
        return self.a_t - (self.a_t @ r_hat) * r_hat

    def latax_of(self, a_cmd: np.ndarray) -> float:
        """Signed planar latax (positive right) of a NED accel command."""
        return float(np.asarray(a_cmd, dtype=float) @ right_normal(self.v_p))

    def collision_heading(self) -> float | None:
        """Heading for a constant-velocity collision triangle, or ``None``.

        Solves for the pursuer heading that intercepts a target holding its
        current velocity, assuming the pursuer holds speed ``Vp``. Returns
        ``None`` when no such heading exists (target too fast to catch on a
        straight line).
        """
        if self.Vp <= 1e-6:
            return None
        # Component of target velocity across the LOS must be matched by the
        # pursuer's, which fixes the lead angle off the LOS.
        n_hat = right_normal(self.r)
        v_t_cross = float(self.v_t @ n_hat)
        s = v_t_cross / self.Vp
        if abs(s) > 1.0:
            return None
        lead = math.asin(s)
        return wrap_pi(self.lam + lead)

    def as_dict(self) -> dict:
        """Flat dict of the logged quantities (used by the CSV writers)."""
        d = {
            'R': self.R, 'Rdot': self.Rdot, 'Vc': self.Vc,
            'lam': self.lam, 'lamdot': self.lamdot,
            'alpha_p': self.alpha_p, 'alpha_t': self.alpha_t,
            'eps': self.eps, 'Vp': self.Vp, 'Vt': self.Vt,
            't_go': self.t_go,
            'zem_perp': float(np.linalg.norm(self.zem_perp)),
            'nu': self.speed_ratio,
        }
        for name, vec in (('p_p', self.p_p), ('v_p', self.v_p),
                          ('p_t', self.p_t), ('v_t', self.v_t)):
            for ax, val in zip('nedNED'[:3], vec):
                d[f'{name}_{ax}'] = float(val)
        return d
