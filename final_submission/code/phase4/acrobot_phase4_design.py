"""
Phase 4 - Swing-up: start hanging (E1), bring the acrobot to the upright E3 and hold it.

Hybrid controller (Spong 1995 / Tedrake, adapted to |tau| <= 20 N m):

  1. Swing-up (energy pumping with collocated partial feedback linearization)
       E~  = E(q, qd) - E_top,   E_top = U(E3) = +24.525 J
       v   = -k1 q2~ - k2 q2_dot + k3 q1_dot E~       (desired elbow acceleration)
       tau = (M22 - M21 M12 / M11) v + h2 - M21 h1 / M11,   h = C qd - tau_g
     The elbow follows v exactly (PFL); the k3 term pumps energy in (or out)
     until the arm has exactly the energy of the upright pose.

  2. Catch: when V = x~' S x~ < C_SWITCH (x~ measured from E3, S from the
     Phase 3 LQR), switch to the Phase 3 LQR  tau = sat(-K x~).
     If the arm leaves that region again (V >= C_SWITCH), swing-up resumes,
     so a failed catch or a later push is retried automatically.

Gains chosen by a vectorized search (see the Phase 4 results script):
k1 = 20, k2 = 2, k3 = 3, C_SWITCH = 5 -> swung up and held from 39 of 40 random
starts near hanging, first catch after ~4 s (median).
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase3"))
import acrobot_phase3_design as d3  # noqa: E402

acro = d3.acro
TAU_MAX = acro.TAU_MAX
E_TOP = 24.525                      # U at the upright E3 [J]
SWING_GAINS = (20.0, 2.0, 3.0)      # k1, k2, k3
C_SWITCH = 5.0                      # catch when x~' S x~ < C_SWITCH

K_TOP, S_TOP = d3.lqr(*acro.linearize(d3.EQ[3][1]), d3.LQR_Q, d3.LQR_R[3])


def energy(q, qd):
    return 0.5 * qd @ acro.mass_matrix(q) @ qd - 14.715 * np.cos(q[0]) - 9.81 * np.cos(q[0] + q[1])


def swing_torque(q, qd, gains=SWING_GAINS):
    """Energy-pumping swing-up through collocated partial feedback linearization."""
    k1, k2, k3 = gains
    M = acro.mass_matrix(q)
    h = acro.coriolis(q, qd) @ qd - acro.gravity_torque(q)
    v = -k1 * acro.wrap(q[1]) - k2 * qd[1] + k3 * qd[0] * (energy(q, qd) - E_TOP)
    return (M[1, 1] - M[1, 0] * M[0, 1] / M[0, 0]) * v + h[1] - M[1, 0] * h[0] / M[0, 0]


def top_error(q, qd):
    return np.array([acro.wrap(q[0] - np.pi), acro.wrap(q[1]), qd[0], qd[1]])


class SwingUpLQR:
    """Hybrid swing-up -> LQR controller. full(dq, qd) gets deviations from the
    *selected* equilibrium, so it is handed the absolute angles via q_star."""
    full_state = True

    def __init__(self, q_star, gains=SWING_GAINS, c_switch=C_SWITCH):
        self.q_star = np.asarray(q_star, float)
        self.gains, self.c_switch = gains, c_switch
        self.reset()

    def reset(self, y0=0.0):
        self.mode = "swing"
        self.t_catch = None
        self.n = 0

    def full(self, dq, qd):
        q = self.q_star + dq                     # absolute joint angles
        e = top_error(q, qd)
        self.n += 1
        if e @ S_TOP @ e < self.c_switch:
            self.mode = "lqr"
            if self.t_catch is None:
                self.t_catch = self.n * acro.DT
            u = -(K_TOP[0] @ e)
        else:
            self.mode = "swing"
            u = swing_torque(q, qd, self.gains)
        return float(np.clip(u, -TAU_MAX, TAU_MAX))

    def closed_loop_A(self, A, B):
        return A                                  # hybrid: no single linear loop to show
