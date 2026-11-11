"""
Phase 3 - controller designs and tools to measure how far from the equilibrium
each one still works (region of attraction) with |tau| <= 20 N m.

The model, linearization and PyBullet simulator are reused from Phase 2
(../phase2/acrobot_phase2_sim.py). This module adds:

    * the chosen controller designs (one place, used by the dashboard, the
      results script and mirrored in the MATLAB code)
    * a vectorized RK4 of the given equations of motion, so thousands of
      initial conditions can be simulated at once for region-of-attraction maps
"""

import os
import sys

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import place_poles

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase2"))
import acrobot_phase2_sim as acro  # noqa: E402

TAU_MAX = acro.TAU_MAX
EQ = acro.EQUILIBRIA

# ------------------------------------------------------------- chosen designs
# Justified in the report (Sec. "Design choices"): Q = I weighs the four state
# deviations equally (1 rad ~ 1 rad/s). R is the knee of the R sweep: smaller R
# barely enlarges the region of attraction but makes the fastest pole much faster.
LQR_Q = np.eye(4)
LQR_R = {2: 0.1, 3: 0.03, 4: 0.1}
PLACE_POLES = {3: [-3.0, -4.0, -5.0, -6.0]}     # comparison design at the upright
# E1: PD on the elbow angle only (tau = -Kp q2~ - Kd q2_dot). Best decay rate on a
# (Kp, Kd) grid; Kp < 0 still leaves the shaped potential U + Kp q2~^2/2 a minimum.
E1_PD = (-5.0, 0.4)


def lqr(A, B, Q, R):
    """K = R^-1 B^T S, S from the continuous algebraic Riccati equation."""
    S = solve_continuous_are(A, B, Q, np.array([[R]]))
    return (B.T @ S) / R, S


def lqr_gain(eq, R=None, Q=None):
    A, B = acro.linearize(EQ[eq][1])
    return lqr(A, B, LQR_Q if Q is None else Q, LQR_R[eq] if R is None else R)[0]


def place_gain(eq, poles=None):
    A, B = acro.linearize(EQ[eq][1])
    return place_poles(A, B, PLACE_POLES[eq] if poles is None else poles).gain_matrix


def e1_pd_gain(kp=None, kd=None):
    """Elbow PD at E1 written as a full-state gain K = [0, Kp, 0, Kd]."""
    K = np.zeros((1, 4))
    K[0, 1] = E1_PD[0] if kp is None else kp
    K[0, 3] = E1_PD[1] if kd is None else kd
    return K


# --------------------------------------------- controllers for the simulator
class FullStateFeedback:
    """tau = sat(-K x~), x~ = [dq1, dq2, dq1_dot, dq2_dot] (deviation from q*)."""
    full_state = True

    def __init__(self, K):
        self.K = np.atleast_2d(K)

    def reset(self, y0=0.0):
        pass

    def full(self, dq, qd):
        return float(np.clip(-(self.K[0] @ np.r_[dq, qd]), -TAU_MAX, TAU_MAX))

    def closed_loop_A(self, A, B):
        return A - B @ self.K


# ----------------------------------------- vectorized nonlinear simulation
def f_vec(X, tau):
    """Given equations of motion for many states at once; X is 4 x N."""
    q1, q2, d1, d2 = X
    c2, s2 = np.cos(q2), np.sin(q2)
    m11, m12, m22 = 2.663 + 2 * c2, 1.33 + c2, 1.33
    s1, s12 = np.sin(q1), np.sin(q1 + q2)
    h1 = -9.81 * (1.5 * s1 + s12) + s2 * (2 * d1 * d2 + d2 * d2)      # tau_g - C qd
    h2 = -9.81 * s12 + tau - s2 * d1 * d1
    det = m11 * m22 - m12 * m12
    return np.array([d1, d2, (m22 * h1 - m12 * h2) / det, (m11 * h2 - m12 * h1) / det])


def simulate_many(eq, K, X0, T=10.0, dt=2e-3, record_every=None):
    """RK4 closed loop tau = sat(-K x~) (zero-order hold per step) for X0 (4 x N)."""
    qs = EQ[eq][1]
    k = np.ravel(K)
    X = np.array(X0, dtype=float)
    peak = np.zeros(X.shape[1])
    hist = []
    for i in range(int(round(T / dt))):
        e = np.array([acro.wrap(X[0] - qs[0]), acro.wrap(X[1] - qs[1]), X[2], X[3]])
        tau = np.clip(-(k @ e), -TAU_MAX, TAU_MAX)
        peak = np.maximum(peak, np.abs(tau))
        k1 = f_vec(X, tau)
        k2 = f_vec(X + dt / 2 * k1, tau)
        k3 = f_vec(X + dt / 2 * k2, tau)
        k4 = f_vec(X + dt * k3, tau)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if record_every and i % record_every == 0:
            hist.append(X.copy())
    return X, peak, hist


def converged(eq, X, tol_deg=1.0, tol_rate=0.05):
    qs = EQ[eq][1]
    e = np.array([acro.wrap(X[0] - qs[0]), acro.wrap(X[1] - qs[1])])
    return (np.abs(e).max(0) < np.radians(tol_deg)) & (np.abs(X[2:]).max(0) < tol_rate)


def roa_map(eq, K, lim_deg=60.0, n=61, T=10.0, dt=2e-3, w0=(0.0, 0.0)):
    """Which initial offsets (dq1, dq2) [deg], at the given initial rates, are recovered."""
    qs = EQ[eq][1]
    g = np.linspace(-lim_deg, lim_deg, n)
    G1, G2 = np.meshgrid(np.radians(g), np.radians(g))
    N = G1.size
    X0 = np.array([qs[0] + G1.ravel(), qs[1] + G2.ravel(), np.full(N, w0[0]), np.full(N, w0[1])])
    X, peak, _ = simulate_many(eq, K, X0, T, dt)
    return converged(eq, X).reshape(n, n), g


def axis_limits(ok, g):
    """Largest symmetric offset along each axis (other offset 0) that is recovered."""
    c = len(g) // 2

    def reach(v):
        r = 0.0
        for kk in range(c, len(v)):
            if not v[kk]:
                break
            r = g[kk]
        l = 0.0
        for kk in range(c, -1, -1):
            if not v[kk]:
                break
            l = -g[kk]
        return min(r, l)
    return reach(ok[c, :]), reach(ok[:, c])


def ellipsoid_bound(eq, K, S):
    """Largest V = x^T S x level set inside |K x| <= tau_max (linear loop never saturates).
    Returns c and the offsets it guarantees along q1 and along q2 (deg)."""
    k = np.ravel(K)
    c = TAU_MAX ** 2 / (k @ np.linalg.solve(S, k))
    return c, np.degrees(np.sqrt(c / S[0, 0])), np.degrees(np.sqrt(c / S[1, 1]))
