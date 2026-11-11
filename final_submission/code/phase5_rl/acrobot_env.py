"""
RL swing-up for the acrobot: environment, dynamics and the hybrid RL -> LQR controller.

Same model, torque limit, catch rule and LQR as Phase 4; only the swing-up is
different: a policy learned with SAC instead of the energy-pumping law.

    RL policy  : obs -> tau in [-20, 20] N m, held for 20 ms
    catch      : x~' S x~ < 5 (from E3)  ->  Phase 3 LQR (K, S from R = 0.03)
    retry      : leave the catch zone -> back to the RL policy
"""

import os
import sys

import gymnasium as gym
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase4"))
import acrobot_phase4_design as d4  # noqa: E402

acro = d4.acro
TAU_MAX = 20.0
E_TOP = d4.E_TOP
K_TOP, S_TOP, C_SWITCH = d4.K_TOP, d4.S_TOP, d4.C_SWITCH
CTRL_DT = 0.02          # policy runs at 50 Hz
SUBSTEPS = 4            # RK4 steps of 5 ms inside each control step


# ------------------------------------------------ dynamics with mass parameters
def f_masses(X, tau, m1=1.0, m2=1.0):
    """Equations of motion for states X (4 x N), links of mass m1, m2 (uniform rods:
    inertias scale with mass). m1 = m2 = 1 gives exactly the project model."""
    q1, q2, d1, d2 = X
    l1, lc1, lc2 = 1.0, 0.5, 1.0
    I1, I2 = 0.083 * m1, 0.33 * m2
    c2, s2 = np.cos(q2), np.sin(q2)
    m11 = I1 + I2 + m1 * lc1 ** 2 + m2 * (l1 ** 2 + lc2 ** 2 + 2 * l1 * lc2 * c2)
    m12 = I2 + m2 * (lc2 ** 2 + l1 * lc2 * c2)
    m22 = I2 + m2 * lc2 ** 2
    s1, s12 = np.sin(q1), np.sin(q1 + q2)
    h1 = -9.81 * ((m1 * lc1 + m2 * l1) * s1 + m2 * lc2 * s12) + m2 * l1 * lc2 * s2 * (2 * d1 * d2 + d2 * d2)
    h2 = -9.81 * m2 * lc2 * s12 + tau - m2 * l1 * lc2 * s2 * d1 * d1
    det = m11 * m22 - m12 * m12
    return np.array([d1, d2, (m22 * h1 - m12 * h2) / det, (m11 * h2 - m12 * h1) / det])


def rk4(X, tau, h, m1=1.0, m2=1.0):
    k1 = f_masses(X, tau, m1, m2)
    k2 = f_masses(X + h / 2 * k1, tau, m1, m2)
    k3 = f_masses(X + h / 2 * k2, tau, m1, m2)
    k4 = f_masses(X + h * k3, tau, m1, m2)
    return X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def energy(X):
    """Energy with the nominal model (what the controllers know)."""
    q1, q2, d1, d2 = X
    c2 = np.cos(q2)
    m11, m12, m22 = 2.663 + 2 * c2, 1.33 + c2, 1.33
    return 0.5 * (m11 * d1 * d1 + 2 * m12 * d1 * d2 + m22 * d2 * d2) - 14.715 * np.cos(q1) - 9.81 * np.cos(q1 + q2)


def top_error(X):
    return np.array([acro.wrap(X[0] - np.pi), acro.wrap(X[1]), X[2], X[3]])


def in_catch_zone(X):
    e = top_error(X)
    return np.einsum("i...,ij,j...->...", e, S_TOP, e) < C_SWITCH


def observe(X, with_energy=False):
    """[cos q1, sin q1, cos q2, sin q2, q1_dot/10, q2_dot/10]: no jump at +-180 deg.

    with_energy=True (robust policy) adds one number, the missing energy
    (E_top - E)/E_top, computed with the nominal model exactly as the energy
    controller does. It is a hint, not new information: it is a function of the
    other six entries."""
    q1, q2, d1, d2 = X
    o = [np.cos(q1), np.sin(q1), np.cos(q2), np.sin(q2), d1 / 10.0, d2 / 10.0]
    if with_energy:
        o.append((E_TOP - energy(X)) / E_TOP)
    return np.array(o, dtype=np.float32)


# ---------------------------------------------------------------- environment
class AcrobotSwingUpEnv(gym.Env):
    """Start near hanging; succeed by entering the LQR catch zone around E3."""
    metadata = {"render_modes": []}

    def __init__(self, horizon_s=15.0, mass_range=None, energy_obs=False, seed=None):
        self.energy_obs = energy_obs          # robust policy: also observe the missing energy
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(7 if energy_obs else 6,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.max_steps = int(round(horizon_s / CTRL_DT))
        self.mass_range = mass_range          # e.g. (0.7, 1.3): new random link masses every episode
        self.rng = np.random.default_rng(seed)
        self.x = np.zeros(4)
        self.m = (1.0, 1.0)

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        # same start spread as the Phase 4 robustness test
        self.x = np.array([self.rng.uniform(-0.6, 0.6), self.rng.uniform(-0.6, 0.6),
                           self.rng.uniform(-0.5, 0.5), self.rng.uniform(-0.5, 0.5)])
        if options and "x0" in options:
            self.x = np.asarray(options["x0"], float)
        lo, hi = self.mass_range or (1.0, 1.0)
        self.m = (self.rng.uniform(lo, hi), self.rng.uniform(lo, hi))
        self.n = 0
        return observe(self.x, self.energy_obs), {}

    def step(self, action):
        tau = float(np.clip(action[0], -1.0, 1.0)) * TAU_MAX
        h = CTRL_DT / SUBSTEPS
        for _ in range(SUBSTEPS):
            self.x = rk4(self.x, tau, h, *self.m)
        self.n += 1
        E = energy(self.x)
        reward = (-((E - E_TOP) / E_TOP) ** 2              # pump to the upright energy
                  - 0.3 * (1.0 + np.cos(self.x[0]))        # link 1 pointing up (0 at q1 = pi)
                  - 0.1 * (1.0 - np.cos(self.x[1]))        # elbow straight
                  - 0.001 * (tau / TAU_MAX) ** 2)          # do not waste torque
        caught = bool(in_catch_zone(self.x))
        if caught:
            reward += 100.0
        if not np.all(np.isfinite(self.x)) or abs(self.x[2]) > 30 or abs(self.x[3]) > 30:
            return observe(np.nan_to_num(self.x), self.energy_obs), -100.0, True, False, {"caught": False}
        return observe(self.x, self.energy_obs), float(reward), caught, self.n >= self.max_steps, {"caught": caught}


# ------------------------------------------------------- hybrid controllers
class RLThenLQR:
    """Learned swing-up (policy held for 20 ms) -> Phase 3 LQR inside the catch zone."""

    def __init__(self, policy):
        self.policy = policy
        self.with_energy = policy.observation_space.shape[0] == 7     # robust policy
        self.reset()

    def reset(self):
        self.t_since, self.tau_rl, self.mode = CTRL_DT, 0.0, "swing"

    def __call__(self, x, dt):
        if in_catch_zone(x):
            self.mode = "lqr"
            return float(np.clip(-(K_TOP[0] @ top_error(x)), -TAU_MAX, TAU_MAX))
        self.mode = "swing"
        self.t_since += dt
        if self.t_since >= CTRL_DT - 1e-12:
            a, _ = self.policy.predict(observe(x, self.with_energy), deterministic=True)
            self.tau_rl, self.t_since = float(np.clip(a[0], -1, 1)) * TAU_MAX, 0.0
        return self.tau_rl


class EnergyThenLQR:
    """The Phase 4 controller in the same interface (energy pumping -> LQR)."""

    def __init__(self):
        self.mode = "swing"

    def reset(self):
        self.mode = "swing"

    def __call__(self, x, dt):
        c = d4.SwingUpLQR(np.zeros(2))
        tau = c.full(x[:2], x[2:])
        self.mode = c.mode
        return tau
