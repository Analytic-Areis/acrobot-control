"""
Phase 2 - Stability analysis of the Acrobot: interactive simulation.

Physics is simulated by PyBullet (headless), the interface is a live dashboard.
Model, parameters and conventions follow the Phase 1 helper / report:
    q1 from the downward vertical, q2 = elbow angle relative to link 1,
    only the elbow is actuated (|tau| <= 20 N m), no friction.

Dashboard (everything updates live while the simulation runs)
    * left    : the acrobot (solid = nonlinear PyBullet, dashed = linearized
                model, faint = target equilibrium q*). Drag a link with the
                mouse to set the initial pose.
    * middle  : joint deviations (nonlinear vs. linear) and elbow torque.
    * right   : s-plane - open-loop poles, closed-loop (linearized) poles and
                the zeros of the measured transfer function, plus a status box.
    * bottom  : equilibrium, controller, measured angle, initial condition,
                PD gains, speed; buttons Pause / Reset / Kick / Save plot.
      Controllers: open loop (tau = 0), PD on ONE joint angle,
                   observer + LQR driven by ONE joint angle.
    Keys: space = pause, r = reset (same initial condition), 0 = reset to q*,
          k = kick, s = save plot, q / Esc = quit.

Usage
    python acrobot_phase2_sim.py            # analysis printout + dashboard
    python acrobot_phase2_sim.py --verify   # PyBullet vs. helper-equation RK4 check
    python acrobot_phase2_sim.py --batch    # headless runs, saves plots to ./plots
"""

import argparse
import os
import signal
import tempfile
import time
from collections import deque

import numpy as np
import pybullet as p
from matplotlib.figure import Figure
from scipy.linalg import expm, solve_continuous_are
from scipy.signal import ss2tf

# ---------------------------------------------------------------- parameters
M1, M2 = 1.0, 1.0
L1, L2 = 1.0, 2.0
LC1, LC2 = L1 / 2, L2 / 2
I1, I2 = 0.083, 0.33
G = 9.81
TAU_MAX = 20.0

DT = 1.0 / 4000.0          # physics step [s] (PyBullet is 1st-order; error ~ DT)
HERE = os.path.dirname(os.path.abspath(__file__))

EQUILIBRIA = {
    1: ("E1 down-down", np.array([0.0, 0.0])),
    2: ("E2 down-up", np.array([0.0, np.pi])),
    3: ("E3 up-up (upright)", np.array([np.pi, 0.0])),
    4: ("E4 up-down", np.array([np.pi, np.pi])),
}
MODES = {0: "open loop (tau = 0)", 1: "single-angle PD", 2: "single-angle observer + LQR"}


# ------------------------------------------------------- nonlinear model (helper)
def mass_matrix(q):
    c2 = np.cos(q[1])
    m11 = I1 + I2 + M1 * L1**2 / 4 + M2 * (L1**2 + L2**2 / 4 + L1 * L2 * c2)
    m12 = I2 + M2 * (L2**2 / 4 + L1 * L2 / 2 * c2)
    m22 = I2 + M2 * L2**2 / 4
    return np.array([[m11, m12], [m12, m22]])


def coriolis(q, qd):
    h = M2 * L1 * L2 / 2 * np.sin(q[1])
    return np.array([[-2 * h * qd[1], -h * qd[1]], [h * qd[0], 0.0]])


def gravity_torque(q):
    s1, s12 = np.sin(q[0]), np.sin(q[0] + q[1])
    return -np.array([(M1 * LC1 + M2 * L1) * G * s1 + M2 * LC2 * G * s12,
                      M2 * LC2 * G * s12])


def energy(q, qd):
    T = 0.5 * qd @ mass_matrix(q) @ qd
    U = -(M1 * LC1 + M2 * L1) * G * np.cos(q[0]) - M2 * LC2 * G * np.cos(q[0] + q[1])
    return T + U


def f_nonlinear(x, tau):
    q, qd = x[:2], x[2:]
    rhs = gravity_torque(q) + np.array([0.0, tau]) - coriolis(q, qd) @ qd
    return np.concatenate([qd, np.linalg.solve(mass_matrix(q), rhs)])


# ------------------------------------------------------------- linearization
def linearize(q_star):
    """x~' = A x~ + B tau~ about (q*, 0), as in the Phase 1 report (P = M^-1 K)."""
    Mq = mass_matrix(q_star)
    c1, c12 = np.cos(q_star[0]), np.cos(q_star[0] + q_star[1])
    a, b = M1 * LC1 + M2 * L1, M2 * LC2          # 1.5 and 1.0 kg m
    K = -G * np.array([[a * c1 + b * c12, b * c12],
                       [b * c12, b * c12]])
    P = np.linalg.solve(Mq, K)
    r = np.linalg.solve(Mq, np.array([0.0, 1.0]))
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [P, np.zeros((2, 2))]])
    B = np.concatenate([np.zeros(2), r]).reshape(4, 1)
    return A, B


def tf_zeros(A, B, joint):
    C = np.zeros((1, 4))
    C[0, joint] = 1.0
    num = ss2tf(A, B, C, np.zeros((1, 1)))[0][0]
    num = np.trim_zeros(np.where(np.abs(num) < 1e-9, 0, num), "f")
    return np.roots(num)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def fmt_eigs(eigs, digits=2):
    out = []
    for e in sorted(eigs, key=lambda z: (round(z.real, 6), z.imag)):
        if abs(e.imag) < 1e-7:
            out.append(f"{e.real:.{digits}f}")
        elif e.imag > 0:
            re = "" if abs(e.real) < 1e-7 else f"{e.real:.{digits}f}"
            out.append(f"{re}±{e.imag:.{digits}f}j")
    return ", ".join(out)


def classify(eigs, tol=1e-7):
    if np.all(eigs.real < -tol):
        return "asymptotically stable"
    n_rhp = int(np.sum(eigs.real > tol))
    if n_rhp:
        return f"UNSTABLE ({n_rhp} RHP pole{'s' if n_rhp > 1 else ''})"
    return "marginally stable (poles on jw-axis)"


# ---------------------------------------------------------------- controllers
class OpenLoop:
    def reset(self, y0=0.0):
        pass

    def __call__(self, y, yd, dt):
        return 0.0

    def closed_loop_A(self, A, B):
        return A


class SingleAnglePD:
    """tau = -(Kp*e + Kd*e_dot), e = deviation of ONE measured joint angle."""

    def __init__(self, joint, kp, kd):
        self.j, self.kp, self.kd = joint, kp, kd

    def reset(self, y0=0.0):
        pass

    def __call__(self, y, yd, dt):
        return float(np.clip(-(self.kp * y + self.kd * yd), -TAU_MAX, TAU_MAX))

    def closed_loop_A(self, A, B):
        F = np.zeros((1, 4))
        F[0, self.j], F[0, self.j + 2] = self.kp, self.kd
        return A - B @ F


class SingleAngleObserverLQR:
    """Luenberger observer driven by ONE joint angle + LQR on the estimate."""

    # weights tuned on the nonlinear plant (largest perturbation still caught at E3:
    # ~1.4 deg measuring q1, ~0.2 deg measuring q2 - higher-bandwidth designs saturate tau)
    def __init__(self, A, B, joint, q_lqr=(10, 10, 1, 1), r_lqr=1.0,
                 q_obs=1.0, r_obs=1.0):
        self.A, self.B, self.j = A, B, joint
        self.C = np.zeros((1, 4))
        self.C[0, joint] = 1.0
        Pc = solve_continuous_are(A, B, np.diag(q_lqr), np.array([[r_lqr]]))
        self.K = (B.T @ Pc) / r_lqr
        Po = solve_continuous_are(A.T, self.C.T, q_obs * np.eye(4), np.array([[r_obs]]))
        self.L = (Po @ self.C.T) / r_obs
        self.reset()

    def reset(self, y0=0.0):
        self.xhat = np.zeros(4)      # only the measured angle is known at t = 0
        self.xhat[self.j] = y0

    def __call__(self, y, yd, dt):
        tau = float(np.clip(-(self.K @ self.xhat)[0], -TAU_MAX, TAU_MAX))
        innov = y - (self.C @ self.xhat)[0]
        self.xhat = self.xhat + dt * (self.A @ self.xhat + self.B[:, 0] * tau
                                      + self.L[:, 0] * innov)
        return tau

    def closed_loop_A(self, A, B):
        # separation principle: eig = eig(A - BK) U eig(A - LC)
        return np.block([[A - B @ self.K, B @ self.K],
                         [np.zeros((4, 4)), A - self.L @ self.C]])


def make_controller(mode, A, B, joint, kp, kd):
    if mode == 1:
        return SingleAnglePD(joint, kp, kd)
    if mode == 2:
        return SingleAngleObserverLQR(A, B, joint)
    return OpenLoop()


# ------------------------------------------------------------- Phase 2 report
def phase2_analysis():
    line = "=" * 78
    print(line + "\nPHASE 2 - STABILITY ANALYSIS (linearized about each equilibrium, tau* = 0)\n" + line)
    for k, (name, qs) in EQUILIBRIA.items():
        A, B = linearize(qs)
        eigs = np.linalg.eigvals(A)
        print(f"\n{name}   q* = ({np.degrees(qs[0]):.0f} deg, {np.degrees(qs[1]):.0f} deg)")
        print(f"  eig(A)              : {fmt_eigs(eigs, 4)}  ->  {classify(eigs)}")
        for j in (0, 1):
            print(f"  G{j + 1}(s) poles / zeros: {fmt_eigs(eigs, 4)}  /  "
                  f"{fmt_eigs(tf_zeros(A, B, j), 4)}")
        if k == 1:
            print("  Nonlinear: linearization is only marginal (centre), so it is inconclusive.\n"
                  "  U(q) has a strict minimum here and T+U is conserved -> E = T + U - U* is a\n"
                  "  Lyapunov function (dV/dt = 0): stable in the sense of Lyapunov, NOT\n"
                  "  asymptotically stable (no friction: it oscillates forever).")
        elif k == 3:
            print("  Nonlinear: a RHP eigenvalue of the linearization -> unstable (Lyapunov's\n"
                  "  indirect method).")
        else:
            print("  Nonlinear: saddle (one RHP real pole) -> unstable (Lyapunov indirect method).")

    # ---- single-angle feedback at the upright equilibrium
    A, B = linearize(EQUILIBRIA[3][1])
    print("\n" + line + "\nE3: CAN FEEDBACK FROM A SINGLE MEASURED JOINT ANGLE STABILIZE IT?\n" + line)
    den = np.poly(np.linalg.eigvals(A)).real
    for j in (0, 1):
        C = np.zeros((1, 4))
        C[0, j] = 1.0
        num = ss2tf(A, B, C, np.zeros((1, 1)))[0][0]
        b2, b0 = num[2], num[4]
        a2, a0 = den[2], den[4]
        obs_rank = np.linalg.matrix_rank(np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(4)]))
        zeros = tf_zeros(A, B, j)
        poles = np.linalg.eigvals(A)
        # parity interlacing: real RHP zeros incl. infinity (relative degree 2)
        zr = sorted([z.real for z in zeros if abs(z.imag) < 1e-9 and z.real >= 0]) + [np.inf]
        pr = [pl.real for pl in poles if abs(pl.imag) < 1e-9 and pl.real > 0]
        pip_ok = all(sum(a < x < b for x in pr) % 2 == 0 for a, b in zip(zr[:-1], zr[1:]))
        print(f"\nMeasure q{j + 1}:  G{j + 1}(s) = ({b2:.4f} s^2 + {b0:.4f}) / "
              f"(s^4 + {a2:.4f} s^2 + {a0:.4f})")
        print(f"  PD  tau = -(Kp + Kd s) q{j + 1}~  ->  closed-loop characteristic polynomial")
        print(f"     s^4 + ({b2:.4f} Kd) s^3 + ({a2:.4f} + {b2:.4f} Kp) s^2 + ({b0:.4f} Kd) s"
              f" + ({a0:.4f} + {b0:.4f} Kp)")
        print(f"     s^3 and s^1 coefficients need Kd*{b2:.3f} > 0 AND Kd*{b0:.3f} > 0 -> "
              f"{'possible' if b2 * b0 > 0 else 'IMPOSSIBLE (opposite signs)'}")
        if b2 * b0 < 0:
            print("     => no static/PD gain from this angle can stabilize E3 "
                  "(real RHP zeros at ±%.4f)." % abs(np.sqrt(-b0 / b2)))
        print(f"  Observability rank of (A, C) = {obs_rank}/4 -> "
              f"{'observable: a DYNAMIC (observer-based) controller CAN stabilize' if obs_rank == 4 else 'not observable'}")
        print(f"  Parity interlacing property: {'satisfied -> a STABLE compensator suffices' if pip_ok else 'violated -> any stabilizing compensator must itself be UNSTABLE'}")
    print("\nSimulation: controller 1 (PD) always falls at E3; controller 2 (observer + LQR,\n"
          "same single angle) holds it, but only for tiny perturbations\n"
          "(~1.4 deg via q1, ~0.2 deg via q2) because tau_max = 20 N m saturates.\n")


# --------------------------------------------------------------------- URDF
def write_urdf():
    def link(name, mass, length, inertia):
        return f"""
  <link name="{name}">
    <inertial>
      <origin xyz="0 0 {-length / 2}"/>
      <mass value="{mass}"/>
      <inertia ixx="{inertia}" iyy="{inertia}" izz="1e-4" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>"""

    urdf = f"""<?xml version="1.0"?>
<robot name="acrobot">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  {link("link1", M1, L1, I1)}
  {link("link2", M2, L2, I2)}
  <joint name="shoulder" type="continuous">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 -1 0"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="elbow" type="continuous">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 {-L1}"/><axis xyz="0 -1 0"/>
    <dynamics damping="0" friction="0"/>
  </joint>
</robot>"""
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix="acrobot_")
    with os.fdopen(fd, "w") as fh:
        fh.write(urdf)
    return path


def load_acrobot(cid):
    path = write_urdf()
    body = p.loadURDF(path, [0, 0, 0], useFixedBase=True, physicsClientId=cid,
                      flags=p.URDF_USE_INERTIA_FROM_FILE)
    os.remove(path)
    for j in (-1, 0, 1):
        p.changeDynamics(body, j, linearDamping=0, angularDamping=0, jointDamping=0,
                         physicsClientId=cid)
    # disable the default velocity motors -> joints are free (shoulder stays passive)
    p.setJointMotorControlArray(body, [0, 1], p.VELOCITY_CONTROL, forces=[0, 0],
                                physicsClientId=cid)
    return body


# ---------------------------------------------------------------- simulator
class AcrobotSim:
    def __init__(self):
        self.cid = p.connect(p.DIRECT)
        p.setGravity(0, 0, -G, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1, physicsClientId=self.cid)
        self.robot = load_acrobot(self.cid)
        self.configure(1, (20.0, -10.0), 0, 0, 0.0, 0.0)

    # -- setup
    def configure(self, eq, dq_deg, mode, joint, kp, kd, w0_deg=(0.0, 0.0)):
        self.eq, self.mode, self.joint = eq, mode, joint
        self.name, self.q_star = EQUILIBRIA[eq]
        self.A, self.B = linearize(self.q_star)
        Phi = expm(np.block([[self.A, self.B], [np.zeros((1, 5))]]) * DT)
        self.Ad, self.Bd = Phi[:4, :4], Phi[:4, 4]
        self.ctrl = make_controller(mode, self.A, self.B, joint, kp, kd)
        self.ctrl_lin = make_controller(mode, self.A, self.B, joint, kp, kd)
        self.dq0 = np.radians(dq_deg)
        self.w0 = np.radians(w0_deg)
        self.reset()

    def set_gains(self, kp, kd):
        for c in (self.ctrl, self.ctrl_lin):
            if isinstance(c, SingleAnglePD):
                c.kp, c.kd = kp, kd

    def reset(self):
        q0 = self.q_star + self.dq0
        for j in (0, 1):
            p.resetJointState(self.robot, j, q0[j], self.w0[j], physicsClientId=self.cid)
        self.x_lin = np.concatenate([self.dq0, self.w0])
        self.lin_ok = True
        self.ctrl.reset(self.dq0[self.joint])
        self.ctrl_lin.reset(self.dq0[self.joint])
        self.t, self.tau, self.n = 0.0, 0.0, 0
        self.log = {k: [] for k in ("t", "q1", "q2", "q1_lin", "q2_lin", "tau", "E")}
        self.E0 = energy(*self.state())
        self._record()

    def kick(self, dqd1=0.8):
        """Velocity disturbance on the (passive) shoulder."""
        q, qd = self.state()
        p.resetJointState(self.robot, 0, q[0], qd[0] + dqd1, physicsClientId=self.cid)
        if self.lin_ok:
            self.x_lin[2] += dqd1

    # -- state
    def state(self):
        js = p.getJointStates(self.robot, [0, 1], physicsClientId=self.cid)
        return np.array([s[0] for s in js]), np.array([s[1] for s in js])

    def deviation(self):
        q, qd = self.state()
        return wrap(q - self.q_star), qd

    # -- one physics step
    def step(self):
        dq, qd = self.deviation()
        self.tau = self.ctrl(dq[self.joint], qd[self.joint], DT)
        p.setJointMotorControl2(self.robot, 1, p.TORQUE_CONTROL, force=self.tau,
                                physicsClientId=self.cid)
        p.stepSimulation(physicsClientId=self.cid)

        if self.lin_ok:
            x = self.x_lin
            tau_l = self.ctrl_lin(x[self.joint], x[self.joint + 2], DT)
            self.x_lin = self.Ad @ x + self.Bd * tau_l
            if np.max(np.abs(self.x_lin[:2])) > np.pi:
                self.lin_ok = False      # linear model left its region of validity
        self.t += DT
        self.n += 1
        if self.n % 20 == 0:
            self._record()

    def _record(self):
        dq, qd = self.deviation()
        lg = self.log
        lg["t"].append(self.t)
        lg["q1"].append(np.degrees(dq[0]))
        lg["q2"].append(np.degrees(dq[1]))
        lg["q1_lin"].append(np.degrees(self.x_lin[0]) if self.lin_ok else np.nan)
        lg["q2_lin"].append(np.degrees(self.x_lin[1]) if self.lin_ok else np.nan)
        lg["tau"].append(self.tau)
        lg["E"].append(energy(*self.state()))

    def closed_loop_eigs(self):
        return np.linalg.eigvals(self.ctrl.closed_loop_A(self.A, self.B))

    def describe(self):
        ctrl = MODES[self.mode]
        if self.mode == 1:
            ctrl += f" on q{self.joint + 1} (Kp={self.ctrl.kp:.0f}, Kd={self.ctrl.kd:.0f})"
        elif self.mode == 2:
            ctrl += f", measuring q{self.joint + 1}"
        return ctrl


# ------------------------------------------------------------------ plotting
def break_wraps(y, jump=180.0):
    """Insert NaNs where a wrapped angle jumps by ~360 deg so lines are not drawn across."""
    y = np.asarray(y, dtype=float).copy()
    if y.size > 1:
        y[1:][np.abs(np.diff(y)) > jump] = np.nan
    return y


def save_plot(sim, tag=""):
    lg = {k: np.asarray(v) for k, v in sim.log.items()}
    fig = Figure(figsize=(9, 9))
    ax = fig.subplots(4, 1, sharex=True)
    for i, key in enumerate(("q1", "q2")):
        ax[i].plot(lg["t"], break_wraps(lg[key]), label=f"Δ{key} nonlinear (PyBullet)")
        ax[i].plot(lg["t"], lg[key + "_lin"], "k--", label=f"Δ{key} linearized")
        ax[i].set_ylabel(f"{key} - {key}* [deg]")
        ax[i].legend(loc="upper right")
        ax[i].grid(alpha=0.3)
    ax[2].plot(lg["t"], lg["tau"], color="tab:red")
    ax[2].axhline(TAU_MAX, color="gray", ls=":")
    ax[2].axhline(-TAU_MAX, color="gray", ls=":")
    ax[2].set_ylabel("tau [N m]")
    ax[2].grid(alpha=0.3)
    ax[3].plot(lg["t"], lg["E"] - lg["E"][0], color="tab:green")
    ax[3].set_ylabel("E(t) - E(0) [J]")
    ax[3].set_xlabel("time [s]")
    ax[3].grid(alpha=0.3)
    eigs = sim.closed_loop_eigs()
    fig.suptitle(f"{sim.name} | {sim.describe()}\n"
                 f"dq0 = ({np.degrees(sim.dq0[0]):.1f}, {np.degrees(sim.dq0[1]):.1f}) deg | "
                 f"linearized loop: {classify(eigs)}", fontsize=10)
    fig.tight_layout()
    out = os.path.join(HERE, "plots")
    os.makedirs(out, exist_ok=True)
    fname = os.path.join(out, f"E{sim.eq}_mode{sim.mode}{tag}_{time.strftime('%H%M%S')}.png")
    fig.savefig(fname, dpi=130)
    print(f"saved {fname}")
    return fname


# ------------------------------------------------------------------ dashboard
COL = dict(link1="#2563eb", link2="#f97316", ghost="#475569", target="#cbd5e1",
           trail="#ef4444", tau="#7c3aed", ok="#15803d", bad="#b91c1c", warn="#b45309",
           q1="#2563eb", q2="#f97316", panel="#f8fafc")


class Dashboard:
    WINDOW = 10.0      # seconds shown in the time plots
    TRAIL = 1.5        # seconds of tip trail

    def __init__(self):
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, Slider, TextBox

        for k in ("keymap.home", "keymap.pan", "keymap.save", "keymap.xscale",
                  "keymap.yscale", "keymap.fullscreen", "keymap.quit"):
            matplotlib.rcParams[k] = []
        matplotlib.rcParams.update({"toolbar": "None", "font.size": 9,
                                    "axes.titlesize": 10, "axes.titleweight": "bold"})
        self.plt = plt
        self.sim = AcrobotSim()
        self.paused = False
        self.drag = None          # None | 0 (dragging link 1) | 1 (dragging link 2)
        self.drag_q = None
        self.mute = False         # suppress widget callbacks while syncing them
        self.trail = deque()
        self.last_wall = time.perf_counter()
        self.full_redraw = True
        self.background = None
        self.show_ghost = True

        fig = self.fig = plt.figure(figsize=(13.5, 8.2), facecolor="white")
        try:
            fig.canvas.manager.set_window_title("Acrobot - Phase 2 stability explorer")
        except AttributeError:
            pass

        # ---------- acrobot view
        ax = self.ax_bot = fig.add_axes([0.02, 0.30, 0.36, 0.64])
        ax.set_xlim(-3.3, 3.3)
        ax.set_ylim(-3.3, 3.3)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor(COL["panel"])
        ax.set_title("Acrobot  -  drag a link to set the initial pose", loc="left")
        ax.axvline(0, color="#e2e8f0", lw=1, zorder=0)
        ax.axhline(0, color="#e2e8f0", lw=1, zorder=0)
        ax.add_patch(matplotlib.patches.Circle((0, 0), L1 + L2, fill=False, ls=":",
                                               ec="#e2e8f0", zorder=0))
        ax.plot([-0.25, 0.25], [0.12, 0.12], color="#334155", lw=4, solid_capstyle="round")
        ax.plot([0, -0.15, 0.15, 0], [0, 0.12, 0.12, 0], color="#334155", lw=1.5)
        self.h_target, = ax.plot([], [], color=COL["target"], lw=7, alpha=0.6,
                                 solid_capstyle="round", zorder=1, label="equilibrium q*")
        self.h_trail, = ax.plot([], [], color=COL["trail"], lw=1.5, alpha=0.45, zorder=2)
        self.h_ghost, = ax.plot([], [], color=COL["ghost"], lw=2, ls="--", marker="o",
                                ms=4, zorder=3, label="linearized model (dashed)")
        self.h_l1, = ax.plot([], [], color=COL["link1"], lw=8, solid_capstyle="round",
                             zorder=4, label="nonlinear (PyBullet)")
        self.h_l2, = ax.plot([], [], color=COL["link2"], lw=8, solid_capstyle="round", zorder=4)
        self.h_joints, = ax.plot([], [], "o", color="#0f172a", ms=8, zorder=5)
        self.h_tau, = ax.plot([], [], color=COL["tau"], lw=3, zorder=6)
        self.h_tau_tip, = ax.plot([], [], "o", color=COL["tau"], ms=6, zorder=6)
        ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
        self.h_clock = ax.text(0.98, 0.97, "", transform=ax.transAxes, ha="right", va="top",
                               fontsize=11, family="monospace")

        # ---------- time plots
        aq = self.ax_q = fig.add_axes([0.43, 0.58, 0.33, 0.36])
        aq.set_title("Joint deviation from q*", loc="left")
        aq.set_ylabel("deg")
        aq.grid(alpha=0.3)
        self.h_q1, = aq.plot([], [], color=COL["q1"], lw=1.8, label="Δq1 nonlinear")
        self.h_q2, = aq.plot([], [], color=COL["q2"], lw=1.8, label="Δq2 nonlinear")
        self.h_q1l, = aq.plot([], [], color=COL["q1"], lw=1.2, ls="--", alpha=0.8, label="Δq1 linear")
        self.h_q2l, = aq.plot([], [], color=COL["q2"], lw=1.2, ls="--", alpha=0.8, label="Δq2 linear")
        aq.legend(loc="upper left", ncol=2, fontsize=8)
        aq.tick_params(labelbottom=False)

        at = self.ax_tau = fig.add_axes([0.43, 0.33, 0.33, 0.21], sharex=aq)
        at.set_title("Elbow torque", loc="left")
        at.set_ylabel("N m  (limit ±20)")
        at.set_xlabel("time [s]")
        at.set_ylim(-1.15 * TAU_MAX, 1.15 * TAU_MAX)
        at.grid(alpha=0.3)
        for s in (-1, 1):
            at.axhline(s * TAU_MAX, color=COL["bad"], ls=":", lw=1)
        self.h_taut, = at.plot([], [], color=COL["tau"], lw=1.5)

        # ---------- s-plane
        ap = self.ax_pz = fig.add_axes([0.82, 0.58, 0.165, 0.335])
        ap.set_title("s-plane (linearized)", loc="left")
        ap.axhline(0, color="#94a3b8", lw=0.8)
        ap.axvline(0, color="#94a3b8", lw=0.8)
        ap.grid(alpha=0.3)
        self.rhp = ap.axvspan(0, 100, color=COL["bad"], alpha=0.07, lw=0)
        self.h_olp, = ap.plot([], [], "x", color="#94a3b8", ms=9, mew=2, label="open-loop poles")
        self.h_zer, = ap.plot([], [], "o", mfc="none", mec="#0f172a", ms=8, mew=1.5,
                              label="zeros of G (measured)")
        self.h_clp, = ap.plot([], [], "x", ms=10, mew=2.5, label="closed-loop poles")
        self.pz_legend = ap.legend(loc="lower left", fontsize=7, framealpha=0.9)
        ap.set_xlabel("Re")
        ap.set_ylabel("Im")

        # ---------- status box
        ai = self.ax_info = fig.add_axes([0.815, 0.30, 0.175, 0.22])
        ai.axis("off")
        self.h_info = ai.text(0.0, 1.0, "", va="top", ha="left", fontsize=8.5,
                              family="monospace", linespacing=1.5)
        self.h_verdict = ai.text(0.0, 0.0, "", va="bottom", ha="left", fontsize=10,
                                 weight="bold", wrap=True)

        # ---------- controls
        def panel(rect, title):
            a = fig.add_axes(rect, facecolor=COL["panel"])
            for sp in a.spines.values():
                sp.set_color("#cbd5e1")
            fig.text(rect[0], rect[1] + rect[3] + 0.012, title, fontsize=10, weight="bold")
            return a

        def radios(rect, title, labels):
            return RadioButtons(panel(rect, title), labels, active=0, activecolor=COL["link1"],
                                useblit=False, label_props={"fontsize": [9.5] * len(labels)},
                                radio_props={"s": [60] * len(labels)})

        self.rb_eq = radios([0.015, 0.04, 0.095, 0.19], "Equilibrium",
                            ["E1 down-down", "E2 down-up", "E3 up-up", "E4 up-down"])
        self.rb_mode = radios([0.125, 0.04, 0.12, 0.19], "Controller",
                              ["Open loop (τ = 0)", "PD on one angle", "Observer + LQR"])
        self.rb_joint = radios([0.26, 0.04, 0.085, 0.19], "Measured angle",
                               ["q1 shoulder", "q2 elbow"])

        # every value has a slider (where useful) AND a box you can type into + Enter
        self.vals = {"dq1": 20.0, "dq2": -10.0, "w1": 0.0, "w2": 0.0,
                     "kp": 0.0, "kd": 0.0, "speed": 1.0}
        self.sliders, self.boxes, self.row_labels = {}, {}, {}

        def row(key, label, x0, y, lo=None, hi=None, step=None, color=None):
            self.row_labels[key] = fig.text(x0 + 0.075, y + 0.0125, label, ha="right",
                                            va="center", fontsize=9.5)
            bx = x0 + 0.085
            if lo is not None:
                s = Slider(fig.add_axes([x0 + 0.085, y, 0.135, 0.025]), "", lo, hi,
                           valinit=self.vals[key], valstep=step, color=color, initcolor="none")
                s.valtext.set_visible(False)
                s.on_changed(lambda v, k=key: self.on_slider(k, v))
                self.sliders[key] = s
                bx = x0 + 0.228
            tb = TextBox(fig.add_axes([bx, y - 0.005, 0.052, 0.035]), "",
                         initial=self.fmt(key), textalignment="center",
                         color="white", hovercolor="#f1f5f9")
            tb.text_disp.set_fontsize(9.5)
            tb.on_submit(lambda txt, k=key: self.on_text(k, txt))
            self.boxes[key] = tb

        g1, g2, ys = 0.365, 0.685, (0.185, 0.135, 0.085)
        fig.text(g1, 0.243, "Initial condition (Δ from q*)", fontsize=10, weight="bold")
        fig.text(g2, 0.243, "PD gains & speed", fontsize=10, weight="bold")
        row("dq1", "Δq1(0) [°]", g1, ys[0], -180, 180, 0.5, COL["link1"])
        row("dq2", "Δq2(0) [°]", g1, ys[1], -180, 180, 0.5, COL["link2"])
        row("w1", "ω1(0) [°/s]", g1, ys[2])
        row("w2", "ω2(0) [°/s]", g1 + 0.145, ys[2])
        row("kp", "Kp [N m/rad]", g2, ys[0], -300, 300, 1, COL["tau"])
        row("kd", "Kd [N m s/rad]", g2, ys[1], -100, 100, 0.5, COL["tau"])
        row("speed", "speed [× real]", g2, ys[2], 0.05, 2.0, 0.05, "#64748b")

        def button(rect, label):
            b = Button(fig.add_axes(rect), label, color="#e2e8f0", hovercolor="#cbd5e1")
            b.label.set_fontsize(10)
            return b

        bx0, gap = 0.365, 0.008
        bw = (0.985 - bx0 - 5 * gap) / 6
        bx = [bx0 + i * (bw + gap) for i in range(6)]
        self.b_pause = button([bx[0], 0.012, bw, 0.045], "Pause")
        self.b_reset = button([bx[1], 0.012, bw, 0.045], "Reset")
        self.b_zero = button([bx[2], 0.012, bw, 0.045], "Reset to q*")
        self.b_kick = button([bx[3], 0.012, bw, 0.045], "Kick shoulder")
        self.b_ghost = button([bx[4], 0.012, bw, 0.045], "Hide linear")
        self.b_save = button([bx[5], 0.012, bw, 0.045], "Save plot")
        fig.text(0.015, 0.962, "Acrobot - Phase 2 stability explorer", fontsize=14, weight="bold")
        fig.text(0.985, 0.965, "physics: PyBullet  ·  type a value + Enter  ·  keys: space pause, r reset, 0 reset to q*, k kick, s save, q quit",
                 fontsize=8.5, color="#64748b", ha="right")

        # ---------- callbacks
        for rb in (self.rb_eq, self.rb_mode, self.rb_joint):
            rb.on_clicked(lambda _: self.reconfigure())
        self.b_pause.on_clicked(lambda _: self.toggle_pause())
        self.b_reset.on_clicked(lambda _: self.restart())
        self.b_kick.on_clicked(lambda _: self.sim.kick())
        self.b_zero.on_clicked(lambda _: self.go_to_equilibrium())
        self.b_ghost.on_clicked(lambda _: self.toggle_ghost())
        self.b_save.on_clicked(lambda _: save_plot(self.sim))
        fig.canvas.mpl_connect("key_press_event", self.on_key)
        fig.canvas.mpl_connect("button_press_event", self.on_press)
        fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        fig.canvas.mpl_connect("button_release_event", self.on_release)

        self.reconfigure()

    # ---------------------------------------------------------------- actions
    def selection(self):
        eq = ["E1", "E2", "E3", "E4"].index(self.rb_eq.value_selected[:2]) + 1
        mode = ["Open", "PD o", "Obse"].index(self.rb_mode.value_selected[:4])
        joint = 0 if self.rb_joint.value_selected.startswith("q1") else 1
        return eq, mode, joint

    def fmt(self, key):
        return f"{self.vals[key]:g}"

    def show_value(self, key):
        """Push vals[key] into its slider and text box without firing callbacks."""
        self.mute = True
        if key in self.sliders:
            s = self.sliders[key]
            s.set_val(float(np.clip(self.vals[key], s.valmin, s.valmax)))
        tb = self.boxes[key]
        tb.set_val(self.fmt(key))
        tb.cursor.set_visible(tb.capturekeystrokes)   # set_val would leave a stray cursor
        self.mute = False

    def on_slider(self, key, v):
        if self.mute:
            return
        self.vals[key] = float(v)
        self.show_value(key)
        self.apply(key)

    def on_text(self, key, txt):
        if self.mute:
            return
        try:
            v = float(txt.replace("°", "").strip())
        except ValueError:
            self.show_value(key)          # not a number -> restore the old value
            return
        if key in ("dq1", "dq2"):
            v = round(float(np.degrees(wrap(np.radians(v)))), 6)
        elif key == "speed":
            v = float(np.clip(v, 0.01, 5.0))
        self.vals[key] = v
        self.show_value(key)
        self.apply(key)

    def apply(self, key):
        if key in ("kp", "kd"):
            self.update_gains()
        elif key != "speed":
            self.reconfigure()

    def reconfigure(self):
        if self.mute:
            return
        eq, mode, joint = self.selection()
        v = self.vals
        self.sim.configure(eq, (v["dq1"], v["dq2"]), mode, joint, v["kp"], v["kd"],
                           w0_deg=(v["w1"], v["w2"]))
        self.trail.clear()
        self.last_wall = time.perf_counter()
        self.refresh_poles()

    def restart(self):
        self.sim.reset()
        self.trail.clear()

    def update_gains(self):
        self.sim.set_gains(self.vals["kp"], self.vals["kd"])
        self.refresh_poles()

    def set_offsets(self, d1, d2):
        self.vals["dq1"], self.vals["dq2"] = round(float(d1), 1), round(float(d2), 1)
        self.show_value("dq1")
        self.show_value("dq2")
        self.reconfigure()

    def go_to_equilibrium(self):
        """Initial condition = exactly q* at rest (it then stays there unless kicked)."""
        for k in ("dq1", "dq2", "w1", "w2"):
            self.vals[k] = 0.0
            self.show_value(k)
        self.reconfigure()

    def toggle_ghost(self):
        self.show_ghost = not self.show_ghost
        self.b_ghost.label.set_text("Hide linear" if self.show_ghost else "Show linear")
        self.full_redraw = True

    def toggle_pause(self):
        self.paused = not self.paused
        self.b_pause.label.set_text("Resume" if self.paused else "Pause")
        self.full_redraw = True
        self.last_wall = time.perf_counter()

    def on_key(self, ev):
        if any(tb.capturekeystrokes for tb in self.boxes.values()):
            return                         # typing into a box, not a shortcut
        if ev.key == " ":
            self.toggle_pause()
        elif ev.key == "r":
            self.restart()
        elif ev.key == "0":
            self.go_to_equilibrium()
        elif ev.key in ("q", "escape"):
            self.plt.close(self.fig)
        elif ev.key == "k":
            self.sim.kick()
        elif ev.key == "s":
            save_plot(self.sim)

    # ------------------------------------------------------------ mouse drag
    def on_press(self, ev):
        if ev.inaxes is not self.ax_bot or ev.xdata is None:
            return
        q, _ = self.sim.state()
        elbow, tip = self.points(q)[1:]
        click = np.array([ev.xdata, ev.ydata])

        def dist_to_seg(a, b):
            t = np.clip(np.dot(click - a, b - a) / np.dot(b - a, b - a), 0, 1)
            return np.linalg.norm(click - (a + t * (b - a)))

        d1, d2 = dist_to_seg(np.zeros(2), elbow), dist_to_seg(elbow, tip)
        if min(d1, d2) < 0.4:
            self.drag = 0 if d1 < d2 else 1
            self.drag_q = q.copy()
            self.on_motion(ev)

    def on_motion(self, ev):
        if self.drag is None or ev.inaxes is not self.ax_bot or ev.xdata is None:
            return
        x, y = ev.xdata, ev.ydata
        if self.drag == 0:
            self.drag_q[0] = np.arctan2(x, -y)
        else:
            ex, ey = L1 * np.sin(self.drag_q[0]), -L1 * np.cos(self.drag_q[0])
            self.drag_q[1] = np.arctan2(x - ex, -(y - ey)) - self.drag_q[0]

    def on_release(self, ev):
        if self.drag is None:
            return
        dq = np.degrees(wrap(self.drag_q - self.sim.q_star))
        self.drag, self.drag_q = None, None
        self.set_offsets(dq[0], dq[1])

    # ------------------------------------------------------------- drawing
    @staticmethod
    def points(q):
        elbow = np.array([L1 * np.sin(q[0]), -L1 * np.cos(q[0])])
        tip = elbow + [L2 * np.sin(q[0] + q[1]), -L2 * np.cos(q[0] + q[1])]
        return np.zeros(2), elbow, tip

    def refresh_poles(self):
        sim = self.sim
        ol = np.linalg.eigvals(sim.A)
        cl = sim.closed_loop_eigs()
        z = tf_zeros(sim.A, sim.B, sim.joint)
        stable = np.all(cl.real < -1e-7)
        self.h_olp.set_data(ol.real, ol.imag)
        self.h_zer.set_data(z.real, z.imag)
        self.h_clp.set_data(cl.real, cl.imag)
        self.h_clp.set_color("#cbd5e1" if sim.mode == 0 else (COL["ok"] if stable else COL["bad"]))
        self.h_clp.set_visible(sim.mode != 0)
        self.pz_legend.get_texts()[1].set_text(f"zeros of G{sim.joint + 1} (q{sim.joint + 1} measured)")
        pts = np.concatenate([ol, cl, z])
        lim = float(np.clip(1.15 * np.max(np.abs(np.concatenate([pts.real, pts.imag]))), 3, 40))
        self.ax_pz.set_xlim(-lim, lim)
        self.ax_pz.set_ylim(-lim, lim)

        verdict = classify(cl)
        if sim.mode == 0:
            msg, col = f"Open loop:\n{verdict}", (COL["ok"] if "stable" in verdict and "UN" not in verdict else COL["bad"])
            if sim.eq == 1:
                msg, col = "Open loop: marginally stable\n(Lyapunov-stable via energy)", COL["warn"]
        else:
            msg = ("Closed loop (linear):\n" + verdict) if stable else ("Closed loop (linear):\n" + verdict)
            col = COL["ok"] if stable else COL["bad"]
        self.h_verdict.set_text(msg)
        self.h_verdict.set_color(col)

        on = sim.mode == 1
        for k in ("kp", "kd"):     # gains only matter for the PD controller
            self.row_labels[k].set_color("#0f172a" if on else "#94a3b8")
            self.sliders[k].poly.set_alpha(1.0 if on else 0.3)
        _, et, tt = self.points(sim.q_star)
        self.h_target.set_data([0, et[0], tt[0]], [0, et[1], tt[1]])
        self.full_redraw = True

    def frame(self, _):
        sim = self.sim
        now = time.perf_counter()
        elapsed, self.last_wall = min(now - self.last_wall, 0.1), now
        if not self.paused and self.drag is None:
            for _ in range(int(round(self.vals["speed"] * elapsed / DT))):
                sim.step()

        # acrobot
        if self.drag is not None:
            q = self.drag_q
            for j in (0, 1):
                p.resetJointState(sim.robot, j, q[j], 0.0, physicsClientId=sim.cid)
        else:
            q, _ = sim.state()
        o, e, tip = self.points(q)
        self.h_l1.set_data([o[0], e[0]], [o[1], e[1]])
        self.h_l2.set_data([e[0], tip[0]], [e[1], tip[1]])
        self.h_joints.set_data([o[0], e[0]], [o[1], e[1]])
        if self.show_ghost and sim.lin_ok and self.drag is None:
            _, el, tl = self.points(sim.q_star + sim.x_lin[:2])
            self.h_ghost.set_data([0, el[0], tl[0]], [0, el[1], tl[1]])
        else:
            self.h_ghost.set_data([], [])

        self.trail.append((sim.t, *tip))
        while self.trail and self.trail[0][0] < sim.t - self.TRAIL:
            self.trail.popleft()
        tr = np.array(self.trail)
        self.h_trail.set_data(tr[:, 1], tr[:, 2])

        # torque arc around the elbow (length ~ |tau| / tau_max)
        frac = sim.tau / TAU_MAX
        if abs(frac) > 0.01:
            base = np.arctan2(tip[1] - e[1], tip[0] - e[0])
            th = base + np.linspace(0, frac * 1.6 * np.pi, 30)
            self.h_tau.set_data(e[0] + 0.35 * np.cos(th), e[1] + 0.35 * np.sin(th))
            self.h_tau_tip.set_data([e[0] + 0.35 * np.cos(th[-1])], [e[1] + 0.35 * np.sin(th[-1])])
        else:
            self.h_tau.set_data([], [])
            self.h_tau_tip.set_data([], [])
        state = "DRAG" if self.drag is not None else ("PAUSED" if self.paused else "")
        self.h_clock.set_text(f"t = {sim.t:6.2f} s  {state}")

        # time plots (last WINDOW seconds)
        lg = sim.log
        t = np.asarray(lg["t"])
        i0 = int(np.searchsorted(t, sim.t - self.WINDOW))
        t = t[i0:]
        series = {k: np.asarray(lg[k][i0:]) for k in ("q1", "q2", "q1_lin", "q2_lin", "tau")}
        self.h_q1.set_data(t, break_wraps(series["q1"]))
        self.h_q2.set_data(t, break_wraps(series["q2"]))
        self.h_q1l.set_data(t, series["q1_lin"])
        self.h_q2l.set_data(t, series["q2_lin"])
        self.h_taut.set_data(t, series["tau"])
        # axis limits move in steps; each change costs one full redraw, other frames blit
        lo, hi = self.ax_q.get_xlim()
        if sim.t > hi or sim.t < lo:
            lo = max(0.0, sim.t - self.WINDOW + 3.0) if sim.t > self.WINDOW else 0.0
            self.ax_q.set_xlim(lo, lo + self.WINDOW)
            self.full_redraw = True
        allq = np.concatenate([series["q1"], series["q2"], series["q1_lin"], series["q2_lin"]])
        need = 1.1 * np.nanmax(np.abs(allq)) if np.any(np.isfinite(allq)) else 1.0
        cur = self.ax_q.get_ylim()[1]
        if need > cur or need < cur / 3:
            span = next((s for s in (2, 5, 10, 20, 45, 90, 200) if s >= need), 200)
            if span != cur:
                self.ax_q.set_ylim(-span, span)
                self.full_redraw = True

        # status
        dq, qd = sim.deviation()
        far = np.max(np.abs(dq)) > np.radians(30)
        self.h_info.set_text(
            f"{sim.name}\n"
            f"q* = ({np.degrees(sim.q_star[0]):.0f}°, {np.degrees(sim.q_star[1]):.0f}°)\n"
            f"Δq1 = {np.degrees(dq[0]):+8.2f}°\n"
            f"Δq2 = {np.degrees(dq[1]):+8.2f}°\n"
            f"τ   = {sim.tau:+7.2f} N m{'  SAT' if abs(sim.tau) >= TAU_MAX - 1e-6 else ''}\n"
            f"E-E0= {energy(*sim.state()) - sim.E0:+8.3f} J\n"
            f"{'left |Δq| < 30° region' if far else 'near equilibrium'}"
            f"{'' if sim.lin_ok else chr(10) + 'linear model diverged'}")

    # ------------------------------------------------------------- blitting
    def animated_artists(self):
        return [self.h_trail, self.h_ghost, self.h_l1, self.h_l2, self.h_joints, self.h_tau,
                self.h_tau_tip, self.h_clock, self.h_q1, self.h_q2, self.h_q1l, self.h_q2l,
                self.h_taut, self.h_info]

    def on_draw(self, _):
        canvas = self.fig.canvas
        self.background = canvas.copy_from_bbox(self.fig.bbox)
        for a in self.animated_artists():
            self.fig.draw_artist(a)

    def tick(self):
        canvas = self.fig.canvas
        # after the window closes, queued timer events can still arrive -> stop quietly
        if (not self.plt.fignum_exists(self.fig.number) or not hasattr(canvas, "restore_region")
                or not p.isConnected(self.sim.cid)):
            self.timer.stop()
            return
        self.frame(None)
        if self.full_redraw or self.background is None:
            self.full_redraw = False
            canvas.draw_idle()
            return
        canvas.restore_region(self.background)
        for a in self.animated_artists():
            self.fig.draw_artist(a)
        canvas.blit(self.fig.bbox)
        canvas.flush_events()

    # ---------------------------------------------------- window-size scaling
    def capture_base_sizes(self):
        """Remember design-size fonts and line widths; rescale() keeps them proportional."""
        from matplotlib.lines import Line2D
        from matplotlib.text import Text
        self.base_inches = self.fig.get_size_inches().copy()
        self.base_text = [(t, t.get_fontsize()) for t in self.fig.findobj(Text)]
        self.base_lines = [(ln, ln.get_linewidth(), ln.get_markersize())
                           for ln in self.fig.findobj(Line2D)]
        self.base_radio = [(rb._buttons, rb._buttons.get_sizes().copy())
                           for rb in (self.rb_eq, self.rb_mode, self.rb_joint)]
        self.scale = 1.0

    def rescale(self, _=None):
        w, h = self.fig.get_size_inches()
        k = min(w / self.base_inches[0], h / self.base_inches[1])
        if abs(k - self.scale) < 0.01:
            return
        self.scale = k
        for t, fs in self.base_text:
            t.set_fontsize(fs * k)
        for ln, lw, ms in self.base_lines:
            ln.set_linewidth(lw * k)
            ln.set_markersize(ms * k)
        for coll, sizes in self.base_radio:
            coll.set_sizes(sizes * k * k)
        for ax in (self.ax_q, self.ax_tau, self.ax_pz):
            ax.tick_params(labelsize=9 * k)
        self.full_redraw = True

    def run(self):
        self.capture_base_sizes()
        self.fig.canvas.mpl_connect("resize_event", self.rescale)
        for a in self.animated_artists():
            a.set_animated(True)
        self.background = None
        self.fig.canvas.mpl_connect("draw_event", self.on_draw)
        self.timer = self.fig.canvas.new_timer(interval=25)
        self.timer.add_callback(self.tick)
        self.timer.start()
        self.fig.canvas.mpl_connect("close_event", self.on_close)
        signal.signal(signal.SIGINT, signal.SIG_DFL)   # GUI loop would otherwise swallow Ctrl+C
        self.plt.show()

    def on_close(self, _):
        if getattr(self, "timer", None) is not None:
            self.timer.stop()
        if p.isConnected(self.sim.cid):
            p.disconnect(self.sim.cid)


# ---------------------------------------------------------------- headless
def run_verify():
    """Check PyBullet's multibody dynamics against RK4 of the helper equations."""
    sim = AcrobotSim()
    sim.configure(1, (0, 0), 0, 0, 0, 0)
    q0, T = np.array([0.8, -0.5]), 5.0
    for j in (0, 1):
        p.resetJointState(sim.robot, j, q0[j], 0.0, physicsClientId=sim.cid)
    x = np.concatenate([q0, np.zeros(2)])
    tau_fn = lambda t: 3.0 * np.sin(2.0 * t)
    err, t = 0.0, 0.0
    E0 = energy(q0, np.zeros(2))
    for _ in range(int(T / DT)):
        tau = tau_fn(t)
        p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
        p.stepSimulation(physicsClientId=sim.cid)
        k1 = f_nonlinear(x, tau)
        k2 = f_nonlinear(x + DT / 2 * k1, tau_fn(t + DT / 2))
        k3 = f_nonlinear(x + DT / 2 * k2, tau_fn(t + DT / 2))
        k4 = f_nonlinear(x + DT * k3, tau_fn(t + DT))
        x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += DT
        q, _ = sim.state()
        err = max(err, np.max(np.abs(wrap(q - x[:2]))))
    print(f"[verify] q0 = {q0} rad, tau = 3 sin(2t), {T:.0f} s: "
          f"max |q_pybullet - q_helperRK4| = {np.degrees(err):.3f} deg")

    sim.configure(1, (0, 0), 0, 0, 0, 0)
    for j in (0, 1):
        p.resetJointState(sim.robot, j, q0[j], 0.0, physicsClientId=sim.cid)
    for _ in range(int(T / DT)):
        p.stepSimulation(physicsClientId=sim.cid)
    print(f"[verify] tau = 0, {T:.0f} s: energy drift E(T) - E(0) = "
          f"{energy(*sim.state()) - E0:+.2e} J (|E0| = {abs(E0):.2f} J)")
    p.disconnect(sim.cid)


def run_batch():
    sim = AcrobotSim()
    runs = [  # (eq, dq_deg, mode, joint, kp, kd, T, tag)
        (1, (5, -5), 0, 0, 0, 0, 10, "_open"),
        (2, (2, 0), 0, 0, 0, 0, 4, "_open"),
        (3, (1, 0), 0, 0, 0, 0, 2, "_open"),
        (4, (2, 0), 0, 0, 0, 0, 4, "_open"),
        (3, (2, 0), 1, 0, -150, -30, 3, "_PD_q1"),
        (3, (2, 0), 1, 1, 150, 30, 3, "_PD_q2"),
        (3, (1, 0), 2, 0, 0, 0, 8, "_obsLQR_q1"),
        (3, (0.1, 0), 2, 1, 0, 0, 8, "_obsLQR_q2"),
    ]
    for eq, dq, mode, joint, kp, kd, T, tag in runs:
        sim.configure(eq, dq, mode, joint, kp, kd)
        for _ in range(int(T / DT)):
            sim.step()
        save_plot(sim, tag)
    p.disconnect(sim.cid)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verify", action="store_true", help="compare PyBullet with the helper model")
    ap.add_argument("--batch", action="store_true", help="headless runs, save plots to ./plots")
    ap.add_argument("--no-analysis", action="store_true", help="skip the printed analysis")
    args = ap.parse_args()
    if not args.no_analysis:
        phase2_analysis()
    if args.verify:
        run_verify()
    elif args.batch:
        run_batch()
    else:
        Dashboard().run()
