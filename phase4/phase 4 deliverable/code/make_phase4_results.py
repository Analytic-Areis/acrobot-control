"""
Phase 4 - numbers, figures and animation data for the report and the Manim video.

PyBullet runs (../../../phase2/acrobot_phase2_sim.py) for the main evidence; the
gain search and the robustness test need thousands of runs and use a vectorized
RK4 of the given equations (same model; PyBullet agreement checked in Phase 3).

Run from the phase4 folder with the project venv:

    ../phase2/.venv/bin/python "phase 4 deliverable/code/make_phase4_results.py"
"""

import itertools
import json
import os
import sys
import time

import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
import acrobot_phase4_design as d4  # noqa: E402

d3, acro = d4.d3, d4.acro
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OUT_FIG = os.path.join(HERE, "..", "single_column")
OUT_NPZ = os.path.join(HERE, "..", "manim-sim", "phase4_data.npz")
os.makedirs(OUT_FIG, exist_ok=True)
os.makedirs(os.path.dirname(OUT_NPZ), exist_ok=True)
DT = acro.DT
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8,
                     "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                     "font.family": "serif", "mathtext.fontset": "stix",
                     "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2})
C1, C2, CT, CG, CR = "#2563eb", "#f97316", "#7c3aed", "#16a34a", "#dc2626"
R, NPZ = {}, {}
t0 = time.time()
K3, S3 = d4.K_TOP, d4.S_TOP


def save(fig, name):
    fig.savefig(os.path.join(OUT_FIG, name), bbox_inches="tight")
    plt.close(fig)
    print(f"figure {name} ({time.time() - t0:.0f} s)")


# ------------------------------------------------- vectorized hybrid controller
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def ctrl_vec(X, P):
    """Hybrid swing-up/LQR for many states; P rows = k1, k2, k3, c_switch."""
    q1, q2, d1, d2 = X
    k1, k2, k3, csw = P
    c2, s2 = np.cos(q2), np.sin(q2)
    m11, m12, m22 = 2.663 + 2 * c2, 1.33 + c2, 1.33
    E = 0.5 * (m11 * d1 * d1 + 2 * m12 * d1 * d2 + m22 * d2 * d2) - 14.715 * np.cos(q1) - 9.81 * np.cos(q1 + q2)
    h1 = -s2 * (2 * d1 * d2 + d2 * d2) + 9.81 * (1.5 * np.sin(q1) + np.sin(q1 + q2))
    h2 = s2 * d1 * d1 + 9.81 * np.sin(q1 + q2)
    v = -k1 * wrap(q2) - k2 * d2 + k3 * d1 * (E - d4.E_TOP)
    u_sw = (m22 - m12 * m12 / m11) * v + h2 - m12 * h1 / m11
    e = np.array([wrap(q1 - np.pi), wrap(q2), d1, d2])
    V = np.einsum("in,ij,jn->n", e, S3, e)
    u = np.where(V < csw, -(K3[0] @ e), u_sw)
    return np.clip(u, -20, 20), V < csw, E


def run_vec(X0, P, T=40.0, dt=2e-3):
    X = np.array(X0, float)
    first = np.full(X.shape[1], np.nan)
    for i in range(int(T / dt)):
        tau, sw, _ = ctrl_vec(X, P)
        first = np.where(np.isnan(first) & sw, i * dt, first)
        k1 = d3.f_vec(X, tau); k2 = d3.f_vec(X + dt / 2 * k1, tau)
        k3 = d3.f_vec(X + dt / 2 * k2, tau); k4 = d3.f_vec(X + dt * k3, tau)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return d3.converged(3, X), first


# ------------------------------------------------------------ 1. gain search
K1s, K2s, K3s, Cs = [20, 50, 100], [2, 5, 10], [0.3, 1, 3, 10], [0.5, 1, 2, 5]
grid = np.array(list(itertools.product(K1s, K2s, K3s, Cs)), float).T
X0 = np.array([np.full(grid.shape[1], 0.05), *np.zeros((3, grid.shape[1]))])
ok, first = run_vec(X0, grid)
R["search"] = dict(combos=int(grid.shape[1]), success=int(ok.sum()))

# robustness of every successful combo from 40 random starts near hanging
rng = np.random.default_rng(0)
starts = np.column_stack([rng.uniform(-0.6, 0.6, 40), rng.uniform(-0.6, 0.6, 40),
                          rng.uniform(-0.5, 0.5, 40), rng.uniform(-0.5, 0.5, 40)])
starts[:3] = [[0.01, 0, 0, 0], [-0.01, 0, 0, 0], [0.05, 0, 0, 0]]
robust = []
for j in np.where(ok)[0]:
    P = np.tile(grid[:, j:j + 1], (1, len(starts)))
    okj, fj = run_vec(starts.T, P)
    robust.append((int(okj.sum()), float(np.nanmedian(fj[okj])) if okj.any() else np.nan, grid[:, j]))
robust.sort(key=lambda r: (-r[0], r[1]))
R["robust_top"] = [dict(gains=[float(v) for v in g], success_of_40=s, median_catch_s=round(t, 2))
                   for s, t, g in robust[:8]]
best = robust[0]
R["chosen"] = dict(k1=d4.SWING_GAINS[0], k2=d4.SWING_GAINS[1], k3=d4.SWING_GAINS[2], c_switch=d4.C_SWITCH)
chosen_row = next(r for r in robust if tuple(r[2]) == (*d4.SWING_GAINS, d4.C_SWITCH))
R["chosen"].update(success_of_40=chosen_row[0], median_catch_s=round(chosen_row[1], 2))
print("chosen", R["chosen"])

# success-rate map over (k1, k3) at the chosen k2 and threshold (40 starts each)
rate = np.zeros((len(K3s), len(K1s)))
for a, k3 in enumerate(K3s):
    for b, k1 in enumerate(K1s):
        P = np.tile(np.array([[k1], [d4.SWING_GAINS[1]], [k3], [d4.C_SWITCH]]), (1, len(starts)))
        okj, _ = run_vec(starts.T, P)
        rate[a, b] = okj.mean()
R["rate_map"] = dict(k1=K1s, k3=K3s, k2=d4.SWING_GAINS[1], c=d4.C_SWITCH,
                     rate=[[round(float(v), 3) for v in row] for row in rate])

# why stiff elbows fail: share of the swing-up phase at the torque limit
def sat_study(g):
    P = np.tile(np.array(g, float)[:, None], (1, len(starts)))
    X = starts.T.copy(); dt = 2e-3
    sat = steps = 0.0
    for _ in range(int(40 / dt)):
        tau, sw, E = ctrl_vec(X, P)
        q1, q2, d1, d2 = X
        c2, s2 = np.cos(q2), np.sin(q2)
        m11, m12, m22 = 2.663 + 2 * c2, 1.33 + c2, 1.33
        h1 = -s2 * (2 * d1 * d2 + d2 * d2) + 9.81 * (1.5 * np.sin(q1) + np.sin(q1 + q2))
        h2 = s2 * d1 * d1 + 9.81 * np.sin(q1 + q2)
        v = -P[0] * wrap(q2) - P[1] * d2 + P[2] * d1 * (E - d4.E_TOP)
        u = (m22 - m12 * m12 / m11) * v + h2 - m12 * h1 / m11
        sat += np.sum(~sw & (np.abs(u) > 20)); steps += np.sum(~sw)
        a1 = d3.f_vec(X, tau); a2 = d3.f_vec(X + dt / 2 * a1, tau)
        a3 = d3.f_vec(X + dt / 2 * a2, tau); a4 = d3.f_vec(X + dt * a3, tau)
        X = X + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
    return int(d3.converged(3, X).sum()), round(100 * sat / max(steps, 1), 1)
R["saturation_study"] = []
for g in ((20, 2, 3, 5), (50, 2, 3, 5), (100, 2, 3, 5), (20, 2, 0.3, 5), (50, 5, 10, 5), (50, 5, 10, 1)):
    s_ok, s_sat = sat_study(g)
    R["saturation_study"].append(dict(gains=list(g), success_of_40=s_ok, swing_saturated_pct=s_sat))
print("saturation study", R["saturation_study"])

# ------------------------------------------------------- 2. PyBullet evidence
sim = acro.AcrobotSim()


def pyb(gains, c_switch, start_deg=(3.0, 0.0), T=25.0, kick_at=None, kick=0.8):
    sim.configure(1, start_deg, 0, 0, 0, 0)
    c = d4.SwingUpLQR(acro.EQUILIBRIA[1][1], gains=gains, c_switch=c_switch)
    S = d4.S_TOP
    lg = {k: [] for k in ("t", "q1", "q2", "w1", "w2", "tau", "E", "V", "lqr")}
    for i in range(int(round(T / DT))):
        if kick_at is not None and abs(i * DT - kick_at) < DT / 2:
            q, qd = sim.state()
            p.resetJointState(sim.robot, 0, q[0], qd[0] + kick, physicsClientId=sim.cid)
        dq, qd = sim.deviation()
        tau = c.full(dq, qd)
        p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
        p.stepSimulation(physicsClientId=sim.cid)
        if i % 20 == 0:
            q, qd = sim.state()
            e = d4.top_error(q, qd)
            lg["t"].append(i * DT); lg["q1"].append(q[0]); lg["q2"].append(q[1])
            lg["w1"].append(qd[0]); lg["w2"].append(qd[1]); lg["tau"].append(tau)
            lg["E"].append(d4.energy(q, qd)); lg["V"].append(float(e @ S @ e)); lg["lqr"].append(c.mode == "lqr")
    out = {k: np.asarray(v) for k, v in lg.items()}
    out["t_catch"] = c.t_catch
    return out


runs = {
    "good": pyb(d4.SWING_GAINS, d4.C_SWITCH, T=12.0),
    "orig": pyb((50.0, 5.0, 10.0), d4.C_SWITCH, T=25.0),
    "kick": pyb(d4.SWING_GAINS, d4.C_SWITCH, T=22.0, kick_at=8.0),
    "awkward": pyb(d4.SWING_GAINS, d4.C_SWITCH, start_deg=(-10.0, 20.0), T=30.0),
}


def summary(lg):
    e1 = np.degrees(wrap(lg["q1"][-1] - np.pi)); e2 = np.degrees(wrap(lg["q2"][-1]))
    held = abs(e1) < 1 and abs(e2) < 1 and abs(lg["w1"][-1]) < 0.05 and abs(lg["w2"][-1]) < 0.05
    return dict(held=bool(held), t_catch=None if lg["t_catch"] is None else round(lg["t_catch"], 2),
                end_dev=[round(float(e1), 3), round(float(e2), 3)],
                sat_pct=round(100 * float(np.mean(np.abs(lg["tau"]) >= 19.999)), 1),
                peak_E=round(float(lg["E"].max()), 2))
R["runs"] = {k: summary(v) for k, v in runs.items()}

# why the textbook gains fail: entries into the catch region, and the speed there
def entries(lg):
    inside = lg["V"] < d4.C_SWITCH
    idx = np.where(inside & ~np.r_[False, inside[:-1]])[0]
    return [dict(t=round(float(lg["t"][i]), 2), w1=round(float(lg["w1"][i]), 2),
                 w2=round(float(lg["w2"][i]), 2), E_err=round(float(lg["E"][i] - d4.E_TOP), 2)) for i in idx]
R["entries_orig"] = entries(runs["orig"])[:6]
R["entries_good"] = entries(runs["good"])[:3]
R["orig_energy_overshoot"] = round(float(runs["orig"]["E"].max() - d4.E_TOP), 2)
R["good_energy_overshoot"] = round(float(runs["good"]["E"][: int(3 / 0.005)].max() - d4.E_TOP), 2)
print("runs", R["runs"])

# ------------------------------------------------------------------ figures
def shade_lqr(ax, lg):
    on = lg["lqr"]
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        ax.axvspan(lg["t"][a], lg["t"][min(b, len(lg["t"]) - 1)], color=CG, alpha=0.12, lw=0)


lg = runs["good"]
fig, axs = plt.subplots(1, 3, figsize=(7.1, 2.3), layout="constrained")
axs[0].plot(lg["t"], np.degrees(lg["q1"]), color=C1, label=r"$q_1$")
axs[0].plot(lg["t"], np.degrees(lg["q2"]), color=C2, label=r"$q_2$")
axs[0].axhline(180, color="k", ls=":", lw=0.8)
shade_lqr(axs[0], lg)
axs[0].set_title("(a) angles (green: LQR on)")
axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("[deg]"); axs[0].legend(fontsize=6.5)
axs[1].plot(lg["t"], lg["E"], color=CT)
axs[1].axhline(d4.E_TOP, color="k", ls=":", lw=0.8)
axs[1].text(lg["t"][-1], d4.E_TOP + 2, r"$E_{top}=24.5$ J", ha="right", fontsize=6.5)
shade_lqr(axs[1], lg)
axs[1].set_title("(b) energy pumped up to $E_{top}$")
axs[1].set_xlabel("time [s]"); axs[1].set_ylabel("$E$ [J]")
axs[2].plot(lg["t"], lg["tau"], color=CR, lw=0.9)
for s in (-20, 20):
    axs[2].axhline(s, color="gray", ls=":", lw=0.8)
shade_lqr(axs[2], lg)
axs[2].set_title(rf"(c) torque ({R['runs']['good']['sat_pct']:.0f}% saturated)")
axs[2].set_xlabel("time [s]"); axs[2].set_ylabel(r"$\tau$ [N m]")
save(fig, "fig_swingup.pdf")

fig, axs = plt.subplots(1, 3, figsize=(7.1, 2.35), layout="constrained")
lo = runs["orig"]
axs[0].plot(lo["t"], lo["E"], color=CR, label="(50, 5, 10): fails")
axs[0].plot(lg["t"], lg["E"], color=CG, label="(20, 2, 3): held")
axs[0].axhline(d4.E_TOP, color="k", ls=":", lw=0.8)
axs[0].set_title("(a) energy: textbook gains overshoot")
axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("$E$ [J]"); axs[0].legend(fontsize=6.3)
axs[1].semilogy(lo["t"], lo["V"], color=CR, lw=0.9)
axs[1].semilogy(lg["t"], lg["V"], color=CG, lw=0.9)
axs[1].axhline(d4.C_SWITCH, color="k", ls="--", lw=0.8)
axs[1].text(0.3, d4.C_SWITCH * 1.4, "catch threshold", fontsize=6.5)
axs[1].set_ylim(1e-6, 1e4)
axs[1].set_title(r"(b) distance to upright $\tilde x^T S\tilde x$")
axs[1].set_xlabel("time [s]")
im = axs[2].imshow(100 * rate, origin="lower", cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
axs[2].set_xticks(range(len(K1s)), [str(k) for k in K1s])
axs[2].set_yticks(range(len(K3s)), [str(k) for k in K3s])
for a in range(len(K3s)):
    for b in range(len(K1s)):
        axs[2].text(b, a, f"{100 * rate[a, b]:.0f}%", ha="center", va="center", fontsize=6.5)
axs[2].set_xlabel("$k_1$"); axs[2].set_ylabel("$k_3$")
axs[2].set_title(f"(c) success from 40 starts ($k_2$={d4.SWING_GAINS[1]:g})")
axs[2].grid(False)
save(fig, "fig_why.pdf")

lk = runs["kick"]
fig, axs = plt.subplots(1, 2, figsize=(7.1, 2.2), layout="constrained")
axs[0].plot(lk["t"], np.degrees(wrap(lk["q1"] - np.pi)), color=C1, label=r"$q_1-\pi$")
axs[0].plot(lk["t"], np.degrees(wrap(lk["q2"])), color=C2, label=r"$q_2$")
axs[0].axvline(8.0, color="k", ls="--", lw=0.8)
axs[0].text(8.1, 150, "kick 0.8 rad/s", fontsize=6.5)
shade_lqr(axs[0], lk)
axs[0].set_title("(a) knocked off, swings up again, re-caught")
axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("deviation from upright [deg]"); axs[0].legend(fontsize=6.3)
la = runs["awkward"]
axs[1].plot(np.degrees(wrap(la["q1"] - np.pi)), la["w1"], color=C1, lw=0.7)
axs[1].plot(np.degrees(wrap(la["q1"][la["lqr"]] - np.pi)), la["w1"][la["lqr"]], ".", color=CG, ms=2)
axs[1].set_title(r"(b) phase portrait from $(-10^\circ, 20^\circ)$ (green: LQR)")
axs[1].set_xlabel(r"$q_1-\pi$ [deg]"); axs[1].set_ylabel(r"$\dot q_1$ [rad/s]")
save(fig, "fig_kick.pdf")

# ------------------------------------------------------------- manim data
for k, v in runs.items():
    for f in ("t", "q1", "q2", "tau", "E", "lqr"):
        NPZ[f"{k}_{f}"] = v[f].astype(float)
NPZ["rate"] = rate
np.savez_compressed(OUT_NPZ, **NPZ)
p.disconnect(sim.cid)
with open(os.path.join(HERE, "results.json"), "w") as fh:
    json.dump(R, fh, indent=1, default=float)
print(json.dumps(R, indent=1, default=float))
print(f"done in {time.time() - t0:.0f} s")
