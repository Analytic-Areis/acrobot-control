"""
Phase 2 - numbers, figures and animation data for the report and the Manim video.

Every simulation here runs on the PyBullet model in ../../acrobot_phase2_sim.py.
Run from the phase2 folder with the project venv:

    .venv/bin/python "phase 2 deliverable/code/make_phase2_results.py"

Outputs
    ../single_column/fig_*.pdf      figures used in the report
    results.json                    every number quoted in the report
    ../manim-sim/phase2_data.npz    trajectories animated in the video
"""

import json
import os
import sys

import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
import acrobot_phase2_sim as acro  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OUT_FIG = os.path.join(HERE, "..", "single_column")
OUT_NPZ = os.path.join(HERE, "..", "manim-sim", "phase2_data.npz")
DT = acro.DT
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8,
                     "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                     "font.family": "serif", "mathtext.fontset": "stix",
                     "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2})
C1, C2 = "#2563eb", "#f97316"
R = {}            # every number for the report
NPZ = {}          # trajectories for manim


# ------------------------------------------------------------------ helpers
def run_open_loop(sim, eq, dq_deg, T):
    sim.configure(eq, dq_deg, 0, 0, 0, 0)
    for _ in range(int(round(T / DT))):
        sim.step()
    return {k: np.asarray(v) for k, v in sim.log.items()}


def run_policy(sim, eq, dq_deg, policy, T, stop_deg=170.0):
    """Closed loop on the PyBullet plant; policy(dq, qd) -> tau (already saturated)."""
    sim.configure(eq, dq_deg, 0, 0, 0, 0)
    log = {"t": [], "q1": [], "q2": [], "tau": []}
    n = int(round(T / DT))
    for i in range(n):
        dq, qd = sim.deviation()
        tau = policy(dq, qd)
        p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
        p.stepSimulation(physicsClientId=sim.cid)
        if i % 20 == 0:
            log["t"].append(i * DT)
            log["q1"].append(np.degrees(dq[0]))
            log["q2"].append(np.degrees(dq[1]))
            log["tau"].append(tau)
        if np.max(np.abs(np.degrees(dq))) > stop_deg:
            break
    return {k: np.asarray(v) for k, v in log.items()}


def held(log, tol=0.5):
    """Recovered: stays inside 90 deg and ends within tol deg of q*."""
    return (np.max(np.abs(np.r_[log["q1"], log["q2"]])) < 90 and log["t"][-1] > 9.9
            and abs(log["q1"][-1]) < tol and abs(log["q2"][-1]) < tol)


def largest_held(sim, make_policy, joint_dir, lo=0.0, hi=30.0, iters=12):
    """Bisection on the size of the initial offset along one joint (deg)."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        dq = (mid, 0.0) if joint_dir == 0 else (0.0, mid)
        log = run_policy(sim, 3, dq, make_policy(dq), 10.0)
        lo, hi = (mid, hi) if held(log) else (lo, mid)
    return lo


def obs_policy(A, B, joint):
    def make(dq_deg):
        c = acro.SingleAngleObserverLQR(A, B, joint)
        c.reset(np.radians(dq_deg[joint]))
        return lambda dq, qd: c(dq[joint], qd[joint], DT)
    return make


def full_state_policy(A, B):
    K = acro.SingleAngleObserverLQR(A, B, 0).K[0]      # same LQR weights
    def make(dq_deg):
        return lambda dq, qd: float(np.clip(-K @ np.r_[dq, qd], -acro.TAU_MAX, acro.TAU_MAX))
    return make


def save(fig, name):
    fig.savefig(os.path.join(OUT_FIG, name), bbox_inches="tight")
    plt.close(fig)
    print("figure", name)


# ------------------------------------------------------- 1. linear analysis
lin = {}
for k, (name, qs) in acro.EQUILIBRIA.items():
    A, B = acro.linearize(qs)
    P = A[2:, :2]
    Mq = acro.mass_matrix(qs)
    K = Mq @ P
    eigA = np.linalg.eigvals(A)
    ctrb = np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(4)]))
    obs = [int(np.linalg.matrix_rank(np.vstack([np.eye(4)[[j]] @ np.linalg.matrix_power(A, i)
                                                for i in range(4)]))) for j in (0, 1)]
    hessU = -K                                     # Hessian of U at q*
    lin[k] = dict(name=name, eigA=eigA, eigP=np.linalg.eigvals(P).real,
                  eigH=np.linalg.eigvalsh(hessU), trP=np.trace(P), detP=np.linalg.det(P),
                  ctrb=int(ctrb), obs=obs, n_rhp=int(np.sum(eigA.real > 1e-9)),
                  max_re=float(np.max(eigA.real)))
    R[f"E{k}"] = dict(eigA=acro.fmt_eigs(eigA, 4), eigP=[round(v, 4) for v in sorted(lin[k]["eigP"])],
                      hessU=[round(v, 4) for v in lin[k]["eigH"]], trP=round(lin[k]["trP"], 4),
                      detP=round(lin[k]["detP"], 4), ctrb_rank=lin[k]["ctrb"],
                      obs_rank_q1=obs[0], obs_rank_q2=obs[1], n_rhp=lin[k]["n_rhp"])

sim = acro.AcrobotSim()

# ------------------------------------------------- 2. open-loop simulations
# E1: long run for the spectrum and energy; E2-E4: small offsets to see growth
runs = {1: run_open_loop(sim, 1, (10.0, -10.0), 60.0)}
E1_drift = float(np.max(np.abs(runs[1]["E"] - runs[1]["E"][0])))
for k, d0, T in ((2, (0.05, 0.0), 4.0), (3, (0.01, 0.0), 2.5), (4, (0.05, 0.0), 4.0)):
    runs[k] = run_open_loop(sim, k, d0, T)

# growth-rate fit: slope of ln|dq1| while 0.2 deg < |dq| < 15 deg
growth = {}
for k in (2, 3, 4):
    lg = runs[k]
    mag = np.hypot(lg["q1"], lg["q2"])
    i0 = int(np.argmax(mag > 0.2))
    i1 = int(np.argmax(mag > 15.0))
    t_fit = lg["t"][i0:i1]
    slope = np.polyfit(t_fit, np.log(mag[i0:i1]), 1)[0]
    t_escape = float(lg["t"][int(np.argmax(mag > 30.0))]) if np.any(mag > 30.0) else np.nan
    growth[k] = dict(fit=float(slope), pole=lin[k]["max_re"], t30=t_escape,
                     rel_err=float(abs(slope - lin[k]["max_re"]) / lin[k]["max_re"]))
    R[f"E{k}"].update(growth_fit=round(slope, 3), growth_pole=round(lin[k]["max_re"], 4),
                      growth_err_pct=round(100 * growth[k]["rel_err"], 2), t_to_30deg=round(t_escape, 2),
                      d0_deg=[0.05, 0.0] if k != 3 else [0.01, 0.0])

# E1 spectrum: peaks should sit at the jw-poles 2.2400 and 6.1011 rad/s
lg = runs[1]
y = lg["q1"] - lg["q1"].mean()
dt_log = lg["t"][1] - lg["t"][0]
Y = np.abs(np.fft.rfft(y * np.hanning(len(y)), n=2**18))
w = 2 * np.pi * np.fft.rfftfreq(2**18, dt_log)
peaks = []
for band in ((1.5, 4.0), (4.5, 8.0)):
    m = (w > band[0]) & (w < band[1])
    peaks.append(float(w[m][np.argmax(Y[m])]))
R["E1"].update(fft_peaks=[round(v, 3) for v in peaks], energy_drift_60s=float(f"{E1_drift:.2e}"),
               amp_q1_max=round(float(np.max(np.abs(lg["q1"]))), 2))

# figure: open-loop responses at all four equilibria (nonlinear vs linear)
short = {1: run_open_loop(sim, 1, (10.0, -10.0), 8.0), 2: run_open_loop(sim, 2, (2.0, 0.0), 2.5),
         3: run_open_loop(sim, 3, (1.0, 0.0), 1.2), 4: run_open_loop(sim, 4, (2.0, 0.0), 2.0)}
fig, axs = plt.subplots(2, 2, figsize=(7.0, 3.6))
for ax, k in zip(axs.flat, (1, 2, 3, 4)):
    lg = short[k]
    ax.plot(lg["t"], acro.break_wraps(lg["q1"]), color=C1, label=r"$\tilde q_1$ PyBullet")
    ax.plot(lg["t"], acro.break_wraps(lg["q2"]), color=C2, label=r"$\tilde q_2$ PyBullet")
    ax.plot(lg["t"], lg["q1_lin"], "k--", lw=0.9, label="linear model")
    ax.plot(lg["t"], lg["q2_lin"], "k:", lw=0.9)
    lim = 25 if k == 1 else 190
    ax.set_ylim(-lim, lim)
    verdict = "marginal (oscillates)" if k == 1 else f"unstable, {lin[k]['n_rhp']} RHP pole" + ("s" if lin[k]["n_rhp"] > 1 else "")
    ax.set_title(f"({'abcd'[k - 1]}) {acro.EQUILIBRIA[k][0].split(' (')[0]}: {verdict}")
    ax.set_ylabel("deviation [deg]")
    if k > 2:
        ax.set_xlabel("time [s]")
axs[0, 0].legend(loc="upper right", ncol=3, fontsize=6.5)
fig.tight_layout(pad=0.3)
save(fig, "fig_openloop.pdf")

# figure: growth rate (semilog) and E1 spectrum
fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.3))
cols = {2: "#16a34a", 3: "#dc2626", 4: "#9333ea"}
for k in (2, 3, 4):
    lg = runs[k]
    mag = np.hypot(lg["q1"], lg["q2"])
    axs[0].semilogy(lg["t"], mag, color=cols[k], label=f"E{k}: fit {growth[k]['fit']:.2f} vs pole {growth[k]['pole']:.2f} 1/s")
    i0 = int(np.argmax(mag > 0.2))
    tt = np.linspace(lg["t"][i0], lg["t"][i0] + np.log(150 / 0.2) / growth[k]["pole"], 10)
    axs[0].semilogy(tt, 0.2 * np.exp(growth[k]["pole"] * (tt - tt[0])), "k:", lw=0.8)
axs[0].set_ylim(5e-3, 300)
axs[0].set_xlabel("time [s]")
axs[0].set_ylabel(r"$\|\tilde q\|$ [deg]")
axs[0].set_title("(a) growth from a tiny offset (dotted: $e^{p_{max}t}$)")
axs[0].legend(loc="lower right", fontsize=6.3)
axs[1].plot(w, Y / Y.max(), color=C1)
for wp in (2.2400, 6.1011):
    axs[1].axvline(wp, color="k", ls=":", lw=0.9)
axs[1].set_xlim(0, 9)
axs[1].set_xlabel(r"$\omega$ [rad/s]")
axs[1].set_ylabel(r"$|\mathcal{F}\{\tilde q_1\}|$ (norm.)")
axs[1].set_title(r"(b) $E_1$, 60 s: spectrum peaks vs poles $\pm j2.24,\pm j6.10$")
fig.tight_layout(pad=0.3)
save(fig, "fig_growth.pdf")

# ------------------------------------------- 3. single-angle PD at E3
A3, B3 = acro.linearize(acro.EQUILIBRIA[3][1])
kp = np.linspace(-400, 400, 321)
kd = np.linspace(-150, 150, 241)
best = {}
fig, axs = plt.subplots(1, 3, figsize=(7.0, 2.45), layout="constrained")
norm = matplotlib.colors.LogNorm(vmin=0.5, vmax=30)
for j in (0, 1):
    Z = np.empty((len(kd), len(kp)))
    for a, d in enumerate(kd):
        for b, pgain in enumerate(kp):
            F = np.zeros((1, 4))
            F[0, j], F[0, j + 2] = pgain, d
            Z[a, b] = np.max(np.linalg.eigvals(A3 - B3 @ F).real)
    ia, ib = np.unravel_index(np.argmin(Z), Z.shape)
    best[j] = dict(kp=float(kp[ib]), kd=float(kd[ia]), max_re=float(Z[ia, ib]))
    im = axs[j].pcolormesh(kp, kd, np.clip(Z, 0.5, 30), cmap="YlOrRd", norm=norm, shading="auto",
                           rasterized=True, linewidth=0, edgecolors="none")
    axs[j].contour(kp, kd, Z, levels=[1, 2, 3, 5, 10], colors="k", linewidths=0.4, alpha=0.5)
    axs[j].plot(kp[ib], kd[ia], "*", color="#22c55e", ms=9, mec="k", mew=0.6, clip_on=False)
    axs[j].set_title(f"({'ab'[j]}) PD on $q_{j + 1}$: min over grid {Z.min():.2f}")
    axs[j].set_xlabel("$K_p$ [N m/rad]")
    axs[j].grid(False)
axs[0].set_ylabel("$K_d$ [N m s/rad]")
cb = fig.colorbar(im, ax=axs[:2], shrink=0.9, pad=0.01)
cb.set_label("max Re(closed-loop pole) [1/s]", fontsize=7)
R["PD"] = {f"q{j + 1}": dict(best_kp=best[j]["kp"], best_kd=best[j]["kd"],
                             min_max_re=round(best[j]["max_re"], 4)) for j in (0, 1)}

pd_runs = {}
for j in (0, 1):
    pdc = acro.SingleAnglePD(j, best[j]["kp"], best[j]["kd"])
    pd_runs[j] = run_policy(sim, 3, (1.0, 0.0), lambda dq, qd, c=pdc, j=j: c(dq[j], qd[j], DT), 4.0)
    m = np.hypot(pd_runs[j]["q1"], pd_runs[j]["q2"])
    R["PD"][f"q{j + 1}"]["t_to_30deg_from_1deg"] = round(float(pd_runs[j]["t"][int(np.argmax(m > 30))]), 2)
    axs[2].plot(pd_runs[j]["t"], m, color=(C1, C2)[j], label=f"PD on $q_{j + 1}$ (best gains)")
axs[2].set_yscale("log")
axs[2].set_ylim(0.1, 300)
axs[2].set_xlabel("time [s]")
axs[2].set_ylabel(r"$\|\tilde q\|$ [deg]")
axs[2].set_title(r"(c) PyBullet, $\tilde q_1(0)=1^\circ$")
axs[2].legend(fontsize=6.3, loc="lower right")
save(fig, "fig_pd.pdf")

# ----------------------------- 4. dynamic output feedback (observer + LQR)
obsq = {j: acro.SingleAngleObserverLQR(A3, B3, j) for j in (0, 1)}
R["observer"] = {}
for j in (0, 1):
    c = obsq[j]
    eig_ctrl = np.linalg.eigvals(A3 - B3 @ c.K)
    eig_obs = np.linalg.eigvals(A3 - c.L @ c.C)
    zeros = acro.tf_zeros(A3, B3, j)
    z = float(np.max(zeros.real))
    rhp = [pl for pl in np.linalg.eigvals(A3).real if pl > 0]
    bound = float(np.prod([(z + pl) / abs(z - pl) for pl in rhp]))
    R["observer"][f"q{j + 1}"] = dict(K=[round(v, 2) for v in c.K[0]], L=[round(v, 2) for v in c.L[:, 0]],
                                       eig_ctrl=acro.fmt_eigs(eig_ctrl, 3), eig_obs=acro.fmt_eigs(eig_obs, 3),
                                       rhp_zero=round(z, 4), sens_bound=round(bound, 1))

roa = {}
for j in (0, 1):
    for d in (0, 1):
        roa[(f"obs_q{j + 1}", d)] = largest_held(sim, obs_policy(A3, B3, j), d, hi=10.0)
for d in (0, 1):
    roa[("full", d)] = largest_held(sim, full_state_policy(A3, B3), d, hi=40.0)
R["roa_deg"] = {f"{k}_offset_q{d + 1}": round(v, 2) for (k, d), v in roa.items()}
print("ROA", R["roa_deg"])

obs_runs = {
    "q1_ok": run_policy(sim, 3, (1.0, 0.0), obs_policy(A3, B3, 0)((1.0, 0.0)), 8.0),
    "q2_ok": run_policy(sim, 3, (0.1, 0.0), obs_policy(A3, B3, 1)((0.1, 0.0)), 8.0),
    "q1_fail": run_policy(sim, 3, (3.0, 0.0), obs_policy(A3, B3, 0)((3.0, 0.0)), 8.0),
    "full_10": run_policy(sim, 3, (10.0, 0.0), full_state_policy(A3, B3)((10.0, 0.0)), 8.0),
}
fig, axs = plt.subplots(1, 3, figsize=(7.0, 2.3), gridspec_kw={"width_ratios": [1.1, 1.1, 0.8]})
for name, ls, lab in (("q1_ok", "-", r"measure $q_1$, $1^\circ$"), ("q2_ok", "-", r"measure $q_2$, $0.1^\circ$"),
                      ("q1_fail", "--", r"measure $q_1$, $3^\circ$ (falls)")):
    lg = obs_runs[name]
    col = C2 if name == "q2_ok" else C1
    axs[0].plot(lg["t"], acro.break_wraps(lg["q1"]), ls=ls, color=col, label=lab)
    axs[1].plot(lg["t"], lg["tau"], ls=ls, color=col)
axs[0].set_ylim(-30, 30)
axs[0].set_xlabel("time [s]")
axs[0].set_ylabel(r"$\tilde q_1$ [deg]")
axs[0].set_title("(a) observer + LQR, one angle measured")
axs[0].legend(fontsize=6.3, loc="upper right")
axs[1].set_ylim(-22, 22)
axs[1].axhline(20, color="r", ls=":", lw=0.8)
axs[1].axhline(-20, color="r", ls=":", lw=0.8)
axs[1].set_xlabel("time [s]")
axs[1].set_ylabel(r"$\tau$ [N m]")
axs[1].set_title(r"(b) elbow torque ($|\tau|\leq 20$)")
labels = [r"obs. $q_1$", r"obs. $q_2$", "full state"]
vals = [roa[("obs_q1", 0)], roa[("obs_q2", 0)], roa[("full", 0)]]
bars = axs[2].bar(labels, vals, color=[C1, C2, "#64748b"])
axs[2].set_yscale("log")
axs[2].set_ylim(0.02, 100)
for bar, v in zip(bars, vals):
    axs[2].text(bar.get_x() + bar.get_width() / 2, v * 1.25, f"{v:.2g}°", ha="center", fontsize=6.5)
axs[2].set_ylabel(r"largest $\tilde q_1(0)$ held [deg]")
axs[2].set_title("(c) region held (PyBullet)")
axs[2].tick_params(axis="x", labelsize=6.5)
axs[2].grid(axis="x", visible=False)
fig.tight_layout(pad=0.3)
save(fig, "fig_observer.pdf")

# ------------------------------------------------- 5. simulator verification
sim.configure(1, (0, 0), 0, 0, 0, 0)
q0, T = np.array([0.8, -0.5]), 5.0
for j in (0, 1):
    p.resetJointState(sim.robot, j, q0[j], 0.0, physicsClientId=sim.cid)
x, err, t = np.r_[q0, 0, 0], 0.0, 0.0
for _ in range(int(T / DT)):
    tau = 3.0 * np.sin(2.0 * t)
    p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
    p.stepSimulation(physicsClientId=sim.cid)
    f = lambda xx, tt: acro.f_nonlinear(xx, 3.0 * np.sin(2.0 * tt))
    k1 = f(x, t); k2 = f(x + DT / 2 * k1, t + DT / 2); k3 = f(x + DT / 2 * k2, t + DT / 2); k4 = f(x + DT * k3, t + DT)
    x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    t += DT
    err = max(err, np.max(np.abs(acro.wrap(sim.state()[0] - x[:2]))))
R["verify"] = dict(max_err_deg=round(float(np.degrees(err)), 3), dt=DT)

# ------------------------------------------------------- 6. data for manim
def pack(prefix, lg, keys=("t", "q1", "q2", "q1_lin", "q2_lin", "tau")):
    for k in keys:
        if k in lg:
            NPZ[f"{prefix}_{k}"] = np.asarray(lg[k], dtype=float)

for k in (1, 2, 3, 4):
    pack(f"open{k}", short[k])
for k in (2, 3, 4):
    pack(f"grow{k}", runs[k])
pack("e1long", {k: runs[1][k][: int(20 / (20 * DT))] for k in ("t", "q1", "q2")})
for j in (0, 1):
    pack(f"pd{j + 1}", pd_runs[j])
for name, lg in obs_runs.items():
    pack(f"obs_{name}", lg)
NPZ["pd_kp"] = kp
NPZ["pd_kd"] = kd
np.savez_compressed(OUT_NPZ, **NPZ)

with open(os.path.join(HERE, "results.json"), "w") as fh:
    json.dump(R, fh, indent=2, default=float)
print(json.dumps(R, indent=1, default=float))
