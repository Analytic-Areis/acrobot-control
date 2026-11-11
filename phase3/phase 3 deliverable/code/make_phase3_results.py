"""
Phase 3 - numbers, figures and animation data for the report and the Manim video.

Time responses run on the PyBullet model (../../../phase2/acrobot_phase2_sim.py).
Region-of-attraction maps need thousands of runs, so they use a vectorized RK4 of
the given equations (../../acrobot_phase3_design.py); a sample of grid points is
re-run in PyBullet to check that both agree.

Run from the phase3 folder with the project venv:

    ../phase2/.venv/bin/python "phase 3 deliverable/code/make_phase3_results.py"

Outputs
    ../single_column/fig_*.pdf      figures used in the report
    results.json                    every number quoted in the report
    ../manim-sim/phase3_data.npz    trajectories and maps animated in the video
"""

import json
import os
import sys
import time

import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
import acrobot_phase3_design as d  # noqa: E402

acro = d.acro
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OUT_FIG = os.path.join(HERE, "..", "single_column")
OUT_NPZ = os.path.join(HERE, "..", "manim-sim", "phase3_data.npz")
os.makedirs(OUT_FIG, exist_ok=True)
os.makedirs(os.path.dirname(OUT_NPZ), exist_ok=True)
DT = acro.DT
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8,
                     "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                     "font.family": "serif", "mathtext.fontset": "stix",
                     "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2})
C1, C2, CT = "#2563eb", "#f97316", "#7c3aed"
R, NPZ = {}, {}
t_start = time.time()


def save(fig, name):
    fig.savefig(os.path.join(OUT_FIG, name), bbox_inches="tight")
    plt.close(fig)
    print(f"figure {name}  ({time.time() - t_start:.0f} s)")


def pyb_run(sim, eq, K, dq_deg, T, w0_deg=(0.0, 0.0)):
    """Closed loop tau = sat(-K x~) on the PyBullet plant; logs every 5 ms."""
    sim.configure(eq, dq_deg, 0, 0, 0, 0, w0_deg=w0_deg)
    ctrl = d.FullStateFeedback(K)
    log = {k: [] for k in ("t", "q1", "q2", "w1", "w2", "tau", "E")}
    for i in range(int(round(T / DT))):
        dq, qd = sim.deviation()
        tau = ctrl.full(dq, qd)
        p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
        p.stepSimulation(physicsClientId=sim.cid)
        if i % 20 == 0:
            q, qdot = sim.state()
            log["t"].append(i * DT)
            log["q1"].append(np.degrees(dq[0]))
            log["q2"].append(np.degrees(dq[1]))
            log["w1"].append(qd[0])
            log["w2"].append(qd[1])
            log["tau"].append(tau)
            log["E"].append(acro.energy(q, qdot))
    return {k: np.asarray(v) for k, v in log.items()}


def settle_time(lg, frac=0.02):
    """Last time the deviation is above frac of its peak."""
    m = np.hypot(lg["q1"], lg["q2"])
    above = np.where(m > frac * m.max())[0]
    return float(lg["t"][above[-1]]) if len(above) else 0.0


def com_line(eq, x):
    """Offsets with the total centre of mass vertically above/below the pivot (linearized)."""
    return {3: -2.5 * x, 2: 0.5 * x, 4: 0.5 * x, 1: -2.5 * x}[eq]


# ----------------------------------------------------------- 1. the designs
designs = {
    "E3_lqr": dict(eq=3, K=d.lqr_gain(3), label="E3 LQR"),
    "E3_place": dict(eq=3, K=d.place_gain(3), label="E3 pole placement"),
    "E2_lqr": dict(eq=2, K=d.lqr_gain(2), label="E2 LQR"),
    "E4_lqr": dict(eq=4, K=d.lqr_gain(4), label="E4 LQR"),
    "E1_pd": dict(eq=1, K=d.e1_pd_gain(), label="E1 elbow PD"),
}
R["design"] = dict(Q="I", R={f"E{k}": v for k, v in d.LQR_R.items()}, place_poles=d.PLACE_POLES[3],
                   E1_pd=dict(Kp=d.E1_PD[0], Kd=d.E1_PD[1]))
for name, ds in designs.items():
    A, B = acro.linearize(d.EQ[ds["eq"]][1])
    eig = np.linalg.eigvals(A - B @ ds["K"])
    ds["eig"] = eig
    R[name] = dict(K=[round(float(v), 2) for v in ds["K"][0]], poles=acro.fmt_eigs(eig, 2),
                   max_re=round(float(eig.real.max()), 3), fastest=round(float(eig.real.min()), 2))

# E1: shaped potential stays a minimum, so V_c = E - E* + Kp q2~^2 / 2 is a Lyapunov function
_, _, _, K1 = (None, None, None, None)
A1, B1 = acro.linearize(d.EQ[1][1])
hessU1 = -acro.mass_matrix(d.EQ[1][1]) @ A1[2:, :2]
hess_shaped = hessU1 + np.diag([0.0, d.E1_PD[0]])
R["E1_pd"].update(hessU=[round(v, 3) for v in np.linalg.eigvalsh(hessU1)],
                  hess_shaped=[round(v, 3) for v in np.linalg.eigvalsh(hess_shaped)],
                  pure_damping_max_re=round(float(np.linalg.eigvals(A1 - B1 @ d.e1_pd_gain(0.0, 2.0)).real.max()), 4))

# ------------------------------------------------------ 2. design choice: R sweep
Rs = np.logspace(-3, 0, 13)
sweep = {}
for eq, lim in ((3, 60), (2, 180), (4, 180)):
    area, fast = [], []
    A, B = acro.linearize(d.EQ[eq][1])
    for r in Rs:
        K = d.lqr_gain(eq, R=r)
        ok, g = d.roa_map(eq, K, lim_deg=lim, n=41, T=10)
        area.append(ok.mean())
        fast.append(-np.linalg.eigvals(A - B @ K).real.min())
    sweep[eq] = (np.array(area), np.array(fast))
R["sweep"] = {f"E{eq}": dict(R=[float(f"{v:.4g}") for v in Rs], area=[round(float(a), 3) for a in sweep[eq][0]],
                             fastest=[round(float(f), 1) for f in sweep[eq][1]]) for eq in sweep}

# pole placement vs LQR on the same grid (E3)
fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.3), layout="constrained")
cols = {3: "#dc2626", 2: "#16a34a", 4: "#9333ea"}
for eq in (3, 2, 4):
    axs[0].semilogx(Rs, 100 * sweep[eq][0], "o-", ms=3, color=cols[eq],
                    label=f"$E_{eq}$ (grid $\\pm${60 if eq == 3 else 180}$^\\circ$)")
    axs[0].semilogx([d.LQR_R[eq]], [100 * sweep[eq][0][np.argmin(abs(Rs - d.LQR_R[eq]))]], "k*", ms=8)
    axs[1].loglog(Rs, sweep[eq][1], "o-", ms=3, color=cols[eq], label=f"$E_{eq}$")
    axs[1].loglog([d.LQR_R[eq]], [sweep[eq][1][np.argmin(abs(Rs - d.LQR_R[eq]))]], "k*", ms=8)
axs[0].set_xlabel("$R$ (with $Q=I$)")
axs[0].set_ylabel("initial offsets recovered [% of grid]")
axs[0].set_title("(a) region of attraction vs. $R$ (stars: chosen)")
axs[0].legend(loc="center left")
axs[1].set_xlabel("$R$ (with $Q=I$)")
axs[1].set_ylabel(r"fastest pole $|\min\,\mathrm{Re}\,\lambda|$ [1/s]")
axs[1].set_title("(b) cost: speed of the fastest closed-loop pole")
axs[1].legend()
save(fig, "fig_design.pdf")

# --------------------------------------------- 3. region-of-attraction maps
maps = {}
spec = {"E3_lqr": 40, "E3_place": 40, "E2_lqr": 180, "E4_lqr": 180, "E1_pd": 180}
for name, lim in spec.items():
    ds = designs[name]
    n = 81 if lim == 40 else 73
    T = 30.0 if name == "E1_pd" else 10.0
    ok, g = d.roa_map(ds["eq"], ds["K"], lim_deg=lim, n=n, T=T)
    maps[name] = (ok, g)
    ax1, ax2 = d.axis_limits(ok, g)
    R[name].update(roa_area_pct=round(100 * float(ok.mean()), 1), grid_deg=lim,
                   axis_q1=round(float(ax1), 1), axis_q2=round(float(ax2), 1))
    # along the zero-centre-of-mass-torque direction
    s = np.linspace(0, lim, 400)
    best = 0.0
    for si in s:
        a, b = si, com_line(ds["eq"], si)
        if abs(b) > lim:
            break
        ia, ib = np.argmin(abs(g - a)), np.argmin(abs(g - b))
        ja, jb = np.argmin(abs(g + a)), np.argmin(abs(g + b))
        if ok[ib, ia] and ok[jb, ja]:
            best = float(np.hypot(a, b))
        else:
            break
    R[name]["along_com_line"] = round(best, 1)
    NPZ[f"roa_{name}"] = ok.astype(np.uint8)
    NPZ[f"roa_{name}_g"] = g

# guaranteed (linear, never saturates) ellipsoid for the LQR designs
for name in ("E3_lqr", "E2_lqr", "E4_lqr"):
    eq = designs[name]["eq"]
    A, B = acro.linearize(d.EQ[eq][1])
    K, S = d.lqr(A, B, d.LQR_Q, d.LQR_R[eq])
    c, e1, e2 = d.ellipsoid_bound(eq, K, S)
    designs[name]["S"], designs[name]["c"] = S, c
    R[name].update(ellipse_c=round(float(c), 4), ellipse_q1=round(float(e1), 2), ellipse_q2=round(float(e2), 2))

# angular-velocity kick from rest at q* (shoulder), largest recovered
for name, ds in designs.items():
    w = np.linspace(-8, 8, 321)
    qs = d.EQ[ds["eq"]][1]
    X0 = np.array([np.full(w.size, qs[0]), np.full(w.size, qs[1]), w, np.zeros(w.size)])
    X, _, _ = d.simulate_many(ds["eq"], ds["K"], X0, T=30.0 if name == "E1_pd" else 10.0)
    ok = d.converged(ds["eq"], X)
    c = w.size // 2
    hi = next((w[k - 1] for k in range(c, w.size) if not ok[k]), w[-1])
    lo = next((w[k + 1] for k in range(c, -1, -1) if not ok[k]), w[0])
    R[name]["kick_rad_s"] = round(float(min(hi, -lo)), 2)

fig, axs = plt.subplots(1, 4, figsize=(7.2, 2.15), layout="constrained")
for ax, name, title in zip(axs, ("E3_lqr", "E3_place", "E2_lqr", "E4_lqr"),
                           ("(a) $E_3$, LQR", "(b) $E_3$, pole placement", "(c) $E_2$, LQR", "(d) $E_4$, LQR")):
    ok, g = maps[name]
    eq = designs[name]["eq"]
    ax.pcolormesh(g, g, ok, cmap=matplotlib.colors.ListedColormap(["#fee2e2", "#bbf7d0"]),
                  shading="auto", rasterized=True)
    xs = np.linspace(-g[-1], g[-1], 50)
    ax.plot(xs, com_line(eq, xs), "k--", lw=0.7, label="CoM over pivot")
    if "S" in designs[name]:
        S, c = designs[name]["S"], designs[name]["c"]
        aa, bb = np.meshgrid(np.radians(g), np.radians(g))
        Vq = S[0, 0] * aa ** 2 + 2 * S[0, 1] * aa * bb + S[1, 1] * bb ** 2
        ax.contour(g, g, Vq, levels=[c], colors="#1d4ed8", linewidths=1.0)
    ax.set_xlim(g[0], g[-1])
    ax.set_ylim(g[0], g[-1])
    ax.set_aspect("equal")
    ax.set_title(f"{title}: {R[name]['roa_area_pct']:.0f}%")
    ax.set_xlabel(r"$\tilde q_1(0)$ [deg]")
    ax.grid(False)
axs[0].set_ylabel(r"$\tilde q_2(0)$ [deg]")
axs[0].legend(loc="upper right", fontsize=5.5, framealpha=0.9)
save(fig, "fig_roa.pdf")

# ----------------------------------- 4. agreement of the maps with PyBullet
sim = acro.AcrobotSim()
rng = np.random.default_rng(1)
check = {"random": [0, 0], "boundary": [0, 0]}
for name in ("E3_lqr", "E2_lqr", "E4_lqr"):
    ok, g = maps[name]
    eq, K = designs[name]["eq"], designs[name]["K"]
    edge = np.argwhere(ok ^ np.roll(ok, 1, 0) | ok ^ np.roll(ok, 1, 1))
    picks = {"random": rng.integers(0, len(g), (15, 2)),
             "boundary": edge[rng.choice(len(edge), size=min(15, len(edge)), replace=False)]}
    for kind, pick in picks.items():
        for ib, ia in pick:
            lg = pyb_run(sim, eq, K, (g[ia], g[ib]), 10.0)
            pb_ok = (abs(lg["q1"][-1]) < 1 and abs(lg["q2"][-1]) < 1
                     and abs(lg["w1"][-1]) < 0.05 and abs(lg["w2"][-1]) < 0.05)
            check[kind][0] += int(pb_ok == ok[ib, ia])
            check[kind][1] += 1
R["map_check"] = {k: dict(points=v[1], agree=v[0], pct=round(100 * v[0] / v[1], 1)) for k, v in check.items()}
print("map check", R["map_check"])

# ------------------------------------------- 5. time responses in PyBullet
K3 = designs["E3_lqr"]["K"]
lim1 = R["E3_lqr"]["axis_q1"]
runs = {
    "e3_small": pyb_run(sim, 3, K3, (2.0, 0.0), 5.0),
    "e3_edge": pyb_run(sim, 3, K3, (lim1 - 0.5, 0.0), 6.0),
    "e3_fail": pyb_run(sim, 3, K3, (lim1 + 1.5, 0.0), 6.0),
    "e3_diag": pyb_run(sim, 3, K3, (-10.0, 25.0), 6.0),
    "e3_place": pyb_run(sim, 3, designs["E3_place"]["K"], (2.0, 0.0), 5.0),
    "e2": pyb_run(sim, 2, designs["E2_lqr"]["K"], (60.0, 90.0), 8.0),
    "e4": pyb_run(sim, 4, designs["E4_lqr"]["K"], (20.0, 30.0), 8.0),
    "e1": pyb_run(sim, 1, designs["E1_pd"]["K"], (60.0, -60.0), 25.0),
    "e1_open": pyb_run(sim, 1, np.zeros((1, 4)), (60.0, -60.0), 25.0),
}
for name, key in (("E3_lqr", "e3_small"), ("E3_place", "e3_place"), ("E2_lqr", "e2"), ("E4_lqr", "e4"), ("E1_pd", "e1")):
    lg = runs[key]
    R[name].update(settle_s=round(settle_time(lg), 2), peak_tau=round(float(np.max(np.abs(lg["tau"]))), 2),
                   sat_pct=round(100 * float(np.mean(np.abs(lg["tau"]) >= acro.TAU_MAX - 1e-6)), 1))
R["runs"] = {k: dict(start=[round(float(v["q1"][0]), 2), round(float(v["q2"][0]), 2)],
                     end=[round(float(v["q1"][-1]), 3), round(float(v["q2"][-1]), 3)],
                     peak_tau=round(float(np.max(np.abs(v["tau"]))), 2)) for k, v in runs.items()}

fig, axs = plt.subplots(1, 3, figsize=(7.0, 2.3), layout="constrained")
for key, ls, col, lab in (("e3_small", "-", C1, r"LQR, $\tilde q_1(0)=2^\circ$"),
                          ("e3_place", "-", "#64748b", r"pole placement, $2^\circ$"),
                          ("e3_edge", "-", "#16a34a", rf"LQR, ${lim1 - 0.5:g}^\circ$ (held)"),
                          ("e3_fail", "--", "#dc2626", rf"LQR, ${lim1 + 1.5:g}^\circ$ (falls)")):
    lg = runs[key]
    axs[0].plot(lg["t"], acro.break_wraps(lg["q1"]), ls=ls, color=col, label=lab)
    axs[1].plot(lg["t"], lg["tau"], ls=ls, color=col)
lg = runs["e3_diag"]
axs[2].plot(lg["t"], lg["q1"], color=C1, label=r"$\tilde q_1$")
axs[2].plot(lg["t"], lg["q2"], color=C2, label=r"$\tilde q_2$")
axs[0].set_ylim(-40, 40)
axs[0].set_xlabel("time [s]")
axs[0].set_ylabel(r"$\tilde q_1$ [deg]")
axs[0].set_title("(a) upright $E_3$: offset in $q_1$")
axs[0].legend(fontsize=6, loc="upper right")
axs[1].axhline(20, color="r", ls=":", lw=0.8)
axs[1].axhline(-20, color="r", ls=":", lw=0.8)
axs[1].set_ylim(-22, 22)
axs[1].set_xlabel("time [s]")
axs[1].set_ylabel(r"$\tau$ [N m]")
axs[1].set_title(r"(b) torque ($|\tau|\leq 20$)")
axs[2].set_xlabel("time [s]")
axs[2].set_ylabel("deviation [deg]")
axs[2].set_title(r"(c) LQR from $(-10^\circ, 25^\circ)$: held")
axs[2].legend(fontsize=6.5)
save(fig, "fig_e3.pdf")

# other equilibria + E1 energy
fig, axs = plt.subplots(1, 4, figsize=(7.2, 2.2), layout="constrained")
for ax, key, title in ((axs[0], "e2", r"(a) $E_2$ LQR from $(60^\circ, 90^\circ)$"),
                       (axs[1], "e4", r"(b) $E_4$ LQR from $(20^\circ, 30^\circ)$")):
    lg = runs[key]
    ax.plot(lg["t"], acro.break_wraps(lg["q1"]), color=C1, label=r"$\tilde q_1$")
    ax.plot(lg["t"], acro.break_wraps(lg["q2"]), color=C2, label=r"$\tilde q_2$")
    ax2 = ax.twinx()
    ax2.plot(lg["t"], lg["tau"], color=CT, lw=0.7, alpha=0.6)
    ax2.set_ylim(-60, 22)
    ax2.set_yticks([-20, 0, 20])
    ax2.tick_params(labelsize=6, colors=CT)
    ax2.grid(False)
    ax.set_title(title)
    ax.set_xlabel("time [s]")
axs[0].set_ylabel("deviation [deg]  (purple: $\\tau$)")
axs[0].legend(fontsize=6, loc="lower right")
lg, lo = runs["e1"], runs["e1_open"]
axs[2].plot(lo["t"], lo["q1"], color="#94a3b8", lw=0.8, label="open loop")
axs[2].plot(lg["t"], lg["q1"], color=C1, label="elbow PD")
axs[2].set_title(r"(c) $E_1$: $\tilde q_1$ from $(60^\circ,-60^\circ)$")
axs[2].set_xlabel("time [s]")
axs[2].legend(fontsize=6, loc="upper right")
Vc = lg["E"] - lg["E"].min() + 0.5 * d.E1_PD[0] * np.radians(lg["q2"]) ** 2
E1star = acro.energy(np.zeros(2), np.zeros(2))
Vc = lg["E"] - E1star + 0.5 * d.E1_PD[0] * np.radians(lg["q2"]) ** 2
axs[3].semilogy(lg["t"], np.maximum(Vc, 1e-6), color=C1)
axs[3].set_title(r"(d) $E_1$: $V_c=E-E^*+\frac{1}{2}K_p\tilde q_2^{\,2}$")
axs[3].set_xlabel("time [s]")
axs[3].set_ylabel("[J]")
R["E1_pd"]["Vc_start"] = round(float(Vc[0]), 2)
R["E1_pd"]["Vc_monotone"] = bool(np.all(np.diff(Vc) <= 1e-6))
save(fig, "fig_others.pdf")

# E1 map
ok, g = maps["E1_pd"]
fig, ax = plt.subplots(figsize=(2.4, 2.3), layout="constrained")
ax.pcolormesh(g, g, ok, cmap=matplotlib.colors.ListedColormap(["#fee2e2", "#bbf7d0"]), shading="auto", rasterized=True)
ax.set_aspect("equal")
ax.set_title(f"$E_1$ elbow PD: {R['E1_pd']['roa_area_pct']:.0f}% of ±180°")
ax.set_xlabel(r"$\tilde q_1(0)$ [deg]")
ax.set_ylabel(r"$\tilde q_2(0)$ [deg]")
ax.grid(False)
save(fig, "fig_e1map.pdf")

# ------------------------------------------------------------ 6. manim data
for k, lg in runs.items():
    for f in ("t", "q1", "q2", "tau"):
        NPZ[f"{k}_{f}"] = lg[f]
NPZ["sweep_R"] = Rs
for eq in sweep:
    NPZ[f"sweep_area_E{eq}"] = sweep[eq][0]
    NPZ[f"sweep_fast_E{eq}"] = sweep[eq][1]
for name in ("E3_lqr", "E2_lqr", "E4_lqr"):
    NPZ[f"S_{name}"] = designs[name]["S"]
    NPZ[f"c_{name}"] = designs[name]["c"]
np.savez_compressed(OUT_NPZ, **NPZ)
p.disconnect(sim.cid)

with open(os.path.join(HERE, "results.json"), "w") as fh:
    json.dump(R, fh, indent=1, default=float)
print(json.dumps({k: v for k, v in R.items() if k not in ("sweep", "runs")}, indent=1, default=float))
print(f"done in {time.time() - t_start:.0f} s")
