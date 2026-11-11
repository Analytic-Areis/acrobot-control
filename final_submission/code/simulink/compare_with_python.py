"""Cross-check the Simulink trajectories against the project's Python model and controllers.

Reads results/*.csv written by make_simulink_figures.m, re-simulates every case with
the Python code used in the reports (f_vec from phase 3, swing_torque from phase 4),
integrated independently with fixed-step RK4 (0.1 ms, control evaluated at every stage,
like Simulink's continuous loop), then writes
    results/comparison.json
    <final_submission>/report_source/figures/fig_simulink.pdf

Run with the phase 2 virtual environment:
    ../phase2/.venv/bin/python compare_with_python.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
for sub in ("phase2", "phase3", "phase4"):
    sys.path.insert(0, os.path.join(ROOT, sub))
import acrobot_phase3_design as d3   # noqa: E402
import acrobot_phase4_design as d4   # noqa: E402

RES = os.path.join(HERE, "results")
# report figures: final_submission/report_source/figures, seen from code/simulink or from ../simulink
FIGDIR = next(d for d in (os.path.join(ROOT, "..", "report_source", "figures"),
                          os.path.join(ROOT, "final_submission", "report_source", "figures")) if os.path.isdir(d))
FIG = os.path.join(FIGDIR, "fig_simulink.pdf")
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8,
                     "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                     "font.family": "serif", "mathtext.fontset": "stix",
                     "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2})
C1, C2, CT, CG, CR = "#2563eb", "#f97316", "#7c3aed", "#16a34a", "#dc2626"
K = np.ravel(d4.K_TOP)
S = d4.S_TOP


def load(name):
    return np.loadtxt(os.path.join(RES, name), delimiter=",")


def rk4(ctrl, x0, t):
    """RK4 with steps of at most 0.1 ms, landing exactly on each Simulink time point t."""
    x, out = np.array(x0, float), [np.array(x0, float)]
    f = lambda x: d3.f_vec(x, np.clip(ctrl(x), -20, 20))
    for k in range(len(t) - 1):
        n = max(1, int(np.ceil((t[k + 1] - t[k]) / 1e-4 - 1e-9)))
        h = (t[k + 1] - t[k]) / n
        for _ in range(n):
            k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        out.append(x.copy())
    return np.array(out)


def lqr_ctrl(x):
    e = np.array([wrap(x[0] - np.pi), wrap(x[1]), x[2], x[3]])
    return -K @ e


def swing_ctrl(x):
    e = d4.top_error(x[:2], x[2:])
    return -K @ e if e @ S @ e < 5 else d4.swing_torque(x[:2], x[2:])


def max_dev_deg(a, b):
    return float(np.degrees(np.max(np.abs(wrap(a[:, :2] - b[:, :2])))))


R = {}
# gains: MATLAB lqr in Simulink run vs Python lqr
KS = load("lqr_K_S.csv")
R["K_matlab"] = KS[0].tolist()
R["K_python"] = K.tolist()
R["K_rel_diff"] = float(np.max(np.abs(KS[0] - K)) / np.max(np.abs(K)))

# open loop
ol = {}
for eq, qs in (("E1", (0, 0)), ("E3", (np.pi, 0))):
    D = load(f"openloop_{eq}.csv"); t, X, L = D[:, 0], D[:, 1:5], D[:, 5:9]
    P = rk4(lambda x: 0.0, X[0], t)
    ol[eq] = (t, X, L, P, qs)
    R[f"openloop_{eq}_max_diff_deg"] = max_dev_deg(X, P)

# LQR
lq = {}
for a in (2, 5.5, 7.5):
    D = load(f"lqr_E3_{a:g}.csv"); t, X, tau = D[:, 0], D[:, 1:5], D[:, 5]
    P = rk4(lqr_ctrl, X[0], t)
    lq[a] = (t, X, P, tau)
    R[f"lqr_E3_{a:g}_max_diff_deg"] = max_dev_deg(X, P)
    R[f"lqr_E3_{a:g}_held_simulink"] = bool(abs(np.degrees(wrap(X[-1, 0] - np.pi))) < 1)
    R[f"lqr_E3_{a:g}_held_python"] = bool(abs(np.degrees(wrap(P[-1, 0] - np.pi))) < 1)

# swing-up
D = load("swingup_3.csv"); t, X, tau, on = D[:, 0], D[:, 1:5], D[:, 5], D[:, 6]
Psw = rk4(swing_ctrl, X[0], t)
onP = np.array([d4.top_error(p[:2], p[2:]) @ S @ d4.top_error(p[:2], p[2:]) < 5 for p in Psw])
R["swingup_catch_simulink_s"] = float(t[np.argmax(on > 0.5)])
R["swingup_catch_python_s"] = float(t[np.argmax(onP)])
R["swingup_max_diff_deg"] = max_dev_deg(X, Psw)
R["swingup_peak_tau"] = float(np.max(np.abs(tau)))
json.dump(R, open(os.path.join(RES, "comparison.json"), "w"), indent=2)
for k, v in R.items():
    print(f"{k:32s} {v}")

# ------------------------------------------------------------------ figure
fig, ax = plt.subplots(2, 2, figsize=(7.0, 4.6))
t, X, L, P, qs = ol["E1"]
a = ax[0, 0]
for j, c, n in ((0, C1, "q_1"), (1, C2, "q_2")):
    a.plot(t, np.degrees(X[:, j]), color=c, label=f"${n}$ Simulink")
    a.plot(t, np.degrees(L[:, j]), "k--", lw=0.8, label="linear (State-Space)" if j == 0 else None)
    a.plot(t[::200], np.degrees(P[::200, j]), "o", ms=2.2, mfc="none", color=c)
a.set(title="(a) Open loop near $E_1$ (markers: Python)", xlabel="t [s]", ylabel="angle [deg]", ylim=(-16, 26), yticks=range(-15, 16, 5))
a.legend(loc="upper center", ncol=3, fontsize=6)

t, X, L, P, qs = ol["E3"]
a = ax[0, 1]
dev = np.degrees(np.hypot(wrap(X[:, 0] - np.pi), X[:, 1]))
devL = np.degrees(np.hypot(L[:, 0], L[:, 1]))
a.semilogy(t, dev, color=C1, label="nonlinear (Simulink)")
a.semilogy(t, devL, "k--", lw=0.8, label="linear (State-Space)")
a.semilogy(t, dev[0] * np.exp(6.1011 * t) * devL[-1] / (dev[0] * np.exp(6.1011 * t[-1])), ":", color=CR, lw=0.9,
           label=r"slope $e^{6.10t}$")
a.set(title="(b) Open loop near $E_3$: growth at the pole", xlabel="t [s]", ylabel="deviation [deg]", ylim=(0.3, 300))
a.legend(loc="lower right")

a = ax[1, 0]
for ang, c in ((2, CG), (5.5, C1), (7.5, CR)):
    t, X, P, tau = lq[ang]
    ys, yp = (np.degrees(np.unwrap(Y[:, 0]) - np.pi) for Y in (X, P))
    cut = np.argmax(np.abs(ys) > 75) if np.any(np.abs(ys) > 75) else len(t)
    ys[cut:] = yp[cut:] = np.nan                       # stop drawing once it has fallen
    a.plot(t, ys, color=c, label=f"{ang:g}$^\\circ$" + (" (falls)" if ang > 7 else ""))
    a.plot(t[::150], yp[::150], "o", ms=2.2, mfc="none", color=c)
a.set(title="(c) LQR at $E_3$: $q_1-\\pi$ (markers: Python)", xlabel="t [s]", ylabel="angle [deg]", ylim=(-70, 40))
a.legend(title="start", loc="lower right")

a = ax[1, 1]
t, X = D[:, 0], D[:, 1:5]
a.plot(t, np.degrees(np.unwrap(X[:, 0])), color=C1, label="$q_1$")
y2 = np.degrees(wrap(X[:, 1])); y2[1:][np.abs(np.diff(y2)) > 180] = np.nan   # no line across the wrap
a.plot(t, y2, color=C2, label="$q_2$")
a.plot(t[::150], np.degrees(np.unwrap(Psw[::150, 0])), "o", ms=2.0, mfc="none", color=C1)
tc = R["swingup_catch_simulink_s"]
a.axvline(tc, color=CT, ls="--", lw=0.9)
a.text(tc + 0.12, -185, f"catch {tc:.2f} s", color=CT, fontsize=6.5)
a2 = a.twinx(); a2.plot(t, D[:, 5], color="0.6", lw=0.6, label=r"$\tau$ (right axis)"); a2.set_ylabel(r"$\tau$ [N m]", color="0.45"); a2.grid(False)
a2.set_ylim(-60, 25)
a.set(title="(d) Swing-up $\\to$ LQR from $3^\\circ$", xlabel="t [s]", ylabel="angle [deg]", ylim=(-200, 300))
h, l = [sum(z, []) for z in zip(a.get_legend_handles_labels(), a2.get_legend_handles_labels())]
a2.legend(h, l, loc="lower right")
fig.tight_layout()
fig.savefig(FIG)
print("figure ->", os.path.relpath(FIG, HERE))
