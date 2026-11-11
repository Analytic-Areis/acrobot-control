"""
Compare three swing-ups that all hand over to the same Phase 3 LQR:
  energy     the Phase 4 energy-pumping law
  rl         SAC policy trained on 1 kg links only
  rl_robust  SAC policy trained on random masses (0.7-1.3 kg) that also sees the missing energy

    ../phase2/.venv/bin/python evaluate.py

1. 40 random starts near hanging (same as Phase 4), nominal model (RK4, 2 ms):
   share held, time to first catch, time at the torque limit, energy used.
2. Robustness: same starts with the link masses changed by -20% / +20% (inside the
   robust policy's 0.7-1.3 kg training range) and -40% / +40% (outside it).
   All controllers still assume the nominal 1 kg model.
3. PyBullet runs from the same start for the video and plots.

Writes results.json, fig_compare.pdf and video_data.npz.
"""

import json
import os
import time

import numpy as np
import pybullet as p
from stable_baselines3 import SAC

import acrobot_env as E

HERE = os.path.dirname(os.path.abspath(__file__))
acro = E.acro
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

policy = SAC.load(os.path.join(HERE, "policies", "sac_swingup"), device="cpu")
policy_robust = SAC.load(os.path.join(HERE, "policies", "sac_swingup_robust"), device="cpu")
R, NPZ = {}, {}
t0 = time.time()


def starts40():
    rng = np.random.default_rng(0)
    s = np.column_stack([rng.uniform(-0.6, 0.6, 40), rng.uniform(-0.6, 0.6, 40),
                         rng.uniform(-0.5, 0.5, 40), rng.uniform(-0.5, 0.5, 40)])
    s[:3] = [[0.01, 0, 0, 0], [-0.01, 0, 0, 0], [0.05, 0, 0, 0]]
    return s


def run_model(make_ctrl, x0, m=(1.0, 1.0), T=30.0, h=2e-3):
    """One run on the RK4 model with link masses m; returns summary."""
    c = make_ctrl()
    x = np.array(x0, float)
    t_catch, sat, work, entries, was_lqr = None, 0, 0.0, 0, False
    n = int(T / h)
    for i in range(n):
        tau = c(x, h)
        if c.mode == "lqr" and t_catch is None:
            t_catch = i * h
        entries += c.mode == "lqr" and not was_lqr          # hand-overs to the LQR
        was_lqr = c.mode == "lqr"
        sat += abs(tau) >= E.TAU_MAX - 1e-6
        work += abs(tau * x[3]) * h                     # |mechanical power| at the elbow
        x = E.rk4(x.reshape(4, 1), tau, h, *m)[:, 0]
    e = E.top_error(x)
    held = bool(np.all(np.abs(np.degrees(e[:2])) < 1) and np.all(np.abs(e[2:]) < 0.05))
    return dict(held=held, t_catch=t_catch, sat=sat / n, work=work, entries=entries)


def batch(name, make_ctrl, m=(1.0, 1.0)):
    rows = [run_model(make_ctrl, s, m) for s in starts40()]
    held = [r for r in rows if r["held"]]
    out = dict(held=len(held), of=len(rows),
               median_catch_s=round(float(np.median([r["t_catch"] for r in held])), 2) if held else None,
               saturated_pct=round(100 * float(np.mean([r["sat"] for r in rows])), 1),
               median_work_J=round(float(np.median([r["work"] for r in held])), 1) if held else None,
               reached_zone=sum(r["entries"] > 0 for r in rows),
               median_handovers=float(np.median([r["entries"] for r in rows])))
    print(f"{name:28s} masses {m}: {out}  ({time.time() - t0:.0f} s)", flush=True)
    return out


ctrls = {"energy": E.EnergyThenLQR, "rl": lambda: E.RLThenLQR(policy),
         "rl_robust": lambda: E.RLThenLQR(policy_robust)}
R["nominal"] = {k: batch(k, c) for k, c in ctrls.items()}
R["mass_minus20"] = {k: batch(k, c, (0.8, 0.8)) for k, c in ctrls.items()}
R["mass_plus20"] = {k: batch(k, c, (1.2, 1.2)) for k, c in ctrls.items()}
R["mass_minus40"] = {k: batch(k, c, (0.6, 0.6)) for k, c in ctrls.items()}   # outside the training range
R["mass_plus40"] = {k: batch(k, c, (1.4, 1.4)) for k, c in ctrls.items()}

# ---------------------------------------------------------- PyBullet runs
sim = acro.AcrobotSim()
DT = acro.DT


def pyb(make_ctrl, start_deg=(3.0, 0.0), T=12.0):
    sim.configure(1, start_deg, 0, 0, 0, 0)
    c = make_ctrl()
    lg = {k: [] for k in ("t", "q1", "q2", "tau", "E", "lqr")}
    t_catch = None
    for i in range(int(round(T / DT))):
        q, qd = sim.state()
        x = np.r_[q, qd]
        tau = c(x, DT)
        if c.mode == "lqr" and t_catch is None:
            t_catch = i * DT
        p.setJointMotorControl2(sim.robot, 1, p.TORQUE_CONTROL, force=tau, physicsClientId=sim.cid)
        p.stepSimulation(physicsClientId=sim.cid)
        if i % 20 == 0:
            lg["t"].append(i * DT); lg["q1"].append(q[0]); lg["q2"].append(q[1]); lg["tau"].append(tau)
            lg["E"].append(acro.energy(q, qd)); lg["lqr"].append(c.mode == "lqr")
    q, qd = sim.state()
    e = E.top_error(np.r_[q, qd])
    out = {k: np.asarray(v, float) for k, v in lg.items()}
    return out, dict(t_catch=None if t_catch is None else round(t_catch, 2),
                     held=bool(np.all(np.abs(np.degrees(e[:2])) < 1)),
                     end_dev=[round(float(v), 3) for v in np.degrees(e[:2])])


R["pybullet"] = {}
for k, c in ctrls.items():
    lg, s = pyb(c)
    R["pybullet"][k] = s
    for f, v in lg.items():
        NPZ[f"{k}_{f}"] = v
print("pybullet", R["pybullet"])
np.savez_compressed(os.path.join(HERE, "video_data.npz"), **NPZ)

# ----------------------------------------------------------------- figure
plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.grid": True, "grid.alpha": 0.3})
fig = plt.figure(figsize=(7.2, 4.4), layout="constrained")
gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.05])
axs = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])]
cols = {"energy": "#2563eb", "rl": "#e46c12", "rl_robust": "#16a34a"}
labels = {"energy": "energy swing-up", "rl": "RL, 1 kg only", "rl_robust": "RL, random masses"}
for k in ctrls:
    axs[0].plot(NPZ[f"{k}_t"], NPZ[f"{k}_E"], color=cols[k], label=labels[k])
    axs[1].plot(NPZ[f"{k}_t"], NPZ[f"{k}_tau"], color=cols[k], lw=0.8)
axs[0].axhline(E.E_TOP, color="k", ls=":", lw=0.8)
axs[0].set_title("(a) energy, PyBullet from 3°"); axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("E [J]")
axs[0].legend(fontsize=6.5)
axs[1].set_title("(b) elbow torque"); axs[1].set_xlabel("time [s]"); axs[1].set_ylabel("τ [N m]")
cases = [("mass_minus40", "0.6 kg (−40%)"), ("mass_minus20", "0.8 kg (−20%)"), ("nominal", "1 kg (correct)"),
         ("mass_plus20", "1.2 kg (+20%)"), ("mass_plus40", "1.4 kg (+40%)")]
for xo in (0, 4):                                    # outside the robust policy's training range
    axs[2].axvspan(xo - 0.5, xo + 0.5, color="0.92", zorder=0)
axs[2].text(0, 50, "never practised", ha="center", fontsize=6, color="0.4")
axs[2].text(4, 50, "never practised", ha="center", fontsize=6, color="0.4")
axs[2].set_xlim(-0.5, 4.5)
xs = np.arange(len(cases))
for j, k in enumerate(ctrls):
    v = [R[c][k]["held"] for c, _ in cases]
    b = axs[2].bar(xs + (j - 1) * 0.27, v, 0.27, color=cols[k], label=labels[k])
    axs[2].bar_label(b, fontsize=5.5, padding=1)
axs[2].set_xticks(xs, [l for _, l in cases])
axs[2].set_xlabel("true link masses (all controllers assume 1 kg; the robust policy practised on 0.7–1.3 kg)")
axs[2].set_title("(c) held from 40 random starts"); axs[2].set_ylim(0, 62); axs[2].set_yticks(range(0, 41, 10))
axs[2].legend(fontsize=6, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.0))
fig.savefig(os.path.join(HERE, "fig_compare.pdf"), bbox_inches="tight")
p.disconnect(sim.cid)
with open(os.path.join(HERE, "results.json"), "w") as fh:
    json.dump(R, fh, indent=1)
print(json.dumps(R, indent=1))
print(f"done in {time.time() - t0:.0f} s")
