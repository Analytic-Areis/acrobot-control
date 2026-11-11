"""
Training figure for the Phase 5 report.

    ../phase2/.venv/bin/python plot_training.py

Reads training_log.json, training_log_robust.json (train.py) and video_data.npz
(evaluate.py) and writes fig_training.pdf:
  (a) share of training episodes that reached the catch zone, for both policies;
  (b) shoulder angle in PyBullet from 3 deg for the three swing-ups.
"""

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.grid": True, "grid.alpha": 0.3})
cols = {"energy": "#2563eb", "rl": "#e46c12", "rl_robust": "#16a34a"}
labels = {"energy": "energy swing-up", "rl": "RL, 1 kg only", "rl_robust": "RL, random masses"}

logs = {}
for k, f in (("rl", "training_log.json"), ("rl_robust", "training_log_robust.json")):
    with open(os.path.join(HERE, f)) as fh:
        logs[k] = json.load(fh)
D = np.load(os.path.join(HERE, "video_data.npz"))

fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.3), layout="constrained")
for k, lg in logs.items():
    c = np.array(lg["curve"], float)
    axs[0].plot(c[:, 0] / 1e3, c[:, 1], color=cols[k], lw=1.4, ls="-" if k == "rl" else "--",
                label=f"{labels[k]} ({lg['episodes']:,} episodes, {lg['minutes']:.0f} min)")
axs[0].set_xscale("log")
axs[0].set(title="(a) learning curves", xlabel="training steps [thousands, log scale]",
           ylabel="episodes reaching catch zone [%]", ylim=(0, 105))
axs[0].legend(fontsize=6, loc="lower right")

for k in ("energy", "rl", "rl_robust"):
    axs[1].plot(D[f"{k}_t"], np.degrees(D[f"{k}_q1"]), color=cols[k], lw=1.4, label=labels[k])
for y in (180, -180):
    axs[1].axhline(y, color="k", ls=":", lw=0.8)
axs[1].set(title="(b) PyBullet from 3°: shoulder angle", xlabel="time [s]", ylabel="$q_1$ [deg]")
axs[1].legend(fontsize=6, loc="center right")
fig.savefig(os.path.join(HERE, "fig_training.pdf"), bbox_inches="tight")
print("wrote fig_training.pdf")
