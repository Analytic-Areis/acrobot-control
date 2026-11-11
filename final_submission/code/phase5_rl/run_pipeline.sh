#!/usr/bin/env bash
# Train -> evaluate -> video, one after another (from this phase5_rl folder):
#     bash run_pipeline.sh [training_steps]
set -e
cd "$(dirname "$0")"
PY=../phase2/.venv/bin/python
STEPS="${1:-400000}"

echo "== 0. model check $(date +%H:%M:%S)"
$PY - <<'EOF'
import numpy as np, acrobot_env as E
rng = np.random.default_rng(1)
err = max(np.max(np.abs(E.f_masses(x.reshape(4, 1), t)[:, 0] - E.acro.f_nonlinear(x, t)))
          for x, t in ((rng.uniform(-3, 3, 4), rng.uniform(-20, 20)) for _ in range(50)))
print("max difference parametric model vs project model:", err)
assert err < 1e-9
EOF

echo "== 1a. training, 1 kg links only ($STEPS steps) $(date +%H:%M:%S)"
$PY train.py "$STEPS"

echo "== 1b. training, random masses ($((2 * STEPS)) steps) $(date +%H:%M:%S)"
$PY train.py "$((2 * STEPS))" --robust

echo "== 2. evaluation $(date +%H:%M:%S)"
$PY evaluate.py
$PY plot_training.py

echo "== 3. video preview $(date +%H:%M:%S)"
(cd manim-sim && manim -ql --disable_caching rl_vs_energy.py RLvsEnergy)

echo "== 4. video 2K $(date +%H:%M:%S)"
(cd manim-sim && manim -r 2560,1440 --fps 60 --disable_caching rl_vs_energy.py RLvsEnergy)
cp manim-sim/media/videos/rl_vs_energy/1440p60/RLvsEnergy.mp4 RL_vs_Energy_SwingUp_2K.mp4
echo "== done $(date +%H:%M:%S)"
