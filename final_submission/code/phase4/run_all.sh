#!/usr/bin/env bash
# Phase 4 - run every step, one after another, from this phase4 folder:
#     bash run_all.sh            (all steps)
#     bash run_all.sh 3          (only step 3)
set -e
cd "$(dirname "$0")"
PY=../phase2/.venv/bin/python
MATLAB=/Applications/MATLAB_R2025b.app/bin/matlab
D="phase 4 deliverable"
STEP="${1:-all}"

if [[ "$STEP" == all || "$STEP" == 1 ]]; then
  echo "== 1. figures + numbers (PyBullet + gain search, ~1 min)"
  $PY "$D/code/make_phase4_results.py"
fi

if [[ "$STEP" == all || "$STEP" == 2 ]]; then
  echo "== 2. MATLAB version (~1-3 min)"
  $MATLAB -batch "cd('$PWD/$D/code/matlab'); Acrobot_Phase4_Main"
fi

if [[ "$STEP" == all || "$STEP" == 3 ]]; then
  echo "== 3. video, 2K 60 fps (~3-5 min) -> $D/Acrobot_Phase4_2K.mp4"
  (cd "$D/manim-sim" && manim -r 2560,1440 --fps 60 --disable_caching acrobot_phase4.py AcrobotPhase4)
  cp "$D/manim-sim/media/videos/acrobot_phase4/1440p60/AcrobotPhase4.mp4" "$D/Acrobot_Phase4_2K.mp4"
fi

if [[ "$STEP" == all || "$STEP" == 4 ]]; then
  echo "== 4. report (Phases 1-4)"
  (cd "$D/single_column" && latexmk -pdf -interaction=nonstopmode Phase4_Acrobot_Report.tex && latexmk -c)
fi

if [[ "$STEP" == all || "$STEP" == 5 ]]; then
  echo "== 5. interactive simulator (close the window or press q to finish)"
  $PY acrobot_phase4_sim.py
fi
echo "done"
