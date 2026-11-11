# Acrobot – Final Submission

Systems Thinking project: modelling, stability, control, swing-up and learned swing-up of an acrobot (two links, elbow motor only, |τ| ≤ 20 N·m).

## Folder layout

| Folder | Contents |
|---|---|
| `reports/` | final PDFs (list below) |
| `videos/` | 2K Manim animations, one per part |
| `code/` | all code: Python/PyBullet simulators, MATLAB scripts, Simulink models, RL |
| `report_source/` | LaTeX sources, figures and pseudo-code, plus the scripts that build the PDFs |

### Reports

| File | Contents |
|---|---|
| `Acrobot_Full_Report.pdf` | **main report**: everything, organised by topic, including the Simulink section (Sec. 12) |
| `Phase1_Report.pdf`, `Phase1_Report_IEEE.pdf` | Phase 1: model, equilibria, linearization (single-column and IEEE versions) |
| `Phase2_Report.pdf` | Phase 2: stability, single-angle feedback |
| `Phase3_Report.pdf` | Phase 3: controller design and limits under the torque limit |
| `Phase4_Report.pdf` | Phase 4: energy swing-up → LQR catch |
| `Phase5_Report.pdf` | Phase 5: learned (SAC) swing-up vs. energy swing-up |

### Videos

| File | Topic |
|---|---|
| `Phase1_Model_and_Equilibria.mp4` | model and equilibria |
| `Phase2_Stability_2K.mp4` | stability |
| `Phase3_Control_2K.mp4` | control |
| `Phase4_SwingUp_2K.mp4` | swing-up |
| `Phase5_RL_vs_Energy_SwingUp_2K.mp4` | RL vs. energy swing-up |

### Code

| Folder | What it is | How to run |
|---|---|---|
| `code/phase1/` | MATLAB: nonlinear model, linearization, linear simulation | `Acrobot_Phase1_Main.m` |
| `code/phase2/` | PyBullet model + interactive dashboard; `phase 2 deliverable/code/matlab` | see `code/phase2/README.md` |
| `code/phase3/` | LQR / pole-placement design, regions of attraction, dashboard | see `code/phase3/README.md` |
| `code/phase4/` | swing-up controller, dashboard, `run_all.sh` | see `code/phase4/README.md` |
| `code/phase5_rl/` | RL environment, SAC training, evaluation, trained policy | see `code/phase5_rl/README.md` |
| `code/simulink/` | four Simulink models and their build/test scripts | see `code/simulink/README.md` |

The Python folders import from each other (`phase4` → `phase3` → `phase2`), so keep the `code/phase*` folders side by side. Every phase README explains how to create the Python virtual environment (Python 3, `numpy`, `matplotlib`, `pybullet`, plus `stable-baselines3` for RL).

The Simulink models (`Acrobot_OpenLoop`, `Acrobot_SingleAnglePD`, `Acrobot_LQR`, `Acrobot_SwingUp`) need MATLAB R2025b with Simulink and the Control System Toolbox. In MATLAB, from `code/simulink/`:

```matlab
build_simulink_models   % regenerate the .slx files
run_simulink_models     % 15 checks, all PASS
```

## Rebuilding the reports

From `report_source/` (needs a LaTeX installation with `latexmk`):

```bash
python3 build_phase_reports.py
```

```bash
python3 build_full_report.py
```

The finished PDFs are written into `reports/`.
