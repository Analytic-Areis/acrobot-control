# Acrobot – RL swing-up vs energy swing-up

A learned swing-up (SAC, stable-baselines3) replaces the Phase 4 energy-pumping law. Both controllers hand over to the **same Phase 3 LQR** inside the same catch zone (x̃ᵀSx̃ < 5), so the only difference is who does the swing-up.

## Results

40 random starts near hanging; |τ| ≤ 20 N·m; "held" means caught and balanced within 30 s. Every controller assumes 1 kg links.

| | Energy swing-up | RL, 1 kg only | RL, random masses |
|---|---|---|---|
| Held, correct masses | 35/40 | **40/40** | **40/40** |
| Median time to catch | 3.98 s | **1.71 s** | **1.71 s** |
| Time at the torque limit | 8.8% | 0.1% | **0.0%** |
| Median elbow work | 245 J | 144 J | **130 J** |
| Held, 0.6 kg (−40%, never practised) | 39/40 | 2/40 | **40/40** |
| Held, 0.8 kg (−20%) | 37/40 | 38/40 | **40/40** |
| Held, 1.2 kg (+20%) | 35/40 | 15/40 | **40/40** |
| Held, 1.4 kg (+40%, never practised) | **33/40** | 3/40 | 2/40 |

PyBullet, from 3° off hanging: energy caught at 2.17 s, RL (1 kg only) at 1.76 s, RL (random masses) at 1.89 s; all held.

**Takeaway:** the first policy was trained only on 1 kg links, so it learned that robot's timing rather than the physics, and on heavier links it failed (15/40). The robust policy differs in **only two ways**:

1. every practice swing uses new random link masses, each in [0.7, 1.3] kg (domain randomization);
2. it also observes the missing energy (E_top − E)/E_top. This is computed from the other six inputs with the 1 kg model, like the energy controller does, so it is a hint, not new information.

It holds 40/40 at every mass from 0.6 to 1.2 kg, at no cost on the correct model. At 1.4 kg, far heavier than anything it practised on, it still reaches the catch zone every time but the LQR hand-over keeps failing (median 12 hand-overs in 30 s against 1 for the energy law), because the reward stops at "reach the catch zone" and never trains the hand-over. A learned controller is reliable over the variety it practised on; the physics-based law extrapolates further.

The energy controller scores 35/40 here, not Phase 4's 39/40, because this test allows 30 s instead of 40 s.

## Files

| File | What |
|---|---|
| `acrobot_env.py` | Gymnasium environment (mass-parametric model, reward, catch zone) and the `RLThenLQR` / `EnergyThenLQR` controllers |
| `train.py` | SAC training: `train.py 1000000` → `policies/sac_swingup.zip`; `train.py 2000000 --robust` → `policies/sac_swingup_robust.zip` (each with its `training_log*.json`) |
| `evaluate.py` | all three swing-ups: 40 starts × five link masses (0.6–1.4 kg), hand-over counts, PyBullet runs; writes `results.json`, `fig_compare.pdf`, `video_data.npz` |
| `plot_training.py` | learning curves of both policies and the PyBullet runs; writes `fig_training.pdf` |
| `manim-sim/rl_vs_energy.py` | Explanatory comparison video; rendered: `RL_vs_Energy_SwingUp_2K.mp4` |
| `Phase5_Report.pdf` | the Phase 5 report (copied here when the reports are rebuilt in `final_submission/report_source`) |
| `run_pipeline.sh` | Runs everything in order: `bash run_pipeline.sh 1000000` (training ~12 + 26 min, evaluation ~8 min, video ~5 min) |

## Training details

- **Algorithm:** SAC with a 2×256 network, 4 parallel environments, seed 0. Original policy: 1,000,000 steps (9,206 practice swings, 12.4 min on a laptop CPU). Robust policy: 2,000,000 steps (20,489 practice swings, 25.9 min).
- **Observation:** [cos q1, sin q1, cos q2, sin q2, q̇1/10, q̇2/10]; the robust policy adds (E_top − E)/E_top.
- **Action:** τ = 20·a, with a ∈ [−1, 1], held for 20 ms.
- **Reward:** −(E−E_top)²/E_top² − 0.3(1+cos q1) − 0.1(1−cos q2) − 0.001(τ/20)², plus 100 on reaching the catch zone (the episode then ends).

Uses the Phase 2 virtual environment (`../phase2/.venv`) with `gymnasium`, `stable-baselines3` and `torch` added.
