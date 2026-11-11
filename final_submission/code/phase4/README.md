# Acrobot – Phase 4: Swing-up

Start hanging (E1), swing up to the upright E3 and hold it, with |τ| ≤ 20 N·m.

## The controller (hybrid)

1. **Swing-up: energy pumping.** The elbow follows a desired acceleration through partial feedback linearization:
   - Desired elbow acceleration: v = −k₁ q̃₂ − k₂ q̇₂ + k₃ q̇₁ (E − E_top)
   - Torque: τ = (M₂₂ − M₂₁M₁₂/M₁₁) v + h₂ − M₂₁h₁/M₁₁, where h = C q̇ − τ_g

   The k₃ term adds energy until the arm has exactly the upright energy, E_top = +24.525 J.
2. **Catch: switch to the Phase 3 LQR** when x̃ᵀ S x̃ < 5 (x̃ measured from E3).
3. **Retry:** if the arm leaves that region again (a failed catch, or a push), swing-up resumes automatically.

Gains: **k₁ = 20, k₂ = 2, k₃ = 3, switch threshold 5.** They were chosen by searching 144 gain/threshold combinations, then testing the winners from 40 random starts near hanging:

| Result | Value |
|---|---|
| Random starts swung up and held | **39 of 40** |
| Median time to first catch | ~4 s |
| Textbook gains k = (50, 5, 10), same threshold | 25 of 40 |

PyBullet check (30 s runs):

| Start (from hanging) | First catch | Final deviation from upright | Time saturated |
|---|---|---|---|
| 3° | 2.2 s | 0.000° | 2% |
| 0.5° | 1.9 s | 0.000° | 6% |
| (−10°, 20°) | 9.5 s | < 0.01° | 16% |

**Why the stiffer textbook gains do worse:** a stiff elbow (large k₁) asks for more torque than 20 N·m. Once the motor saturates, the elbow stops following the swing-up law, so the energy pumping no longer delivers the arm cleanly into the catch region.

| k₁ (with k₂ = 2, k₃ = 3) | Swing-up phase at the torque limit | Success |
|---|---|---|
| 20 | 26% | 39/40 |
| 50 | 38% | 4/40 |
| 100 | 45% | 2/40 |

A very small k₃ (0.3) fails for the opposite reason: it pumps energy too slowly to ever arrive (0/40).

## Run

From this `phase4` folder (uses the Phase 2 virtual environment):

```bash
../phase2/.venv/bin/python acrobot_phase4_sim.py
```

It opens on **E1 (hanging), 3° offset, Swing-up → LQR**: watch it pump up and get caught. The plots show the deviation from E1, so −180° in q1 means upright.

**Things to try:**
- Press **Kick shoulder** once it is up: it falls, swings back up and is caught again.
- Type other start offsets.
- Pick E3 and **Full-state LQR** for comparison: it only works close to upright.

## Files

| File | What |
|---|---|
| `acrobot_phase4_design.py` | swing-up law, energy, catch rule, `SwingUpLQR` controller |
| `acrobot_phase4_sim.py` | dashboard (Phase 3 dashboard + Swing-up → LQR) |
| `phase 4 deliverable/code/make_phase4_results.py` | regenerates every Phase 4 figure and number (`results.json`) |
| `phase 4 deliverable/code/matlab/` | MATLAB version: swing-up runs and the 40-start robustness test (see its README) |
| `phase 4 deliverable/single_column/Phase4_Acrobot_Report.pdf` | report: Phases 1–4, each starting on its own page |
| `phase 4 deliverable/manim-sim/acrobot_phase4.py` | Phase 4 animation; rendered: `phase 4 deliverable/Acrobot_Phase4_2K.mp4` |
| `run_all.sh` | runs everything in order (`bash run_all.sh`, or one step: `bash run_all.sh 2`) |
