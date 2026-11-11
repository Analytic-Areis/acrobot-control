# Acrobot Project — Phase 4 MATLAB Code

**Team:** Bhargav (2025102061), Sreekar (2025102074), Kaushik (2025102071), Divijat (2025102040)

Requires MATLAB with the Control System Toolbox (`lqr`). Run from inside this `matlab/` folder; it takes about 2 minutes.

## Files

| File | Purpose |
| :--- | :--- |
| `Acrobot_Phase4_Main.m` | LQR at the upright, swing-up runs from three starts (fixed-step RK4, 1 ms, 30 s), plots of angles / energy / torque (`Fig_Phase4_SwingUp.pdf`), and a robustness test from 40 random starts. Set `DO_ROBUST = false` to skip the slowest part. |
| `Acrobot_Phase4_Controller.m` | Hybrid controller, the same law as the Python model: an energy-pumping swing-up through partial feedback linearization, which switches to the Phase 3 LQR when x̃ᵀSx̃ < 5. |
| `Acrobot_Nonlinear_Dynamics.m`, `Acrobot_Linearize.m` | Model and linearization from Phase 1. |

## Expected output

**LQR at E3:** K = [−503.03 −203.89 −218.34 −101.52]

**Swing-up runs**

| Start (from hanging) | First catch | Final deviation from upright |
| :--- | :--- | :--- |
| (3°, 0) | 2.17 s | 0.000° |
| (0.5°, 0) | 1.91 s | 0.000° |
| (−10°, 20°) | 8.74 s | 0.000° |

**Robustness (40 random starts)**

| Gains | Swung up and held |
| :--- | :--- |
| [20 2 3] | 39 of 40 |
| [50 5 10] | 29 of 40 |

The Python/PyBullet value for [50 5 10] is 25 of 40; the swing-up is sensitive to the integrator, but both show the same gap.
