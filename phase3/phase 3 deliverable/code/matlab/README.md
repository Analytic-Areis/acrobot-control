# Acrobot Project — Phase 3 MATLAB Code

**Team:** Bhargav (2025102061), Sreekar (2025102074), Kaushik (2025102071), Divijat (2025102040)

Requires MATLAB with the Control System Toolbox (`lqr`, `place`). Run everything from inside this `matlab/` folder.

## Files

| File | Purpose |
| :--- | :--- |
| `Acrobot_Phase3_Main.m` | Designs every controller, prints K, the closed-loop poles and the guaranteed (never-saturating) regions, simulates the nonlinear saturated loops with `ode45`, and maps the regions of attraction (saves `Fig_Phase3_ROA.pdf`). Runtime about 1–2 minutes; set `DO_MAPS = false` to skip the maps. |
| `Acrobot_Phase3_Design.m` | The Phase 3 controllers: E3 LQR (Q = I, R = 0.03), E3 pole placement (−3, −4, −5, −6), E2 and E4 LQR (Q = I, R = 0.1), E1 elbow PD (Kp = −5, Kd = 0.4). |
| `Acrobot_Nonlinear_Dynamics.m` | Nonlinear state equation from Phase 1. It saturates τ at ±20 N·m, so every simulation respects τ_max. |
| `Acrobot_Linearize.m` | Linearization A, B at an equilibrium (from Phase 1). |

## Expected output (matches the report)

**Gains and closed-loop poles**

| Controller | K | Closed-loop poles |
| :--- | :--- | :--- |
| E3 LQR | [−503.03 −203.89 −218.34 −101.52] | −39.86, −2.25 ± 0.10j, −1.34 |
| E3 pole placement | [−322.89 −134.61 −138.06 −66.00] | −6, −5, −4, −3 |
| E2 LQR | [6.60 46.49 5.23 13.24] | −4.26, −2.40, −1.24 ± 4.16j |
| E4 LQR | [−49.31 38.91 −13.19 6.10] | −5.73, −3.21, −0.96 ± 3.08j |
| E1 elbow PD | [0 −5 0 0.4] | −0.69 ± 2.31j, −0.52 ± 2.14j |

**Never-saturating ellipsoid** (|Δq1| at Δq2 = 0, and |Δq2| at Δq1 = 0):

- E3: 0.72°, 1.66°
- E2: 45.55°, 19.94°
- E4: 20.00°, 23.14°

**Nonlinear runs**

| Controller | From (Δq1, Δq2) | Result | Peak \|τ\| |
| :--- | :--- | :--- | :--- |
| E3 LQR | (2°, 0) | held | 17.56 |
| E3 LQR | (5.5°, 0) | held | 20 |
| E3 LQR | (7.5°, 0) | **not held** | 20 |
| E3 LQR | (−10°, 25°) | held | 3.34 |
| E3 pole placement | (2°, 0) | held | 11.27 |
| E2 LQR | (60°, 90°) | held | 20 |
| E4 LQR | (20°, 30°) | held | 4.47 |
| E1 elbow PD | (60°, −60°) | held | 5.28 |

**Regions of attraction** (coarse grid): E3 27.0% of ±40°, E2 99.4% of ±180°, E4 20.8% of ±180°. The report's finer Python grid gives 26.1%, 99.7% and 20.5%.
