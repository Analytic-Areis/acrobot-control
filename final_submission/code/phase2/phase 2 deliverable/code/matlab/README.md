# Acrobot Project — Phase 2 MATLAB Code

**Team:** Bhargav (2025102061), Sreekar (2025102074), Kaushik (2025102071), Divijat (2025102040)

Requires MATLAB with the Control System Toolbox. Run from inside this `matlab/` folder; it takes under a minute.

## Files

| File | Purpose |
| :--- | :--- |
| `Acrobot_Phase2_Main.m` | For each equilibrium: poles, zeros, ranks, characteristic polynomial and stability verdict. Nonlinear `ode45` check of growth rates. Single-angle PD test at the upright, with a gain-grid search (saves `Fig_Phase2_PD.pdf`). |
| `Acrobot_Nonlinear_Dynamics.m`, `Acrobot_Linearize.m` | Model and linearization from Phase 1. |

## Expected output (matches the report)

**Stability of each equilibrium**

| Equilibrium | eig(A) | Zeros of G₁ / G₂ | Verdict |
| :--- | :--- | :--- | :--- |
| E1 | ±6.1011j, ±2.2400j | ±2.0519j / ±2.2934j | marginally stable |
| E2 | ±4.3010j, ±3.1775 | ±5.4523 / ±2.7200j | unstable, 1 RHP pole |
| E3 | ±6.1011, ±2.2400 | ±2.0519 / ±2.2934 | unstable, 2 RHP poles |
| E4 | ±4.3010, ±3.1775j | ±5.4523j / ±2.7200 | unstable, 1 RHP pole |

- **Characteristic polynomial:** the s³ and s¹ coefficients are 0 at every equilibrium.
- **Ranks:** controllability rank = 4, and observability rank from q1 alone or from q2 alone = 4.

**Nonlinear check** (growth rate vs largest RHP pole):

| Equilibrium | Measured | Pole | Error |
| :--- | :--- | :--- | :--- |
| E2 | 3.171 | 3.1775 | 0.20% |
| E3 | 6.088 | 6.1011 | 0.21% |
| E4 | 4.300 | 4.3010 | 0.03% |

E1 stays bounded: max |q1| = 10.00° over 20 s, never decaying.

**Upright E3, PD on one angle:** impossible for both angles, because the coefficients have opposite signs.

| Measured angle | Smallest max Re(pole) on the grid | At |
| :--- | :--- | :--- |
| q1 | 0.962 | Kp = −15, Kd = 0 |
| q2 | 2.294 | Kp = 400, Kd = 150 |
