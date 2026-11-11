# Acrobot – Simulink models (Phases 2–4)

Two scripts create and test the Simulink models. Every model has the same nonlinear plant: a MATLAB Function block computing ẋ = f(x, τ) from the given equations of motion, an Integrator with initial state `x0`, and a ±20 N·m Saturation block in front of the plant.

| Model | Phase | Contents |
|---|---|---|
| `Acrobot_OpenLoop.slx` | 2 | τ = 0, the nonlinear plant next to a linear State-Space block (A, B) from the same start |
| `Acrobot_SingleAnglePD.slx` | 2 | PD on one measured angle at the upright (gains Kp, Kd; measured angle `j_meas`) |
| `Acrobot_LQR.slx` | 3 | full-state feedback τ = sat(−K x̃) at any equilibrium `qs` |
| `Acrobot_SwingUp.slx` | 4 | energy-pumping swing-up → LQR catch when x̃ᵀSx̃ < 5 |

Each model logs `x_nl` (state) and `tau_log` (torque) to the workspace and has a Scope on the two angles. `Acrobot_SwingUp` also logs `lqr_on`, which is 1 while the LQR is active.

## 1. Requirements

MATLAB R2025b with **Simulink** and the Control System Toolbox (for `lqr`). Simulink was added to this installation with the MathWorks Package Manager:

```bash
mpm install --release=R2025b --destination=/Applications/MATLAB_R2025b.app --products Simulink
```

Check it with `ver simulink`. Stateflow is not needed: the MATLAB Function block is part of Simulink.

## 2. Build and test

In MATLAB, from this folder:

```matlab
build_simulink_models     % creates the four .slx files here and copies them into the phase folders
run_simulink_models       % simulates each model; every line should end in PASS
```

The built models are copied to:

| Phase | Folder |
|---|---|
| 2 | `phase2/phase 2 deliverable/code/simulink/` (OpenLoop, SingleAnglePD) |
| 3 | `phase3/phase 3 deliverable/code/simulink/` (LQR) |
| 4 | `phase4/phase 4 deliverable/code/simulink/` (SwingUp) |

## Results (`run_simulink_models`, all PASS)

**Open loop (τ = 0)**

| Equilibrium | Measured | Linear prediction |
|---|---|---|
| E1, from (10°, −10°) | bounded, max deviation 14.7° over 8 s | stable (marginally, no damping) |
| E2 | growth rate 3.171 /s | pole 3.1775 (error 0.19 %) |
| E3 | growth rate 6.088 /s | pole 6.1011 (error 0.22 %) |
| E4 | growth rate 4.300 /s | pole 4.3010 (error 0.03 %) |

**Single-angle PD at E3, from 1°:** both the best q1 gains (Kp = −15, Kd = 0) and the best q2 gains (Kp = 400, Kd = 150) fall (180° deviation).

**LQR (Q = I)**

| Case | Result | Peak \|τ\| (N·m) |
|---|---|---|
| E3, R = 0.03, from 2° | held | 17.56 |
| E3, from 5.5° | held | 20.00 (saturates) |
| E3, from 7.5° | falls, as expected | 20.00 |
| E3, from (−10°, 25°) | held | 3.34 |
| E2, R = 0.1, from (60°, 90°) | held | 20.00 |
| E4, R = 0.1, from (20°, 30°) | held | 4.47 |

**Swing-up → LQR (gains 20, 2, 3; switch at x̃ᵀSx̃ < 5)**

| Start | First catch | End |
|---|---|---|
| (3°, 0°) | 2.17 s | held upright |
| (0.5°, 0°) | 1.90 s | held upright |
| (−10°, 20°) | 8.10 s | held upright |

These match the Python and MATLAB-script results in the reports (for example, the 2.17 s catch from 3°).

## Report figures and the Python cross-check

```matlab
make_simulink_figures     % run from the MATLAB desktop: exports the diagrams (batch mode leaves block icons blank) and trajectories
```

```bash
../phase2/.venv/bin/python compare_with_python.py
```

- `make_simulink_figures.m` writes `final_submission/report_source/figures/slx_*.png` and `results/*.csv`.
- `compare_with_python.py` re-simulates each case with the report's Python model and controllers (RK4, at most 0.1 ms per step, landing on the Simulink time points). It writes `results/comparison.json` and `final_submission/report_source/figures/fig_simulink.pdf`.

| Case | Max. difference, Simulink vs Python |
|---|---|
| LQR gain K | 1.2e-13 (relative) |
| Open loop E1 / E3, LQR from 2° / 5.5° | about 1e-10 deg |
| LQR from 7.5° (both fall) | 0.17 deg |
| Swing-up from 3° | 0.0066 deg; catch 2.16691 s vs 2.16693 s |

All of this is written up in Section 12 of `final_submission/reports/Acrobot_Full_Report.pdf`.

## Using a model by hand

Set the workspace variables, then press **Run** or call `sim`:

```matlab
% example: LQR at the upright from a 5 deg offset
[A, B] = Acrobot_Linearize(pi, 0);   % from the phase 3/4 code/matlab folder
K = lqr(A, B, eye(4), 0.03);  qs = [pi; 0];
x0 = [pi + deg2rad(5); 0; 0; 0];  Tstop = 10;
out = sim('Acrobot_LQR');  plot(out.x_nl)
```

**Variables each model needs**

| Model | Variables |
|---|---|
| all | `Tstop`, `x0` |
| OpenLoop | `A`, `B`, `x0_lin` |
| SingleAnglePD | `qs`, `Kp`, `Kd`, `j_meas` |
| LQR | `qs`, `K` |
| SwingUp | `K`, `S`, `gains`, `c_switch` |
