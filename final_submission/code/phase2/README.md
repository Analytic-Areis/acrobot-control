# Acrobot – Phase 2 interactive simulator

`acrobot_phase2_sim.py` simulates the Acrobot with the **PyBullet** physics engine and shows it in a live dashboard. You can change the equilibrium, the initial condition and the controller while it runs. It uses the same model and parameters as the Phase 1 report: `q1` from the downward vertical, `q2` relative to link 1, motor at the elbow only, `|τ| ≤ 20 N·m`, no friction.

## 1. Setup (once)

You need Python 3.10 or newer. From this folder:

```bash
python3 -m venv .venv
.venv/bin/pip install numpy scipy matplotlib pybullet
```

This folder already contains a working `.venv`, so you can skip this step on this machine.

> **macOS + Python 3.14 only.** PyBullet has no prebuilt wheel for Python 3.14, and its source build fails because of a `fdopen` macro clash in the bundled zlib. Patch that one line and build it:
>
> ```bash
> .venv/bin/pip download --no-deps --no-binary :all: pybullet
> tar xzf pybullet-*.tar.gz && cd pybullet-*/
> sed -i '' '128s|.*|/* fdopen exists on modern macOS */|' examples/ThirdPartyLibs/zlib/zutil.h
> ../.venv/bin/pip install .
> ```
>
> On Python 3.10–3.12, a plain `pip install pybullet` works.

## 2. Run

```bash
.venv/bin/python acrobot_phase2_sim.py              # prints the Phase 2 analysis, then opens the dashboard
.venv/bin/python acrobot_phase2_sim.py --no-analysis
.venv/bin/python acrobot_phase2_sim.py --verify     # PyBullet vs. RK4 of the helper equations
.venv/bin/python acrobot_phase2_sim.py --batch      # headless runs; saves 8 plots to ./plots
```

## 3. The dashboard

| Area | What it shows |
|---|---|
| **Acrobot (left)** | Solid blue/orange = nonlinear PyBullet model. Dashed grey = linearized model `x̃' = A x̃ + B τ` started from the same initial condition. Faint thick line = target equilibrium `q*`. Red trail = arm tip. Purple arc at the elbow = motor torque (length ∝ \|τ\|/20). **Drag a link with the mouse** to set the initial pose. |
| **Joint deviation** | Δq1, Δq2 against time: nonlinear (solid) vs. linear (dashed). |
| **Elbow torque** | τ(t) with the ±20 N·m limit. |
| **s-plane** | Open-loop poles (grey ×), closed-loop poles of the linearized loop (green = stable, red = unstable), zeros of the measured transfer function (○). Updates instantly when you change gains. |
| **Status box** | Δq, τ (`SAT` when saturated), energy change, and the stability verdict. |

**Controls**

- **Equilibrium:** E1 down-down, E2 down-up, E3 up-up (upright), E4 up-down.
- **Controller:**
  - *Open loop* (τ = 0)
  - *PD on one angle*: τ = −(Kp·e + Kd·ė), using only the measured joint
  - *Observer + LQR*: a Luenberger observer driven by **one** measured angle, plus LQR on the estimate
- **Measured angle:** q1 (shoulder) or q2 (elbow); used by the two controllers.
- **Initial condition:** Δq1(0), Δq2(0) in degrees (sliders) and ω1(0), ω2(0) in °/s. **Every value also has a text box: type a number and press Enter.**
- **PD gains & speed:** Kp, Kd (typed values may go beyond the slider range) and simulation speed.
- **Buttons:**
  - **Pause**
  - **Reset**: restart from the same initial condition
  - **Reset to q\***: start exactly on the equilibrium, at rest
  - **Kick shoulder**: a +0.8 rad/s velocity disturbance
  - **Hide/Show linear**
  - **Save plot**: writes a PNG to `./plots`
- **Keys:**

  | Key | Action |
  |---|---|
  | space | pause |
  | r | reset |
  | 0 | reset to q* |
  | k | kick |
  | s | save plot |
  | q / Esc | quit |

  Ctrl+C in the terminal also quits.

The dashed linear model disappears once it moves more than 180° from `q*`. At E2, E3 and E4 the linear model grows like `e^{p t}` with no bound, so beyond that point its prediction has no physical meaning. The status box then says *linear model diverged*.

## 4. Things to try (the Phase 2 results)

1. **E1 + Open loop:** the arm oscillates forever with the energy constant. This is marginal stability (poles at ±j2.24 and ±j6.10).
2. **E2 / E3 / E4 + Open loop, then *Reset to q\**, then *Kick*:** the arm leaves the equilibrium. The dashed linear model leaves along the same path at first. These equilibria have 1, 2 and 1 RHP poles respectively.
3. **E3 + PD on one angle:** sweep Kp and Kd for q1 or q2. A red closed-loop pole always stays in the right half-plane, so no PD gain stabilizes the upright from a single angle.
4. **E3 + Observer + LQR, measuring q1, Δq1(0) = 1:** the arm is held upright. Try 3: it falls because τ saturates. The limit is about 1.4° measuring q1 and about 0.2° measuring q2.

## 5. Files

- `acrobot_phase2_sim.py`: model, linearization, controllers, PyBullet simulator, dashboard
- `phase 2 deliverable/code/make_phase2_results.py`: regenerates every figure and number in the Phase 2 report (and the data for the video)
- `phase 2 deliverable/single_column/`: Phase 2 report (LaTeX + PDF)
- `phase 2 deliverable/manim-sim/acrobot_phase2.py`: Phase 2 animation
