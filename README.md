# Acrobot: Modelling, Stability Analysis, Underactuated Control & Reinforcement Learning

[![MATLAB](https://img.shields.io/badge/Platform-MATLAB%20%7C%20Simulink-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.14-blue.svg)](https://www.python.org/)
[![Physics](https://img.shields.io/badge/Physics-PyBullet-purple.svg)](https://pybullet.org/)
[![RL](https://img.shields.io/badge/RL-Gymnasium%20%7C%20SAC%20%7C%20SB3-green.svg)](https://stable-baselines3.readthedocs.io/)
[![Course](https://img.shields.io/badge/Course-Systems%20Thinking%20%26%20Control-darkblue.svg)](#)
[![Status](https://img.shields.io/badge/Phases%201--5-Validated-brightgreen.svg)](#)
[![License](https://img.shields.io/badge/License-Academic%20Use-lightgrey.svg)](#)

A comprehensive systems and control engineering project investigating the **Acrobot**—a classic 2-DOF underactuated planar robotic manipulator with an actuator strictly at the elbow joint and a completely unactuated, passive shoulder joint, subject to hard motor torque saturation $|\tau| \le 20\,\text{N}\cdot\text{m}$.

This repository provides an end-to-end analytical, numerical, simulation, and learning pipeline across five complete project phases, complemented by an interactive visual web explainer, high-definition 2K Manim animations, a 4-model Simulink test suite, and peer-reviewed IEEE-format reports.

---

## Table of Contents

1. [System Architecture & Physics](#1-system-architecture--physics)
2. [Phase-by-Phase Technical Breakdown](#2-phase-by-phase-technical-breakdown)
   - [Phase 1: Analytical Mechanics, Equilibria & Linearization](#phase-1-analytical-mechanics-equilibria--linearization)
   - [Phase 2: Open-Loop Stability & Single-Angle Control Failure Proof](#phase-2-open-loop-stability--single-angle-control-failure-proof)
   - [Phase 3: Multivariable LQR Balancing & Basin of Attraction](#phase-3-multivariable-lqr-balancing--basin-of-attraction)
   - [Phase 4: Nonlinear Hybrid Swing-Up & Catch Control](#phase-4-nonlinear-hybrid-swing-up--catch-control)
   - [Phase 5: Deep Reinforcement Learning (SAC) vs. Energy Swing-Up](#phase-5-deep-reinforcement-learning-sac-vs-energy-swing-up)
3. [Simulink Model Suite](#3-simulink-model-suite)
4. [Interactive Web Explainer & 2K Animations](#4-interactive-web-explainer--2k-animations)
5. [Repository Structure](#5-repository-structure)
6. [Getting Started & Reproduction](#6-getting-started--reproduction)
7. [Comprehensive Quantitative Results](#7-comprehensive-quantitative-results)
8. [Course Context & Deliverables](#8-course-context--deliverables)
9. [References](#9-references)

---

## 1. System Architecture & Physics

```
          Passive Shoulder (Pivot)
                    (O)
                    / \
                   /   \
                  /  q1 \
                 /       \
          Link 1 [m1, l1] \   q1 = 0 (straight down)
               /
             (O)  Actuated Elbow (Torque: tau, |tau| <= 20 N*m)
              \
               \   q2 (relative to Link 1)
                \
          Link 2 [m2, l2]
```

The Acrobot configuration vector is $q = [q_1, q_2]^T \in \mathbb{T}^2$ and the state vector is $x = [q_1, q_2, \dot{q}_1, \dot{q}_2]^T \in \mathbb{R}^4$:
* **$q_1$**: Shoulder angle measured counter-clockwise from the downward vertical ($q_1 = 0$: hanging straight down; $q_1 = \pi$: upright).
* **$q_2$**: Elbow angle measured relative to Link 1 ($q_2 = 0$: straight configuration).
* **$\tau$**: Scalar actuation torque applied across the elbow joint, strictly bounded by $|\tau| \le \tau_{\max} = 20\,\text{N}\cdot\text{m}$.
* **Coupling**: The shoulder joint possesses zero direct control authority ($B = [0, 1]^T$). Acceleration of Link 1 is produced exclusively through inertial and Coriolis coupling terms in $M(q)$ and $C(q, \dot{q})$.

### Physical Parameters (Benchmark Rod Model)

| Parameter | Symbol | Nominal Value | Unit | Definition |
| :--- | :--- | :--- | :--- | :--- |
| **Link Masses** | $m_1, m_2$ | $1.0, 1.0$ | $\text{kg}$ | Mass of Link 1 and Link 2 |
| **Link Lengths** | $l_1, l_2$ | $1.0, 2.0$ | $\text{m}$ | Kinematic length of each link |
| **COM Locations** | $l_{c1}, l_{c2}$ | $0.5, 1.0$ | $\text{m}$ | Centers of mass from proximal joints ($l_i / 2$) |
| **COM Inertias** | $I_1, I_2$ | $0.083, 0.330$ | $\text{kg}\cdot\text{m}^2$ | Moment of inertia about each link COM |
| **Gravity** | $g$ | $9.81$ | $\text{m/s}^2$ | Gravitational acceleration |
| **Torque Bound** | $\tau_{\max}$ | $20.0$ | $\text{N}\cdot\text{m}$ | Actuator saturation limit |

### Nonlinear Equations of Motion

Derived via the Euler-Lagrange equations $\frac{d}{dt}\frac{\partial L}{\partial \dot{q}} - \frac{\partial L}{\partial q} = B\tau$:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} = \tau_g(q) + B\tau$$

where:
$$M(q) = \begin{bmatrix} 2.663 + 2\cos(q_2) & 1.33 + \cos(q_2) \\ 1.33 + \cos(q_2) & 1.33 \end{bmatrix}$$

$$C(q, \dot{q}) = \sin(q_2) \begin{bmatrix} -2\dot{q}_2 & -\dot{q}_2 \\ \dot{q}_1 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

$$\tau_g(q) = -9.81 \begin{bmatrix} 1.5\sin(q_1) + \sin(q_1 + q_2) \\ \sin(q_1 + q_2) \end{bmatrix}$$

**Global Invertibility:**
$$\det M(q) = 1.7729 - \cos^2(q_2) \in [0.7729, 1.7729] > 0 \quad \forall q_2 \in \mathbb{R}$$
Because $\det M(q) \ge 0.7729 > 0$ everywhere, dynamic singularities never occur, allowing state acceleration to be solved in closed form:
$$\ddot{q} = M(q)^{-1}\left(\tau_g(q) + B\tau - C(q,\dot{q})\dot{q}\right)$$

---

## 2. Phase-by-Phase Technical Breakdown

```
       Phase 1                      Phase 2                      Phase 3
  ┌───────────────┐           ┌─────────────────┐          ┌─────────────────┐
  │  Euler-Lagrange│           │ Stability Proof │          │ LQR State-Space │
  │  Derivation & ├──────────►│ SISO Limitation ├─────────►│ Balance Control │
  │ Linearization │           │  Root Locus Trap│          │ & Basin of Attr.│
  └───────────────┘           └─────────────────┘          └────────┬────────┘
                                                                    │
       Phase 5                      Phase 4                         │
  ┌───────────────┐           ┌─────────────────┐                   │
  │ SAC Deep RL   │           │ Nonlinear Spong │                   │
  │ Swing-Up vs.  │◄─────────┤ Energy Swing-Up ├───────────────────┘
  │ Energy Laws   │           │ & Riccati Catch │
  └───────────────┘           └─────────────────┘
```

### Phase 1: Analytical Mechanics, Equilibria & Linearization
* **Zero-Torque Equilibria**: Setting $\dot{q} = 0, \ddot{q} = 0, \tau = 0$ yields exactly four distinct physical equilibria in $[-\pi, \pi) \times [-\pi, \pi)$:
  1. $E_1(0, 0)$: Hanging straight down (Center-Center).
  2. $E_2(0, \pi)$: Link 1 down, Link 2 folded up (Saddle-Center).
  3. $E_3(\pi, 0)$: Fully upright inverted equilibrium (Saddle-Saddle, target).
  4. $E_4(\pi, \pi)$: Link 1 upright, Link 2 folded down (Saddle-Center).
* **Linear State-Space Formulation**:
  $$\dot{\tilde{x}} = A\tilde{x} + B_\ell \tilde{\tau}, \quad A = \begin{bmatrix} 0_{2\times2} & I_{2\times2} \\ P & 0_{2\times2} \end{bmatrix}, \quad B_\ell = \begin{bmatrix} 0_{2\times1} \\ r \end{bmatrix}$$
  where $P = M(q^*)^{-1} \left.\frac{\partial \tau_g}{\partial q}\right|_{q^*}$ and $r = M(q^*)^{-1} B$.
* **Symmetry Relations**: $P(E_3) = -P(E_1)$, $P(E_4) = -P(E_2)$, with identical input coupling vectors $r(E_3) = r(E_1)$ and $r(E_4) = r(E_2)$.
* **Deliverable Code**: See [`phase1/phase 1 deliverable/code/`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase1/phase%201%20deliverable/code/).

---

### Phase 2: Open-Loop Stability & Single-Angle Control Failure Proof
* **Modal Analysis**:
  * $E_1$: 4 imaginary eigenvalues ($\pm j2.2400, \pm j6.1011\,\text{rad/s}$), neutrally stable undamped center.
  * $E_3$: 2 pairs of real symmetric eigenvalues ($\pm 2.2400, \pm 6.1011\,\text{rad/s}$), exponentially unstable with dominant divergence rate $e^{+6.1011 t}$.
* **The Root Locus Trap ($q_1$ SISO Feedback)**:
  * Open-loop unstable pole at $p = +2.2400\,\text{rad/s}$.
  * Right-Half-Plane (non-minimum phase) transmission zero at $z = +2.0519\,\text{rad/s}$.
  * Because $z < p$ on the positive real axis, standard root locus mechanics dictate that for any gain direction, the root locus branch originating at $p = +2.2400$ is drawn directly into $z = +2.0519$. No linear SISO controller (regardless of poles/zeros or order) can stabilize the plant using shoulder angle feedback alone.
* **Poisson Sensitivity Peaking**:
  * By the Poisson-Bode sensitivity integral constraint:
    $$M_s = \|S(j\omega)\|_\infty \ge \prod_{i=1}^{2} \left|\frac{z + p_i}{z - p_i}\right| \ge 45.9 \quad (33.2\,\text{dB})$$
  * Any attempt to push closed-loop poles into the LHP induces severe sensitivity peaking ($M_s \ge 45.9$), causing immediate actuator saturation and divergent trajectories.
* **Parity Interlacing Property ($q_2$ SISO Feedback)**:
  * For $G_2(s)$, an odd number of real poles lies strictly to the right of the real zero, requiring an unstable controller (unviable under actuator saturation).
* **Interactive Dashboard**: PyBullet real-time interactive physics with draggable links, live s-plane pole/zero plots, and saturation indicators in [`phase2/acrobot_phase2_sim.py`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase2/acrobot_phase2_sim.py).

---

### Phase 3: Multivariable LQR Balancing & Basin of Attraction
* **Full-State Feedback Design**:
  * Synthesized at upright $E_3(\pi, 0, 0, 0)$ via the Algebraic Riccati Equation:
    $$A^T S + S A - S B_\ell R^{-1} B_\ell^T S + Q = 0$$
    with $Q = \text{diag}(1, 1, 1, 1)$ and $R = 0.03$.
  * Resulting optimal gain: $K = R^{-1} B_\ell^T S$.
  * Actuator saturation control law:
    $$\tau(t) = \text{sat}_{[-20, +20]}\left(-K(x(t) - x^*)\right)$$
* **Region of Attraction (ROA) Mapping**:
  * Exhaustively simulated across 2D/4D perturbation grids from rest.
  * Standalone coordinate limits: stabilizes up to $\pm 6^\circ$ in $q_1$ and $\pm 13^\circ$ in $q_2$.
  * **Center-of-Mass Invariant Ridge**: When perturbed along the manifold where the total center of mass stays vertically aligned above the shoulder pivot ($\Delta q_2 = -2.5 \Delta q_1$), the controller recovers from massive initial displacements exceeding $\pm 43^\circ$.
* **Deliverable Code**: See [`phase3/acrobot_phase3_sim.py`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase3/acrobot_phase3_sim.py).

---

### Phase 4: Nonlinear Hybrid Swing-Up & Catch Control
* **Collocated Partial Feedback Linearization (PFL)**:
  * Decouple the actuated elbow acceleration $\ddot{q}_2 = v$:
    $$\tau = \left(M_{22} - \frac{M_{12}^2}{M_{11}}\right)v + \tau_{g2} - C_2 \dot{q} + \frac{M_{12}}{M_{11}}\left(C_1 \dot{q} - \tau_{g1}\right)$$
* **Spong Energy-Pumping Control**:
  * Target mechanical energy at upright posture: $E_{\text{top}} = +24.525\,\text{J}$.
  * Synthetic acceleration law:
    $$v = -k_1 q_2 - k_2 \dot{q}_2 + k_3 (E(q, \dot{q}) - E_{\text{top}})\dot{q}_1 \cos(q_1)$$
  * Optimal tuned parameters: $k_1 = 20, k_2 = 2, k_3 = 3$. Stiffer gains (e.g., textbook $k_1 = 50$) trigger severe torque saturation (38%–45% duty cycle), causing trajectory divergence.
* **Lyapunov-Riccati Catch Criterion**:
  * Switches smoothly from Spong energy pumping to LQR state feedback when inside the invariant basin:
    $$\tilde{x}^T S \tilde{x} < \epsilon_{\text{catch}} = 5.0$$
  * Automatic retry fallback: if an external shock or failed catch drives $\tilde{x}^T S \tilde{x} \ge 5$, energy swing-up instantly resumes.
  * Performance: **39 out of 40** random hanging starts caught and balanced within $\sim 4\,\text{s}$.
* **Deliverable Code**: See [`phase4/acrobot_phase4_sim.py`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase4/acrobot_phase4_sim.py) and [`phase4/run_all.sh`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase4/run_all.sh).

---

### Phase 5: Deep Reinforcement Learning (SAC) vs. Energy Swing-Up
* **Formulation & Architecture**:
  * Custom Gymnasium environment ([`phase5_rl/acrobot_env.py`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase5_rl/acrobot_env.py)).
  * Algorithm: Soft Actor-Critic (SAC) with continuous torque actions $a \in [-1, 1] \implies \tau = 20 a\,\text{N}\cdot\text{m}$, 20 ms time steps, 2-layer 256-unit MLP.
  * Reward shaping: Energy deficit squared penalty, upright posture bonus, torque penalty, and $+100$ catch reward.
  * Hand-over: Both RL policies and the Spong energy controller switch to the exact same Phase 3 LQR upon reaching $\tilde{x}^T S \tilde{x} < 5$.
* **Robust Policy Innovation**:
  * Standard SAC trained solely on 1 kg links memorizes timing and collapses when link masses change (only 15/40 caught at 1.2 kg).
  * The **Robust SAC Policy** incorporates:
    1. *Domain Randomization*: Random link masses $m_1, m_2 \in [0.7, 1.3]\,\text{kg}$ sampled on every swing.
    2. *Energy Observation Hint*: Injects normalized missing energy $(E_{\text{top}} - E)/E_{\text{top}}$ into the actor-critic observation vector.

#### Benchmark Comparison: 40 Random Starts (30 s Budget)

| Performance Metric | Spong Energy Law | SAC (Nominal 1 kg) | SAC (Robust Random Masses) |
| :--- | :---: | :---: | :---: |
| **Held Rate (Nominal 1.0 kg)** | 35 / 40 (87.5%) | **40 / 40 (100%)** | **40 / 40 (100%)** |
| **Median Time to Catch** | 3.98 s | **1.71 s** | **1.71 s** |
| **Time at Torque Limit (Saturation)** | 8.8% | 0.1% | **0.0%** |
| **Median Elbow Mechanical Work** | 245 J | 144 J | **130 J** |
| **Held @ 0.6 kg (-40% Mass)** | 39 / 40 | 2 / 40 | **40 / 40 (100%)** |
| **Held @ 0.8 kg (-20% Mass)** | 37 / 40 | 38 / 40 | **40 / 40 (100%)** |
| **Held @ 1.2 kg (+20% Mass)** | 35 / 40 | 15 / 40 | **40 / 40 (100%)** |
| **Held @ 1.4 kg (+40% Mass)** | **33 / 40** | 3 / 40 | 2 / 40 |

> **Key Control Takeaway:** The robust RL agent learns a faster, smoother, torque-efficient trajectory (0.0% saturation vs 8.8%) across its training distribution. However, physics-based energy pumping extrapolates significantly further into extreme unmodelled regimes (+40% mass) without retraining.

---

## 3. Simulink Model Suite

Four fully configured Simulink models located in [`simulink/`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/simulink/):

| Model File | Phase | Architecture & Features |
| :--- | :---: | :--- |
| `Acrobot_OpenLoop.slx` | 2 | Zero-torque free dynamics alongside linear state-space comparison block $(A, B)$ |
| `Acrobot_SingleAnglePD.slx` | 2 | Single-angle feedback testing with saturation and pole/zero verification |
| `Acrobot_LQR.slx` | 3 | Full-state feedback $\tau = \text{sat}_{20}(-K\tilde{x})$ tested across all 4 equilibria |
| `Acrobot_SwingUp.slx` | 4 | Hybrid Spong energy pumping $\to$ Lyapunov-Riccati LQR catch logic |

### Automated Verification (`run_simulink_models.m`)

Run from MATLAB to execute all 15 automated validation checks:
* Open-loop divergence rates at $E_2, E_3, E_4$ match theoretical eigenvalues to $< 0.25\%$ error.
* $E_1$ bounded oscillation verification.
* Single-angle PD instability reproduction.
* LQR stabilization and catch region convergence across initial condition grids.

```matlab
cd simulink
build_simulink_models    % Rebuilds and exports .slx models
run_simulink_models      % Executes test suite (15/15 PASS)
```

---

## 4. Interactive Web Explainer & 2K Animations

### Interactive Web Explainer
An interactive browser-based technical document is included in [`explainer/index.html`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/explainer/index.html). It features:
* Interactive system visualization with coordinate definitions and link colors.
* Step-by-step walkthrough of the Root Locus trap and Poisson peaking curves.
* 2D and 4D Region of Attraction (ROA) contour plots and CM-over-pivot ridge analysis.
* Energy pumping phase portraits and hybrid handover timelines.

Open directly in any modern web browser:
```bash
open explainer/index.html
```

### High-Resolution 2K Manim Animations
Five cinematic 2K animations rendered via Manim Community in [`final_submission/videos/`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/final_submission/videos/):
* `Phase1_Model_and_Equilibria.mp4`: Coordinate frames, Lagrangian formulation, equilibrium manifold.
* `Phase2_Stability_2K.mp4`: Eigenmode decomposition, phase portraits, SISO Root Locus trap proof.
* `Phase3_Control_2K.mp4`: LQR state feedback, torque saturation clipping, 4D basin of attraction.
* `Phase4_SwingUp_2K.mp4`: PFL energy pumping, limit cycles, Lyapunov-Riccati catch.
* `Phase5_RL_vs_Energy_SwingUp_2K.mp4`: Side-by-side comparison of SAC neural swing-up vs. Spong energy law under nominal and mass-perturbed conditions.

---

## 5. Repository Structure

```
acrobot-control/
├── README.md                                  <- This comprehensive document
├── .gitignore                                 <- Git ignore rules
│
├── explainer/                                 <- Interactive Web Visualizer
│   ├── index.html                             <- Full interactive web explainer
│   └── images/                                <- High-resolution figures & plots
│
├── phase1/                                    <- Phase 1: Modelling & Linearization
│   ├── handouts/                              <- Project problem specification
│   └── phase 1 deliverable/                   <- MATLAB source code, ODE45 sim & README
│
├── phase2/                                    <- Phase 2: Stability Analysis & SISO Proof
│   ├── acrobot_phase2_sim.py                  <- PyBullet interactive GUI & s-plane visualizer
│   ├── plots/                                 <- Generated simulation plots
│   └── phase 2 deliverable/                   <- MATLAB & Simulink models, report figures
│
├── phase3/                                    <- Phase 3: LQR Balancing & Basin of Attraction
│   ├── acrobot_phase3_sim.py                  <- PyBullet LQR balancing GUI
│   ├── acrobot_phase3_design.py               <- Riccati solver & ROA computation
│   └── phase 3 deliverable/                   <- MATLAB ROA scripts & Simulink models
│
├── phase4/                                    <- Phase 4: Energy Swing-Up & Hybrid Catch
│   ├── acrobot_phase4_sim.py                  <- Real-time swing-up & catch GUI
│   ├── acrobot_phase4_design.py               <- Parameter sweep & tuning scripts
│   ├── run_all.sh                             <- Full pipeline execution script
│   └── phase 4 deliverable/                   <- Deliverable code, results & figures
│
├── phase5_rl/                                 <- Phase 5: Deep Reinforcement Learning
│   ├── acrobot_env.py                         <- Gymnasium Acrobot environment
│   ├── train.py                               <- SAC training script (nominal & robust)
│   ├── evaluate.py                            <- Benchmark evaluation suite (40 starts x 5 masses)
│   ├── plot_training.py                       <- Generates training curve figures
│   ├── run_pipeline.sh                        <- End-to-end training & evaluation pipeline
│   ├── policies/                              <- Saved trained PyTorch/SB3 models (.zip)
│   └── manim-sim/                             <- Manim script for RL vs Energy video
│
├── simulink/                                  <- Complete Simulink Suite
│   ├── Acrobot_OpenLoop.slx                   <- Open-loop free response
│   ├── Acrobot_SingleAnglePD.slx              <- Single-angle PD failure test
│   ├── Acrobot_LQR.slx                        <- Full-state LQR balancing
│   ├── Acrobot_SwingUp.slx                    <- Hybrid swing-up & catch
│   ├── build_simulink_models.m                <- Model builder script
│   ├── run_simulink_models.m                  <- 15-test automated validation runner
│   └── results/                               <- Exported CSV trajectories & comparison JSON
│
└── final_submission/                          <- Consolidated Final Submission
    ├── README.md                              <- Submission overview
    ├── reports/                               <- Final compiled PDF reports (Full + Phases 1-5)
    ├── videos/                                <- Final 2K Manim animation MP4s
    ├── report_source/                         <- LaTeX sources, figures, and build scripts
    └── code/                                  <- Consolidated code organized by phase
```

---

## 6. Getting Started & Reproduction

### Prerequisites

* **Python Environment**: Python 3.10 to 3.12 (or 3.14 via source build)
* **Python Libraries**: `numpy`, `scipy`, `matplotlib`, `pybullet`, `gymnasium`, `stable-baselines3`, `torch`
* **MATLAB**: R2022b or later (R2025b recommended for Simulink models) with:
  * Control System Toolbox
  * Symbolic Math Toolbox
  * Simulink (MATLAB Function blocks)

### Python Setup

Create and activate a virtual environment in `phase2/`:

```bash
cd phase2
python3 -m venv .venv
source .venv/bin/activate
pip install numpy scipy matplotlib pybullet gymnasium stable-baselines3 torch
```

> **macOS + Python 3.14 Build Note:** If building PyBullet on Python 3.14 on macOS, patch line 128 of `zutil.h` as described in [`phase2/README.md`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/phase2/README.md).

### Running Interactive PyBullet Dashboards

```bash
# Phase 2: Open-loop dynamics and live s-plane pole/zero updates
cd phase2
.venv/bin/python acrobot_phase2_sim.py

# Phase 3: LQR upright balancing from perturbations
cd ../phase3
../phase2/.venv/bin/python acrobot_phase3_sim.py

# Phase 4: Full hybrid swing-up from hanging and LQR catch
cd ../phase4
../phase2/.venv/bin/python acrobot_phase4_sim.py
```

### Running the Reinforcement Learning Pipeline (Phase 5)

```bash
cd phase5_rl
# Run complete evaluation of trained policies against energy controller:
../phase2/.venv/bin/python evaluate.py

# Plot training learning curves:
../phase2/.venv/bin/python plot_training.py

# Optional: retrain robust policy from scratch (2M steps):
../phase2/.venv/bin/python train.py 2000000 --robust
```

### Running MATLAB & Simulink Scripts

From the MATLAB command window:

```matlab
% Phase 1: Verify model, calculate equilibria, analytical Jacobians
cd phase1/phase 1 deliverable/code
Acrobot_Phase1_Main

% Simulink Suite: Build and run automated test suite
cd ../../../simulink
build_simulink_models
run_simulink_models
```

---

## 7. Comprehensive Quantitative Results

### Equilibrium Poles, Zeros & Stability Classification

| Equilibrium | State $(q_1, q_2)$ | Open-Loop Poles ($s$) | $G_1(s)$ Zeros $(\tau \to q_1)$ | $G_2(s)$ Zeros $(\tau \to q_2)$ | Classification |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **$E_1$ (Hanging)** | $(0, 0)$ | $\pm j2.2400, \pm j6.1011$ | $\pm j2.0519$ | $\pm j2.2934$ | Undamped Center |
| **$E_2$ (Down-Up)** | $(0, \pi)$ | $\pm 3.1775, \pm j4.3010$ | $\pm 5.4523$ | $\pm j2.7200$ | Saddle-Center |
| **$E_3$ (Upright)** | $(\pi, 0)$ | $\pm 2.2400, \pm 6.1011$ | $\pm 2.0519$ | $\pm 2.2934$ | Saddle-Saddle |
| **$E_4$ (Up-Down)** | $(\pi, \pi)$ | $\pm 4.3010, \pm j3.1775$ | $\pm j5.4523$ | $\pm 2.7200$ | Saddle-Center |

### Linear Simulation Accuracy Near Equilibria

* **$E_1$ hanging** ($\Delta q(0) = [0.1, -0.1]\,\text{rad}$): Maximum state discrepancy between nonlinear dynamics and linear state-space approximation is $\le 1.0^\circ$ in $q_1$ and $\le 2.0^\circ$ in $q_2$ across 8 seconds.
* **$E_3$ upright** ($\Delta q_1(0) = 10^{-3}\,\text{rad}$): Relative error is only $0.02\%$ when $|\Delta q_1| = 1^\circ$ ($t \approx 0.80\,\text{s}$) and $0.37^\circ$ at $|\Delta q_1| = 10^\circ$ ($t \approx 1.20\,\text{s}$).
* **Symplectic Energy Conservation**: Unforced drift over 20 seconds is $< 10^{-8}$ relative error.

---

## 8. Course Context & Deliverables

* **Project**: Systems Thinking & Control Course Project
* **Team Members**:
  * Bhargav Vemparala (2025102061)
  * Sreekar (2025102074)
  * Kaushik (2025102071)
  * Divijat (2025102040)
* **Submitted Reports** (located in [`final_submission/reports/`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/final_submission/reports/)):
  * [`Acrobot_Full_Report.pdf`](file:///Users/bhargav_vemparala/Documents/Projects/projects/ECE_Projects/acrobot-control/final_submission/reports/Acrobot_Full_Report.pdf) (Complete consolidated report across all topics & Simulink models)
  * `Phase1_Report.pdf` & `Phase1_Report_IEEE.pdf`
  * `Phase2_Report.pdf`
  * `Phase3_Report.pdf`
  * `Phase4_Report.pdf`
  * `Phase5_Report.pdf`

---

## 9. References

1. M. W. Spong, "The swing up control problem for the Acrobot," *IEEE Control Systems Magazine*, vol. 15, no. 1, pp. 49–55, 1995.
2. R. Tedrake, *Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation*, MIT, 2024. [Online Notes](https://underactuated.csail.mit.edu).
3. S. Skogestad and I. Postlethwaite, *Multivariable Feedback Control: Analysis and Design*, 2nd ed., Wiley, 2005.
4. T. Haarnoja, A. Zhou, P. Abbeel, and S. Levine, "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor," *ICML*, 2018.
5. D. C. Youla, J. J. Bongiorno, and C. N. Lu, "Single-loop feedback-stabilization of linear multivariable dynamical plants," *Automatica*, vol. 10, no. 2, pp. 159–173, 1974.
