# Acrobot: Modelling, Stability Analysis, and Underactuated Control

[![MATLAB](https://img.shields.io/badge/Platform-MATLAB%20%7C%20Simulink-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Course](https://img.shields.io/badge/Course-Systems%20Thinking%20%26%20Control-blue.svg)](#)
[![Status](https://img.shields.io/badge/Phases%201--4-Validated-brightgreen.svg)](#)
[![License](https://img.shields.io/badge/License-Academic%20Use-lightgrey.svg)](#)

A comprehensive systems and control engineering project on the **Acrobot**—an underactuated planar two-link inverted pendulum with an actuator at the elbow and a completely passive shoulder.

This repository covers the complete analytical and numerical workflow across all four project phases: from Lagrangian formulation and equilibrium bifurcation manifolds to multivariable stability analysis, Poisson sensitivity proofs, full-state LQR balancing with torque saturation, energy-based Spong swing-up, and hybrid switching architecture.

---

## 1. System Overview & Physical Parameters

The Acrobot has two degrees of freedom ($q_1, q_2$) but only one control input ($\tau$), entering at the elbow. The shoulder joint is passive; motion at the shoulder is achieved purely via nonlinear dynamical coupling.

* **$q_1$**: Angle of Link 1 measured counter-clockwise from the downward vertical ($q_1 = 0$ is hanging straight down).
* **$q_2$**: Relative elbow angle measured with respect to Link 1 ($q_2 = 0$ is Link 2 aligned with Link 1).
* **$\tau$**: Actuation torque applied at the elbow, subject to physical saturation $|\tau| \le \tau_{\max} = 20\text{ N}\cdot\text{m}$.

```
      passive shoulder (pivot)
              (O)
              /  \
             /    \
            /  q1  \
           /        \
    Link 1 [m1, l1]  \  q1 = 0 (straight down)
         /
       (O)  actuated elbow (torque: tau)
        \
         \  q2 (relative)
          \
    Link 2 [m2, l2]
```

### Physical Parameters (Uniform Rod Benchmark)

| Parameter | Symbol | Nominal Value | Unit | Definition |
| :--- | :--- | :--- | :--- | :--- |
| **Link Masses** | $m_1, m_2$ | $1.0, 1.0$ | $\text{kg}$ | Mass of each link |
| **Link Lengths** | $l_1, l_2$ | $1.0, 2.0$ | $\text{m}$ | Kinematic link lengths |
| **COM Locations** | $l_{c1}, l_{c2}$ | $0.5, 1.0$ | $\text{m}$ | Midpoint centers of mass ($l_i / 2$) |
| **COM Inertias** | $I_1, I_2$ | $0.083, 0.330$ | $\text{kg}\cdot\text{m}^2$ | Moment of inertia about COM |
| **Gravity** | $g$ | $9.81$ | $\text{m/s}^2$ | Gravitational acceleration |
| **Torque Limit** | $\tau_{\max}$ | $20.0$ | $\text{N}\cdot\text{m}$ | Actuator saturation boundary |

---

## 2. Mathematical Dynamics

### 2.1 Euler-Lagrange Formulation

The equations of motion are given by:
$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} = \tau_g(q) + B\tau$$

where state vector $x = [q_1, q_2, \dot{q}_1, \dot{q}_2]^T \in \mathbb{R}^4$, input matrix $B = [0, 1]^T$, and:

$$M(q) = \begin{bmatrix} 2.663 + 2\cos(q_2) & 1.33 + \cos(q_2) \\ 1.33 + \cos(q_2) & 1.33 \end{bmatrix}$$

$$C(q, \dot{q}) = \sin(q_2) \begin{bmatrix} -2\dot{q}_2 & -\dot{q}_2 \\ \dot{q}_1 & 0 \end{bmatrix}$$

$$\tau_g(q) = -9.81 \begin{bmatrix} 1.5\sin(q_1) + \sin(q_1 + q_2) \\ \sin(q_1 + q_2) \end{bmatrix}$$

$$\det M(q) = 1.7729 - \cos^2(q_2) \in [0.7729, 1.7729] > 0 \quad \forall q_2$$

Because $\det M(q) > 0$ globally, the inertia matrix is invertible for all configurations:
$$\ddot{q} = M(q)^{-1}\left(\tau_g(q) + B\tau - C(q,\dot{q})\dot{q}\right)$$

---

## 3. Project Architecture by Phase

```
.
├── README.md                              <- Project overview & documentation
├── Acrobot_ML_Implementation_Plan.md      <- Full master blueprint & viva prep
├── reports/
│   └── Phase1_Acrobot_Report_IEEE.pdf     <- Phase 1 IEEE-format submitted report
│
├── model/                                 <- Shared mathematical & physical model
│   ├── acrobot_params.m                   <- System parameter definition script
│   ├── acrobot_dynamics.m                 <- Continuous state derivative for ODE45
│   ├── acrobot_energy.m                   <- Total mechanical energy E(q, dq)
│   └── acrobot_urdf.urdf                  <- URDF model for 3D physics engines
│
├── phase1_modelling/                      <- Phase 1: Equilibria & Jacobians
│   ├── find_equilibria.m                  <- Analytical & numerical equilibrium solver
│   ├── linearize_acrobot.m                <- Symbolic & numerical Jacobian computation
│   └── plot_equilibrium_manifold.m       <- Generates 1D equilibrium fold curves
│
├── phase2_stability/                      <- Phase 2: Open-Loop Stability & Single-Angle Proof
│   ├── modal_analysis.m                   <- Eigenvalues, transfer function poles/zeros
│   ├── root_locus_proof.m                 <- Root locus trap analysis (G1 and G2)
│   ├── sensitivity_peaking.m              <- Poisson integral bound calculation (Ms >= 45.9)
│   └── perturbation_sims.m                <- Linear vs nonlinear ODE45 verification
│
├── phase3_balancing/                      <- Phase 3: LQR Balancing & Basin of Attraction
│   ├── design_lqr.m                       <- ARE solution and Bryson weighting tuning
│   ├── basin_of_attraction.m              <- 2D Monte Carlo recovery grid (with |tau| <= 20)
│   ├── nonlinear_recovery_sim.m           <- Time-domain state trajectories & saturation
│   └── simscape_acrobot.slx               <- Simulink/Simscape model for 3D animation
│
├── phase4_swingup/                        <- Phase 4: Energy Pumping & Hybrid Catch
│   ├── partial_feedback_lin.m             <- Collocated partial feedback linearization
│   ├── spong_pumping.m                    <- Energy-based Spong pumping law
│   ├── hybrid_controller.m                <- Riccati handover condition (x'*S*x < eps)
│   ├── sensitivity_study.m                <- 5-point gain study (k3 = 0.5 to 4.0)
│   └── acrobot_hybrid_sim.slx             <- Full hybrid Simulink/Stateflow model
│
└── presentation/                          <- Viva presentation materials
    ├── Acrobot_Viva_Slides.pdf            <- Technical defense slide deck
    └── manim/                             <- Python Manim visualization scripts
        ├── scene_manifold_fold.py
        └── scene_root_locus_trap.py
```

---

## 4. Key Analytical Results

### 4.1 Equilibrium Sets (Unforced & Forced Families)

Setting $\dot{q} = \ddot{q} = 0$ yields the equilibrium condition:
$$1.5\sin(q_1^*) + \sin(q_1^* + q_2^*) = 0$$
$$\tau^* = 9.81\sin(q_1^* + q_2^*) = -14.715\sin(q_1^*)\text{ N}\cdot\text{m}$$

* **Unforced Equilibria ($\tau^* = 0$):**
  * $E_1(0, 0)$: Down-Down (Stable Center-Center, $U = -24.525\text{ J}$)
  * $E_2(0, \pi)$: Down-Up (Saddle-Center, 1 RHP pole, $U = -4.905\text{ J}$)
  * $E_3(\pi, 0)$: Up-Up (Unstable Saddle-Saddle, 2 RHP poles, $U = +24.525\text{ J}$)
  * $E_4(\pi, \pi)$: Up-Down (Saddle-Center, 1 RHP pole, $U = +4.905\text{ J}$)
* **Forced Equilibrium Manifolds ($\tau^* \neq 0$):**
  * Real solutions exist if and only if $|\sin(q_1^*)| \le \frac{2}{3}$.
  * Maximum required holding torque occurs at fold boundary $\sin(q_1^*) = \pm \frac{2}{3}$:
    $$|\tau^*|_{\max} = 9.81\text{ N}\cdot\text{m} < 20.0\text{ N}\cdot\text{m}$$
  * **Result:** Every physical equilibrium of the Acrobot can be held indefinitely without exceeding the motor torque limit.

### 4.2 Linearized Models at Unforced Equilibria

$$\tilde{\dot{x}} = A\tilde{x} + B_\ell\tilde{\tau}, \quad A = \begin{bmatrix} 0 & I_2 \\ P & 0 \end{bmatrix}, \quad B_\ell = \begin{bmatrix} 0 \\ r \end{bmatrix}$$

$$P = M(q^*)^{-1} K(q^*), \quad r = M(q^*)^{-1} B$$

| Equilibrium | Characteristic Denominator $\Delta(s)$ | Poles | $G_1(s)$ Zeros $(\tau \to q_1)$ | $G_2(s)$ Zeros $(\tau \to q_2)$ | Classification |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **$E_1(0,0)$** | $s^4 + 42.2410s^2 + 186.7719$ | $\pm j2.2400, \pm j6.1011$ | $\pm j2.0519$ | $\pm j2.2934$ | Center-Center (0 RHP) |
| **$E_2(0,\pi)$** | $s^4 + 8.4025s^2 - 186.7719$ | $\pm 3.1775, \pm j4.3010$ | $\pm 5.4523$ | $\pm j2.7200$ | Saddle-Center (1 RHP) |
| **$E_3(\pi,0)$** | $s^4 - 42.2410s^2 + 186.7719$ | $\pm 2.2400, \pm 6.1011$ | $\pm 2.0519$ | $\pm 2.2934$ | Saddle-Saddle (2 RHP) |
| **$E_4(\pi,\pi)$** | $s^4 - 8.4025s^2 - 186.7719$ | $\pm 4.3010, \pm j3.1775$ | $\pm j5.4523$ | $\pm 2.7200$ | Saddle-Center (1 RHP) |

### 4.3 Fundamental Limitation: Failure of Single-Angle Feedback at Upright $E_3$

Can feedback from a single measured angle stabilize the upright Acrobot? **No.**

1. **The Root Locus Trap ($q_1$ feedback):**
   * Unstable pole: $p = +2.2400\text{ rad/s}$
   * Non-minimum-phase (RHP) transmission zero: $z = +2.0519\text{ rad/s}$
   * Because $z < p$, the root locus branch originating at $+2.2400$ is drawn directly into the zero at $+2.0519$ along the positive real axis for any gain direction. No SISO controller can push both branches into the Left Half-Plane.
2. **Poisson Integral & Sensitivity Peaking:**
   By Bode's sensitivity integral:
   $$M_s = \|S(j\omega)\|_\infty \ge \prod_{i=1}^{2} \frac{|z + p_i|}{|z - p_i|} \ge 45.9 \quad (33.2\text{ dB})$$
   Attempting SISO stabilization results in severe gain/phase margin erosion and immediate actuator saturation.
3. **Parity Interlacing Property ($q_2$ feedback):**
   For $G_2(s)$, an odd number of real poles lies strictly to the right of the real zero, demanding an unstable controller (which is structurally fragile and practically non-viable under torque limits).

*Balancing at the upright posture requires multi-variable Full-State Feedback (LQR).*

---

## 5. Control Design

### 5.1 LQR Balancing Controller (Phase 3)

Linear Quadratic Regulator synthesized at upright posture $E_3(\pi, 0, 0, 0)$:
* State error: $\tilde{x} = [q_1 - \pi, q_2, \dot{q}_1, \dot{q}_2]^T$
* Algebraic Riccati Equation: $A^T S + S A - S B_\ell R^{-1} B_\ell^T S + Q = 0$
* Optimal Gain: $K = R^{-1} B_\ell^T S = [k_1, k_2, k_3, k_4]$
* Saturation Control Law:
  $$\tau(t) = \text{clip}\left(-K\tilde{x}(t),\; -20.0,\; +20.0\right)$$

### 5.2 Energy-Based Swing-Up & Hybrid Switching (Phase 4)

1. **Collocated Partial Feedback Linearization:**
   Decouple elbow acceleration $\ddot{q}_2 = v$:
   $$\tau = \left(M_{22} - \frac{M_{12}^2}{M_{11}}\right)v + \tau_{g2} - C_2\dot{q} + \frac{M_{12}}{M_{11}}(C_1\dot{q} - \tau_{g1})$$
2. **Spong Energy Pumping Law:**
   $$v = -k_p q_2 - k_d \dot{q}_2 + k_3 (E_{\text{target}} - E(q, \dot{q}))\dot{q}_1 \cos(q_1)$$
3. **Lyapunov-Riccati Catch Criterion:**
   Switch from Spong swing-up to LQR balancing when:
   $$\tilde{x}^T S \tilde{x} < \epsilon_{\text{catch}}$$
   where $S$ is the infinite-horizon Riccati solution, guaranteeing the state has entered the verifiable basin of attraction.

---

## 6. Getting Started

### Prerequisites
* MATLAB R2022b or later
* Control System Toolbox
* Symbolic Math Toolbox
* Simulink & Simscape Multibody (for 3D mechanical simulations)

### Quick Run Guide

1. **Clone the repository:**
   ```bash
   git clone https://github.com/<your-username>/acrobot-control.git
   cd acrobot-control
   ```

2. **Verify Phase 1 Model & Linearization:**
   ```matlab
   % Run inside MATLAB command window
   cd phase1_modelling
   find_equilibria
   linearize_acrobot
   ```

3. **Plot Root Locus & Frequency Response (Phase 2):**
   ```matlab
   cd ../phase2_stability
   modal_analysis
   root_locus_proof
   ```

4. **Compute LQR & Basin of Attraction (Phase 3):**
   ```matlab
   cd ../phase3_balancing
   design_lqr
   basin_of_attraction
   ```

5. **Simulate Full Hybrid Swing-Up & Catch (Phase 4):**
   ```matlab
   cd ../phase4_swingup
   hybrid_controller
   ```

---

## 7. References

1. M. W. Spong, "The swing up control problem for the Acrobot," *IEEE Control Systems Magazine*, vol. 15, no. 1, pp. 49–55, 1995.
2. R. Tedrake, *Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation*, MIT, 2024. [Online Course Notes](https://underactuated.csail.mit.edu).
3. S. Skogestad and I. Postlethwaite, *Multivariable Feedback Control: Analysis and Design*, 2nd ed., Wiley, 2005.
4. D. C. Youla, J. J. Bongiorno, and C. N. Lu, "Single-loop feedback-stabilization of linear multivariable dynamical plants," *Automatica*, vol. 10, no. 2, pp. 159–173, 1974.