# Acrobot Project — Phase 1 MATLAB Code

**Team:** Bhargav (2025102061), Sreekar (2025102074), Kaushik (2025102071), Divijat (2025102040)

Requires MATLAB with the Symbolic Math Toolbox and the Control System Toolbox.
Run everything from inside this `code/` folder.

## Files

| File | Purpose |
| :--- | :--- |
| `Acrobot_Phase1_Main.m` | Verifies the given model against a Lagrangian derivation, finds the zero-torque equilibria, linearizes at each one and prints P, r, the state-space model and the transfer functions τ→q₁, τ→q₂ with their poles and zeros. |
| `Acrobot_Nonlinear_Dynamics.m` | Nonlinear state equation ẋ = f(x, τ) (used by the simulation). |
| `Acrobot_Linearize.m` | Analytical linearization A = [0 I; P 0], B = [0; r] at a given equilibrium. |
| `Acrobot_Linear_Simulation.m` | Compares nonlinear and linear free responses near E₁ and E₃, checks energy conservation, and saves a comparison plot as `Fig_linear_vs_nonlinear.pdf` (numbers quoted in Sec. V-C of the report). |

## Expected output (matches the report)

`Acrobot_Phase1_Main.m`

- Model verification: `M(q)`, `C(q,dq)*dq` and `U(q)` identical → all `1`; det M = 1.7729 − cos²q₂.
- Equilibria (τ* = 0): E₁ (0,0), E₂ (0,π), E₃ (π,0), E₄ (π,π).
- P and r:

| | P | r |
| :--- | :--- | :--- |
| E₁ | [−12.6292 12.6926; 14.7488 −29.6119] | [−3.0147; 6.0332] |
| E₂ | [−12.6292 12.6926; 10.5095 4.2266] | [−0.4270; 0.8578] |
| E₃ | −P(E₁) | same as E₁ |
| E₄ | −P(E₂) | same as E₂ |

- Transfer functions (Δ(s) monic):

| | Δ(s) | G₁(s)·Δ(s) | G₂(s)·Δ(s) |
| :--- | :--- | :--- | :--- |
| E₁ | s⁴ + 42.2410 s² + 186.7719 | −3.0147 (s² + 4.2103) | 6.0332 (s² + 5.2595) |
| E₂ | s⁴ + 8.4025 s² − 186.7719 | −0.4270 (s² − 29.7273) | 0.8578 (s² + 7.3982) |
| E₃ | s⁴ − 42.2410 s² + 186.7719 | −3.0147 (s² − 4.2103) | 6.0332 (s² − 5.2595) |
| E₄ | s⁴ − 8.4025 s² − 186.7719 | −0.4270 (s² + 29.7273) | 0.8578 (s² − 7.3982) |

`Acrobot_Linear_Simulation.m` (values from an independent Python check; ode45 may differ in the last digit)

- E₁, Δq(0) = (0.1, −0.1) rad: max error ≈ 1.0° (q₁), 2.0° (q₂) over 8 s.
- E₃, Δq₁(0) = 10⁻³ rad: relative error ≈ 0.02 % when |Δq₁| = 1° (t ≈ 0.80 s); ≈ 0.37° when |Δq₁| = 10° (t ≈ 1.20 s).
- Energy drift with τ = 0 over 20 s: ~10⁻⁸ relative or smaller.
