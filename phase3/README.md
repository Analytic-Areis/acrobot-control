# Acrobot – Phase 3: Control around the equilibria

Controllers that stabilize the upright equilibrium E3 and the other three equilibria, with |τ| ≤ 20 N·m, and how far from each equilibrium they still work.

| Equilibrium | Controller | Recovered from rest | Along each axis alone |
|---|---|---|---|
| **E3 upright** | full-state LQR, Q = I, R = 0.03 | 26% of ±40° | 6° (q1), 13° (q2) |
| **E2** down-up | full-state LQR, Q = I, R = 0.1 | 99.7% of ±180° | almost anywhere |
| **E4** up-down | full-state LQR, Q = I, R = 0.1 | 20.5% of ±180° | 35° (q1), 55° (q2) |
| **E1** down-down | PD on the elbow angle only, Kp = −5, Kd = 0.4 | 78% of ±180° | 40° (q1), 115° (q2) |

At E3, along the line where the centre of mass stays over the pivot (Δq2 = −2.5 Δq1), the region reaches at least 43° (the edge of the grid that was mapped).

## 1. Setup

Phase 3 reuses the Phase 2 simulator and its Python environment, so nothing new needs installing:

```bash
cd ../phase2 && python3 -m venv .venv && .venv/bin/pip install numpy scipy matplotlib pybullet
```

On this machine `../phase2/.venv` already exists. The macOS + Python 3.14 PyBullet build note is in `../phase2/README.md`.

## 2. Run

From this `phase3` folder:

```bash
../phase2/.venv/bin/python acrobot_phase3_sim.py                              # interactive dashboard
../phase2/.venv/bin/python "phase 3 deliverable/code/make_phase3_results.py"  # all figures + numbers (~2 min)
```

MATLAB (needs the Control System Toolbox), from `phase 3 deliverable/code/matlab`:

```matlab
Acrobot_Phase3_Main
```

## 3. The dashboard

It is the Phase 2 dashboard (see `../phase2/README.md` for every control) with three new controllers:

| Controller | Law |
|---|---|
| **Full-state LQR** | τ = sat(−K x̃), K from Q = I and the chosen R at that equilibrium |
| **Pole placement** | τ = sat(−K x̃), closed-loop poles at −3, −4, −5, −6 |
| **Elbow PD (E1)** | τ = sat(−Kp q̃2 − Kd q̇2), Kp = −5, Kd = 0.4 (designed for E1) |

It opens on **E3 + Full-state LQR** from Δq1(0) = 4°.

**Things to try:**

1. **E3 + Full-state LQR:**
   - Type Δq1(0) = 5.5: held, with the torque briefly saturating.
   - Type 7.5: it falls.
   - Type Δq1(0) = −10, Δq2(0) = 25: a large offset with the centre of mass over the pivot, held easily.
2. **E3 + Pole placement** at 2°: settles faster, with less torque. Try 6°: it fails, while LQR holds it.
3. **E2 + Full-state LQR:** drag the arm almost anywhere and let go; it comes back.
4. **E1 + Elbow PD:** start at (60, −60). It settles, while Open loop swings forever. Only the elbow angle is used.
5. **Kick shoulder** at any equilibrium: E3 tolerates only about 0.2 rad/s, E2 more than 8 rad/s.

## 4. Files

| Path | What |
|---|---|
| `acrobot_phase3_sim.py` | interactive dashboard (extends `../phase2/acrobot_phase2_sim.py`) |
| `acrobot_phase3_design.py` | the chosen designs, vectorized nonlinear simulation, region-of-attraction tools |
| `phase 3 deliverable/single_column/Phase3_Acrobot_Report.pdf` | report: Phases 1, 2 and 3, each starting on its own page |
| `phase 3 deliverable/code/make_phase3_results.py` | regenerates every Phase 3 figure and number (`results.json`) |
| `phase 3 deliverable/code/matlab/` | MATLAB version: designs, `ode45` runs and maps (see its README) |
| `phase 3 deliverable/manim-sim/acrobot_phase3.py` | Phase 3 animation; rendered video: `phase 3 deliverable/Acrobot_Phase3_2K.mp4` |

**How the limits are measured:**

- **Region-of-attraction maps:** an RK4 integration of the given equations at 2 ms steps, run for every grid point at once. A start counts as recovered if it ends within 1° and 0.05 rad/s of q* (modulo 2π) after 10 s; 30 s for E1, whose controller is slower.
- **Check against PyBullet:** 100% agreement on 45 random grid points, and 96% on 45 points on the boundary of the region.
