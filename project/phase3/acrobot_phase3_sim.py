"""
Phase 3 - Control around equilibria: interactive PyBullet simulation.

Same dashboard as Phase 2 (../phase2/acrobot_phase2_sim.py) with the Phase 3
controllers added. Pick an equilibrium, an initial offset and a controller and
see whether, and how, it is brought back with |tau| <= 20 N m:

    Full-state LQR      tau = sat(-K x~), K from Q = I and the chosen R
    Pole placement      tau = sat(-K x~), poles at -3, -4, -5, -6
    Elbow PD (E1)       tau = sat(-Kp q2~ - Kd q2_dot)  (designed for E1)
    (plus the Phase 2 open loop / single-angle PD / single-angle observer + LQR)

Usage
    ../phase2/.venv/bin/python acrobot_phase3_sim.py
"""

import numpy as np

import acrobot_phase3_design as d

acro = d.acro


class Phase3Sim(acro.AcrobotSim):
    MODES = {**acro.MODES, 3: "full-state LQR", 4: "pole placement", 5: "elbow PD (E1 design)"}

    def make_controller(self, mode, A, B, joint, kp, kd):
        if mode == 3:
            return d.FullStateFeedback(d.lqr(A, B, d.LQR_Q, d.LQR_R.get(self.eq, 0.1))[0])
        if mode == 4:
            return d.FullStateFeedback(d.place_gain(self.eq) if self.eq in d.PLACE_POLES else
                                       d.place_poles(A, B, [-3.0, -4.0, -5.0, -6.0]).gain_matrix)
        if mode == 5:
            return d.FullStateFeedback(d.e1_pd_gain())
        return acro.make_controller(mode, A, B, joint, kp, kd)

    def describe(self):
        if self.mode == 3:
            return f"full-state LQR (Q = I, R = {d.LQR_R.get(self.eq, 0.1):g})"
        if self.mode == 5:
            return f"elbow PD (Kp = {d.E1_PD[0]:g}, Kd = {d.E1_PD[1]:g})"
        return super().describe()


class Phase3Dashboard(acro.Dashboard):
    SIM = Phase3Sim
    TITLE = "Acrobot - Phase 3 controller explorer"
    MODE_LABELS = ["Open loop (τ = 0)", "PD on one angle", "Observer + LQR",
                   "Full-state LQR", "Pole placement", "Elbow PD (E1)"]
    DEFAULT_EQ, DEFAULT_MODE, DEFAULT_IC = 3, 3, (4.0, 0.0)


if __name__ == "__main__":
    Phase3Dashboard().run()
