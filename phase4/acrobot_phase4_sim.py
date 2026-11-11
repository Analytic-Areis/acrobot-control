"""
Phase 4 - Swing-up: interactive PyBullet simulation.

The Phase 3 dashboard plus one controller, "Swing-up -> LQR": it pumps energy
from hanging until the arm reaches the upright E3, then the Phase 3 LQR catches
and holds it (and swing-up takes over again if the arm is knocked out).

It opens on E1 (hanging) with a 3 deg offset and swing-up selected: the plots
show the deviation from E1, the arm ends upright. Press Kick shoulder once it
is up to see it fall, swing back up and be caught again.

Usage
    ../phase2/.venv/bin/python acrobot_phase4_sim.py
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase3"))
import acrobot_phase3_sim as s3  # noqa: E402

import acrobot_phase4_design as d4  # noqa: E402

acro = d4.acro


class Phase4Sim(s3.Phase3Sim):
    MODES = {**s3.Phase3Sim.MODES, 6: "swing-up -> LQR (E3)"}

    def make_controller(self, mode, A, B, joint, kp, kd):
        if mode == 6:
            return d4.SwingUpLQR(acro.EQUILIBRIA[self.eq][1])
        return super().make_controller(mode, A, B, joint, kp, kd)

    def describe(self):
        if self.mode == 6:
            c = self.ctrl
            state = "LQR holding upright" if c.mode == "lqr" else "pumping energy"
            caught = "" if c.t_catch is None else f", first caught at {c.t_catch:.1f} s"
            return f"swing-up -> LQR: {state}{caught}"
        return super().describe()


class Phase4Dashboard(s3.Phase3Dashboard):
    SIM = Phase4Sim
    TITLE = "Acrobot - Phase 4 swing-up explorer"
    MODE_LABELS = s3.Phase3Dashboard.MODE_LABELS + ["Swing-up → LQR"]
    DEFAULT_EQ, DEFAULT_MODE, DEFAULT_IC = 1, 6, (3.0, 0.0)

    def frame(self, i):
        super().frame(i)
        if self.sim.mode == 6:            # live text: which half of the hybrid controller is on
            c = self.sim.ctrl
            self.h_verdict.set_text(("LQR holding upright" if c.mode == "lqr" else "Swing-up: pumping energy")
                                    + ("" if c.t_catch is None else f"\n(first caught at {c.t_catch:.1f} s)"))
            self.h_verdict.set_color(acro.COL["ok"] if self.sim.ctrl.mode == "lqr" else acro.COL["warn"])


if __name__ == "__main__":
    Phase4Dashboard().run()
