"""
Acrobot Project - Phase 3 animation (Manim Community Edition)
==============================================================

Controllers at the upright and at the other equilibria, the choice of the LQR
weights, and how far from each equilibrium they still work with |tau| <= 20 N m.

    manim -pql acrobot_phase3.py AcrobotPhase3                     # quick preview
    manim -r 2560,1440 --fps 60 acrobot_phase3.py AcrobotPhase3    # 2K final

All trajectories (PyBullet), maps and numbers come from phase3_data.npz and
../code/results.json, written by ../code/make_phase3_results.py, so the video
always matches the report. Only `Text` is used (no LaTeX needed).
"""

import json
import os

import numpy as np
from manim import *

HERE = os.path.dirname(os.path.abspath(__file__))
D = np.load(os.path.join(HERE, "phase3_data.npz"))
with open(os.path.join(HERE, "..", "code", "results.json")) as fh:
    RES = json.load(fh)

L1, L2 = 1.0, 2.0
EQ = {1: (0.0, 0.0), 2: (0.0, np.pi), 3: (np.pi, 0.0), 4: (np.pi, np.pi)}
C_LINK1, C_LINK2, C_TAU = "#2E86DE", "#E67E22", "#E74C3C"
C_OK, C_BAD, C_WARN = "#2ECC71", "#E74C3C", "#F1C40F"


# ---------------------------------------------------------------- helpers
def tip_positions(pivot, q1, q2, scale):
    elbow = pivot + scale * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    tip = elbow + scale * L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
    return elbow, tip


def acrobot(pivot, q1, q2, scale, width=7):
    elbow, tip = tip_positions(pivot, q1, q2, scale)
    return VGroup(Line(pivot, elbow, color=C_LINK1, stroke_width=width * 1.5),
                  Line(elbow, tip, color=C_LINK2, stroke_width=width * 0.8),
                  Dot(elbow, radius=0.055, color=WHITE))


def torque_arc(pivot, q1, q2, scale, tau, radius=0.3):
    elbow, tip = tip_positions(pivot, q1, q2, scale)
    frac = float(np.clip(tau / 20.0, -1, 1))
    if abs(frac) < 0.02:
        return VGroup()
    base = np.arctan2(*(tip - elbow)[[1, 0]])
    return Arc(radius=radius, start_angle=base, angle=frac * 1.6 * PI, arc_center=elbow,
               color=C_TAU, stroke_width=4)


def pivot_mount(pivot):
    return VGroup(Dot(pivot, radius=0.075, color=WHITE),
                  Circle(radius=0.12, color=GRAY_B, stroke_width=2).move_to(pivot))


def title_bar(text, sub=None):
    t = Text(text, font_size=34, weight=BOLD).to_edge(UP, buff=0.3)
    grp = VGroup(t)
    if sub:
        grp.add(Text(sub, font_size=20, color=GRAY_B).next_to(t, DOWN, buff=0.12))
    return grp


def step_lines(lines, font_size=22, buff=0.2):
    rows = []
    for l, c in lines:
        if l == "":
            rows.append(Line(ORIGIN, UP * font_size / 160, stroke_opacity=0))
        else:
            rows.append(Text(l, font_size=font_size, t2c=c))
    return VGroup(*rows).arrange(DOWN, aligned_edge=LEFT, buff=buff)


def sample(prefix, key, u):
    arr = D[f"{prefix}_{key}"]
    return float(arr[int(round(u * (len(arr) - 1)))])


def roa_image(name, height=4.2):
    ok = D[f"roa_{name}"].astype(bool)
    rgb = np.where(ok[..., None], np.array([46, 204, 113]), np.array([90, 30, 30])).astype(np.uint8)
    img = ImageMobject(np.flipud(rgb))
    img.set_resampling_algorithm(RESAMPLING_ALGORITHMS["nearest"])
    img.height = height
    return img


def clear(scene):
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def run_panel(scene, cases, T_show, scale=0.75, y_pivot=-0.9, label_y=2.35, info_y=-3.2):
    """cases: (prefix, eq, label, result, colour, x0). Animates PyBullet runs side by side."""
    u = ValueTracker(0.0)
    groups = []
    for pre, eq, label, res, col, x0 in cases:
        qs = EQ[eq]
        pivot = np.array([x0, y_pivot, 0])
        scene.play(FadeIn(Text(label, font_size=19).move_to(np.array([x0, label_y, 0]))),
                   FadeIn(pivot_mount(pivot)), run_time=0.4)
        T = float(D[f"{pre}_t"][-1])

        def rob(pre=pre, qs=qs, pivot=pivot, T=T, x0=x0):
            uu = min(1.0, u.get_value() * T_show / T)
            q1 = qs[0] + np.radians(sample(pre, "q1", uu))
            q2 = qs[1] + np.radians(sample(pre, "q2", uu))
            tau = sample(pre, "tau", uu)
            info = Text(f"Δq₁ {sample(pre, 'q1', uu):+7.2f}°  Δq₂ {sample(pre, 'q2', uu):+7.2f}°  τ {tau:+5.1f}",
                        font_size=13, color=GRAY_B).move_to(np.array([x0, info_y, 0]))
            return VGroup(acrobot(pivot, q1, q2, scale), torque_arc(pivot, q1, q2, scale, tau), info)
        groups.append(always_redraw(rob))
    scene.add(*groups)
    scene.play(u.animate.set_value(1.0), run_time=T_show, rate_func=linear)
    scene.play(*[FadeIn(Text(r, font_size=20, color=c, weight=BOLD).move_to(np.array([x0, label_y - 0.4, 0])))
                 for _, _, _, r, c, x0 in cases])


# ------------------------------------------------------------------ parts
def part_approach(scene):
    head = title_bar("Phase 3 · Why full-state feedback", "Phase 2: one angle is not enough at the upright")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    lines = step_lines([
        ("Phase 2:  PD from one angle impossible,  observer holds only ≈ 1°", {"impossible": C_BAD}),
        ("rank [B AB A²B A³B] = 4 at every equilibrium  →  feed back all four states", {"rank [B AB A²B A³B] = 4": C_OK}),
        ("", {}),
        ("τ = sat( −K x̃ ),   x̃ = [q̃₁, q̃₂, q̇₁, q̇₂],   |τ| ≤ 20 N·m", {"τ = sat( −K x̃ )": YELLOW}),
        ("", {}),
        ("LQR:  minimise  J = ∫ ( x̃ᵀ Q x̃ + R τ² ) dt", {"LQR": YELLOW}),
        ("      AᵀS + SA − S B R⁻¹ Bᵀ S + Q = 0,      K = R⁻¹ Bᵀ S", {}),
        ("      always stable · phase margin ≥ 60° · gain margin [½, ∞)", {}),
        ("      Q = I   (1 rad of angle ≈ 1 rad/s of rate),   R chosen by experiment", {}),
        ("", {}),
        ("E₁: PD on the elbow angle only   (zeros at E₁ are imaginary)", {"E₁": C_OK}),
    ], font_size=21, buff=0.16).next_to(head, DOWN, buff=0.5)
    for l in lines:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.45)
    scene.wait(3)
    clear(scene)


def part_design(scene):
    head = title_bar("Choosing R", "simulate the nonlinear saturated loop from a grid of offsets, for every R")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    Rs = D["sweep_R"]
    lx = np.log10(Rs)
    ax = Axes(x_range=[-3, 0, 1], y_range=[0, 100, 20], x_length=6.0, y_length=4.0, tips=False,
              axis_config=dict(stroke_width=1.5, color=GRAY_B)).to_edge(LEFT, buff=0.9).shift(DOWN * 0.5)
    xt = VGroup(*[Text(s, font_size=14, color=GRAY_B).next_to(ax.c2p(e, 0), DOWN, buff=0.1)
                  for e, s in ((-3, "0.001"), (-2, "0.01"), (-1, "0.1"), (0, "1"))])
    yt = VGroup(*[Text(f"{v}%", font_size=14, color=GRAY_B).next_to(ax.c2p(-3, v), LEFT, buff=0.1)
                  for v in (0, 50, 100)])
    xl = Text("R  (Q = I)", font_size=15, color=GRAY_B).next_to(ax, DOWN, buff=0.45)
    yl = Text("share of offsets recovered", font_size=15, color=GRAY_B).next_to(ax, UP, buff=0.1)
    scene.play(Create(ax), FadeIn(xt), FadeIn(yt), FadeIn(xl), FadeIn(yl))
    cols = {3: "#E74C3C", 2: "#2ECC71", 4: "#9B59B6"}
    chosen = {3: 0.03, 2: 0.1, 4: 0.1}
    leg = VGroup()
    for eq in (3, 2, 4):
        a = 100 * D[f"sweep_area_E{eq}"]
        line = VMobject(stroke_color=cols[eq], stroke_width=4).set_points_as_corners(
            [ax.c2p(x, y) for x, y in zip(lx, a)])
        i = int(np.argmin(abs(Rs - chosen[eq])))
        star = Star(n=5, outer_radius=0.13, color=WHITE, fill_opacity=1).move_to(ax.c2p(lx[i], a[i]))
        scene.play(Create(line), run_time=1.2)
        scene.play(FadeIn(star), run_time=0.3)
        leg.add(Text(f"E{'₂₃₄'[eq - 2]}: R = {chosen[eq]:g}", font_size=19, color=cols[eq]))
    notes = step_lines([
        ("E₃: area almost flat (16–20 %) for R = 0.001 … 1", {"E₃": "#E74C3C"}),
        ("   but the fastest pole goes −10 → −214 1/s", {}),
        ("   ⇒ the limit is τ_max, not the tuning", {"the limit is τ_max": YELLOW}),
        ("   R = 0.03: 93 % of best area, pole −40", {}),
        ("E₂: collapses for R > 0.3  ⇒  R = 0.1", {"E₂": "#2ECC71"}),
        ("E₄: peaks near R = 0.1  ⇒  R = 0.1", {"E₄": "#9B59B6"}),
        ("", {}),
        (f"pole placement at E₃ (−3…−6): {RES['E3_place']['roa_area_pct']:.0f} % vs LQR {RES['E3_lqr']['roa_area_pct']:.0f} %",
         {}),
    ], font_size=19, buff=0.14).to_edge(RIGHT, buff=0.5).shift(DOWN * 0.3)
    for l in notes:
        scene.play(FadeIn(l, shift=LEFT * 0.1), run_time=0.4)
    scene.wait(3)
    clear(scene)


def part_upright(scene):
    head = title_bar("Upright E₃ · LQR in PyBullet", "torque arc at the elbow · |τ| ≤ 20 N·m · real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    r = RES["runs"]
    run_panel(scene, [("e3_small", 3, "start q̃₁ = 2°", "held", C_OK, -4.6),
                      ("e3_edge", 3, f"start q̃₁ = {r['e3_edge']['start'][0]:g}°", "held (saturates)", C_OK, 0.0),
                      ("e3_fail", 3, f"start q̃₁ = {r['e3_fail']['start'][0]:g}°", "τ_max → falls", C_BAD, 4.6)],
              T_show=6.0)
    scene.wait(2)
    clear(scene)

    head = title_bar("How far from the upright does it still work?",
                     "green: recovered from rest (nonlinear, |τ| ≤ 20)  ·  q̃₁(0), q̃₂(0) ∈ ±40°")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    img = roa_image("E3_lqr", 4.4).to_edge(LEFT, buff=0.9).shift(DOWN * 0.45)
    frame = SurroundingRectangle(img, buff=0, color=GRAY_B, stroke_width=1.5)
    lim = 40.0

    def to_img(a, b):
        return img.get_center() + RIGHT * (a / lim) * img.width / 2 + UP * (b / lim) * img.height / 2
    com = DashedLine(to_img(-16, 40), to_img(16, -40), color=WHITE, stroke_width=2.5)
    lab = VGroup(Text("q̃₁ →", font_size=15, color=GRAY_B).next_to(img, DOWN, buff=0.1),
                 Text("q̃₂ →", font_size=15, color=GRAY_B).rotate(PI / 2).next_to(img, LEFT, buff=0.1))
    scene.play(FadeIn(img), Create(frame), FadeIn(lab))
    scene.play(Create(com))
    e = RES["E3_lqr"]
    txt = step_lines([
        (f"recovered: {e['roa_area_pct']:.0f} % of the ±40° grid", {}),
        (f"alone:  |q̃₁| ≤ {e['axis_q1']:g}°   or   |q̃₂| ≤ {e['axis_q2']:g}°", {}),
        ("along the dashed line: to the grid edge", {"to the grid edge": C_OK}),
        ("", {}),
        ("dashed = centre of mass over the pivot:", {"dashed": YELLOW}),
        ("τ_g,shoulder = 9.81 (2.5 q̃₁ + q̃₂) = 0", {}),
        ("the shoulder has no motor, so every degree", {}),
        ("of CoM offset must be fought via the elbow", {}),
        (f"2° already costs {e['peak_tau']:.1f} N·m of the 20", {}),
        ("", {}),
        (f"guaranteed (never saturates): {e['ellipse_q1']:.2f}° / {e['ellipse_q2']:.2f}°", {"guaranteed": C_LINK1}),
    ], font_size=18, buff=0.11).to_edge(RIGHT, buff=0.45).shift(DOWN * 0.35)
    for l in txt:
        scene.play(FadeIn(l, shift=LEFT * 0.1), run_time=0.35)
    scene.wait(2.5)
    clear(scene)

    head = title_bar("A large offset along the line is easy", "start (−10°, 25°): CoM almost over the pivot")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    run_panel(scene, [("e3_diag", 3, "LQR · start (−10°, 25°)",
                       f"held · peak τ {RES['runs']['e3_diag']['peak_tau']:.1f} N·m", C_OK, 0.0)], T_show=5.0,
              scale=0.85)
    scene.wait(1.5)
    clear(scene)


def part_others(scene):
    head = title_bar("The other equilibria", "PyBullet · real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    run_panel(scene, [("e2", 2, "E₂ · LQR · start (60°, 90°)", "held", C_OK, -4.6),
                      ("e4", 4, "E₄ · LQR · start (20°, 30°)", "held", C_OK, 0.0),
                      ("e1", 1, "E₁ · elbow PD · start (60°, −60°)", "held", C_OK, 4.6)],
              T_show=8.0, scale=0.62, y_pivot=0.1, info_y=-2.45)
    note = Text("E₁: Vc = E − E* + ½ Kp q̃₂²  decreases (V̇c = −Kd q̇₂²)  ⇒  asymptotically stable",
                font_size=18, color=YELLOW).to_edge(DOWN, buff=0.25)
    scene.play(FadeIn(note))
    scene.wait(2.5)
    clear(scene)

    head = title_bar("Regions of attraction", "green: recovered from rest  ·  q̃₁(0), q̃₂(0) ∈ ±180°")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    items = Group()
    for name, lab in (("E2_lqr", "E₂ LQR"), ("E4_lqr", "E₄ LQR"), ("E1_pd", "E₁ elbow PD")):
        img = roa_image(name, 3.3)
        t = Text(f"{lab}: {RES[name]['roa_area_pct']:.0f} %", font_size=20).next_to(img, UP, buff=0.15)
        items.add(Group(img, SurroundingRectangle(img, buff=0, color=GRAY_B, stroke_width=1.5), t))
    items.arrange(RIGHT, buff=0.7).shift(DOWN * 0.3)
    scene.play(LaggedStart(*[FadeIn(i) for i in items], lag_ratio=0.3))
    scene.wait(3)
    clear(scene)


def part_summary(scene):
    head = title_bar("Phase 3 · How far each controller works (|τ| ≤ 20 N·m)")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    rows = [("controller", "recovered", "q̃₁ alone", "q̃₂ alone", "kick |q̇₁|")]
    for name, lab in (("E3_lqr", "E₃ LQR"), ("E3_place", "E₃ pole pl."), ("E2_lqr", "E₂ LQR"),
                      ("E4_lqr", "E₄ LQR"), ("E1_pd", "E₁ elbow PD")):
        e = RES[name]
        rows.append((lab, f"{e['roa_area_pct']:.0f} % (±{e['grid_deg']}°)", f"{e['axis_q1']:g}°",
                     f"{e['axis_q2']:g}°", f"{e['kick_rad_s']:g} rad/s"))
    table = VGroup()
    for i, r in enumerate(rows):
        row = VGroup(*[Text(c, font_size=21, weight=BOLD if i == 0 else NORMAL,
                            color=YELLOW if i == 0 else WHITE) for c in r])
        for j, c in enumerate(row):
            c.move_to(np.array([-5.2 + 2.6 * j, 1.8 - 0.62 * i, 0]))
        table.add(row)
    for row in table:
        scene.play(FadeIn(row, shift=UP * 0.1), run_time=0.45)
    concl = step_lines([
        ("upright: full-state LQR, limited by τ_max to a thin band around the CoM-over-pivot line", {}),
        ("E₂ almost global, E₄ moderate, E₁ stabilised from one angle (energy shaping)", {}),
        ("interactive simulator: phase3/acrobot_phase3_sim.py   ·   MATLAB: code/matlab", {"phase3/acrobot_phase3_sim.py": C_LINK1}),
    ], font_size=19, buff=0.15).to_edge(DOWN, buff=0.45)
    scene.play(FadeIn(concl))
    scene.wait(4)


# ----------------------------------------------------------------- scenes
class ApproachScene(Scene):
    def construct(self):
        part_approach(self)


class DesignScene(Scene):
    def construct(self):
        part_design(self)


class UprightScene(Scene):
    def construct(self):
        part_upright(self)


class OthersScene(Scene):
    def construct(self):
        part_others(self)


class AcrobotPhase3(Scene):
    def construct(self):
        intro = VGroup(
            Text("Acrobot · Phase 3", font_size=48, weight=BOLD),
            Text("Control Around the Equilibria", font_size=28, color=GRAY_B),
            Text("Bhargav · Sreekar · Kaushik · Divijat", font_size=22, color=GRAY_C),
        ).arrange(DOWN, buff=0.3)
        self.play(FadeIn(intro, shift=UP * 0.2))
        self.wait(1.5)
        self.play(FadeOut(intro))
        part_approach(self)
        part_design(self)
        part_upright(self)
        part_others(self)
        part_summary(self)
