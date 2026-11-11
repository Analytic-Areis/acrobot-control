"""
Acrobot Project - Phase 4 animation (Manim Community Edition): swing-up.

    manim -pql acrobot_phase4.py AcrobotPhase4                     # quick preview
    manim -r 2560,1440 --fps 60 acrobot_phase4.py AcrobotPhase4    # 2K final

Trajectories (PyBullet) and numbers come from phase4_data.npz and
../code/results.json, written by ../code/make_phase4_results.py.
"""

import json
import os

import numpy as np
from manim import *

HERE = os.path.dirname(os.path.abspath(__file__))
D = np.load(os.path.join(HERE, "phase4_data.npz"))
with open(os.path.join(HERE, "..", "code", "results.json")) as fh:
    RES = json.load(fh)

L1, L2 = 1.0, 2.0
C_LINK1, C_LINK2, C_TAU, C_OK, C_BAD, C_E = "#2E86DE", "#E67E22", "#E74C3C", "#2ECC71", "#E74C3C", "#9B59B6"
E_TOP = 24.525


def acrobot(pivot, q1, q2, s, width=7):
    el = pivot + s * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    tip = el + s * L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
    return VGroup(Line(pivot, el, color=C_LINK1, stroke_width=width * 1.5),
                  Line(el, tip, color=C_LINK2, stroke_width=width * 0.8),
                  Dot(el, radius=0.055, color=WHITE))


def mount(pivot):
    return VGroup(Dot(pivot, radius=0.075, color=WHITE), Circle(radius=0.12, color=GRAY_B, stroke_width=2).move_to(pivot))


def title_bar(text, sub=None):
    t = Text(text, font_size=34, weight=BOLD).to_edge(UP, buff=0.3)
    g = VGroup(t)
    if sub:
        g.add(Text(sub, font_size=20, color=GRAY_B).next_to(t, DOWN, buff=0.12))
    return g


def lines(rows, fs=21, buff=0.16):
    out = []
    for l, c in rows:
        out.append(Line(ORIGIN, UP * fs / 160, stroke_opacity=0) if l == "" else Text(l, font_size=fs, t2c=c))
    return VGroup(*out).arrange(DOWN, aligned_edge=LEFT, buff=buff)


def val(pre, key, u):
    a = D[f"{pre}_{key}"]
    return float(a[int(round(u * (len(a) - 1)))])


def clear(scene):
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def animate_run(scene, pre, label, T_show, pivot=np.array([-3.2, 0.5, 0]), s=0.85):
    """One PyBullet run: the arm, an energy bar and which controller is active."""
    u = ValueTracker(0.0)
    scene.play(FadeIn(mount(pivot)), FadeIn(Text(label, font_size=20).move_to(np.array([pivot[0], -2.5, 0]))), run_time=0.4)
    bar_x = 2.6
    frame = Rectangle(width=0.5, height=4.4, color=GRAY_B).move_to(np.array([bar_x, 0.0, 0]))
    top_line = DashedLine(frame.get_left() + LEFT * 0.2, frame.get_right() + RIGHT * 0.2, color=WHITE).move_to(
        np.array([bar_x, -2.2 + 4.4 * (E_TOP + 30) / 70, 0]))
    scene.play(FadeIn(frame), FadeIn(top_line),
               FadeIn(Text("energy", font_size=18).next_to(frame, UP, buff=0.15)),
               FadeIn(Text("E_top = 24.5 J", font_size=15, color=GRAY_B).next_to(top_line, RIGHT, buff=0.2)))

    def dyn():
        uu = u.get_value()
        q1, q2, E, tau = val(pre, "q1", uu), val(pre, "q2", uu), val(pre, "E", uu), val(pre, "tau", uu)
        lqr = val(pre, "lqr", uu) > 0.5
        h = 4.4 * np.clip((E + 30) / 70, 0, 1)
        fill = Rectangle(width=0.5, height=max(h, 0.01), fill_color=C_E, fill_opacity=0.8, stroke_width=0
                         ).align_to(frame, DOWN).set_x(bar_x)
        mode = Text("LQR: holding upright" if lqr else "Swing-up: pumping energy", font_size=22,
                    color=C_OK if lqr else YELLOW, weight=BOLD).move_to(np.array([4.5, 2.9, 0]))
        info = Text(f"t = {uu * T_show:4.1f} s   τ = {tau:+5.1f} N·m", font_size=17, color=GRAY_B
                    ).move_to(np.array([pivot[0], -3.25, 0]))
        return VGroup(acrobot(pivot, q1, q2, s), fill, mode, info)
    g = always_redraw(dyn)
    scene.add(g)
    scene.play(u.animate.set_value(1.0), run_time=T_show, rate_func=linear)
    return g


def part_idea(scene):
    head = title_bar("Phase 4 · Swing-up", "start hanging, end balanced upright, |τ| ≤ 20 N·m")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    txt = lines([
        ("Phase 3: LQR only catches the arm close to upright (≈ 6°)", {}),
        ("hanging → upright needs E: −24.5 J → +24.5 J  (add 49 J)", {"+24.5 J": YELLOW}),
        ("", {}),
        ("1. Swing-up: pump energy like a child on a swing", {"1. Swing-up": YELLOW}),
        ("   v = −k₁ q̃₂ − k₂ q̇₂ + k₃ q̇₁ (E − E_top)      desired elbow acceleration", {}),
        ("   τ = (M₂₂ − M₂₁M₁₂/M₁₁) v + h₂ − M₂₁h₁/M₁₁     (partial feedback linearization)", {}),
        ("2. Catch: when x̃ᵀ S x̃ < 5, switch to the Phase 3 LQR", {"2. Catch": C_OK}),
        ("3. Knocked out of the catch zone? swing-up takes over again", {"3.": C_OK}),
        ("", {}),
        (f"gains k = (20, 2, 3): {RES['chosen']['success_of_40']}/40 random starts held, "
         f"median catch {RES['chosen']['median_catch_s']:.1f} s", {}),
    ], fs=21).next_to(head, DOWN, buff=0.45)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.4)
    scene.wait(3)
    clear(scene)


def part_run(scene):
    head = title_bar("Swing-up in PyBullet", "start 3° from hanging · real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    animate_run(scene, "good", "k = (20, 2, 3)", float(D["good_t"][-1]))
    scene.wait(1.5)
    clear(scene)


def part_why(scene):
    head = title_bar("Why some gains fail", "stiff elbow → asks for > 20 N·m → saturation breaks the swing-up law")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    rows = [("gains (k₁, k₂, k₃)", "swing-up at τ limit", "held from 40 starts")]
    for s in RES["saturation_study"]:
        g = s["gains"]
        rows.append((f"({g[0]:g}, {g[1]:g}, {g[2]:g}), c = {g[3]:g}", f"{s['swing_saturated_pct']:.0f} %",
                     f"{s['success_of_40']} / 40"))
    table = VGroup()
    for i, r in enumerate(rows):
        row = VGroup(*[Text(c, font_size=21, weight=BOLD if i == 0 else NORMAL, color=YELLOW if i == 0 else WHITE)
                       for c in r])
        for j, c in enumerate(row):
            c.move_to(np.array([-3.8 + 3.8 * j, 1.9 - 0.58 * i, 0]))
        table.add(row)
    for row in table:
        scene.play(FadeIn(row, shift=UP * 0.1), run_time=0.4)
    note = lines([
        ("larger k₁ → more saturation → elbow no longer follows v → energy pumping lands badly", {}),
        ("k₃ too small (0.3) → energy grows too slowly, never reaches the catch zone", {}),
    ], fs=19).to_edge(DOWN, buff=0.45)
    scene.play(FadeIn(note))
    scene.wait(3)
    clear(scene)


def part_kick(scene):
    head = title_bar("Knocked off? It swings back up", "kick of 0.8 rad/s at t = 8 s · real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    animate_run(scene, "kick", "k = (20, 2, 3), kick at 8 s", float(D["kick_t"][-1]))
    scene.wait(1.5)
    clear(scene)


def part_end(scene):
    head = title_bar("Phase 4 · Summary")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    r = RES["runs"]
    txt = lines([
        (f"from 3°: caught after {r['good']['t_catch']:.1f} s, held to {max(map(abs, r['good']['end_dev'])):.3f}°", {}),
        (f"from (−10°, 20°): caught after {r['awkward']['t_catch']:.1f} s", {}),
        (f"robust: {RES['chosen']['success_of_40']} of 40 random starts (textbook gains: "
         f"{RES['saturation_study'][4]['success_of_40']} of 40)", {}),
        ("after a kick: swing-up resumes and the arm is caught again", {}),
        ("", {}),
        ("why it works: energy pumping gets the arm near the top;", {"why it works": YELLOW}),
        ("the Phase 3 LQR only has to catch it there", {}),
        ("", {}),
        ("simulator: phase4/acrobot_phase4_sim.py", {"phase4/acrobot_phase4_sim.py": C_LINK1}),
    ], fs=22).next_to(head, DOWN, buff=0.5)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.4)
    scene.wait(4)


class AcrobotPhase4(Scene):
    def construct(self):
        intro = VGroup(Text("Acrobot · Phase 4", font_size=48, weight=BOLD),
                       Text("Swing-up", font_size=28, color=GRAY_B),
                       Text("Bhargav · Sreekar · Kaushik · Divijat", font_size=22, color=GRAY_C)).arrange(DOWN, buff=0.3)
        self.play(FadeIn(intro, shift=UP * 0.2))
        self.wait(1.5)
        self.play(FadeOut(intro))
        part_idea(self)
        part_run(self)
        part_why(self)
        part_kick(self)
        part_end(self)
