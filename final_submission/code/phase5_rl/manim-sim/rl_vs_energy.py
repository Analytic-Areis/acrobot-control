"""
Acrobot - RL swing-up vs energy swing-up (Manim Community Edition).

    manim -pql rl_vs_energy.py RLvsEnergy                     # quick preview
    manim -r 2560,1440 --fps 60 rl_vs_energy.py RLvsEnergy    # 2K final

Data: ../video_data.npz and ../results.json, written by ../evaluate.py
(PyBullet runs and the 40-start comparisons), plus ../training_log.json and
../training_log_robust.json from train.py.
"""

import json
import os

import numpy as np
from manim import *

HERE = os.path.dirname(os.path.abspath(__file__))
D = np.load(os.path.join(HERE, "..", "video_data.npz"))
with open(os.path.join(HERE, "..", "results.json")) as fh:
    RES = json.load(fh)
TRAIN = TRAIN_R = None
if os.path.exists(os.path.join(HERE, "..", "training_log.json")):
    with open(os.path.join(HERE, "..", "training_log.json")) as fh:
        TRAIN = json.load(fh)
if os.path.exists(os.path.join(HERE, "..", "training_log_robust.json")):
    with open(os.path.join(HERE, "..", "training_log_robust.json")) as fh:
        TRAIN_R = json.load(fh)

L1, L2 = 1.0, 2.0
C_LINK1, C_LINK2 = "#2E86DE", "#E67E22"
C_EN, C_RL, C_OK, C_BAD, C_E = "#5DADE2", "#F5B041", "#2ECC71", "#E74C3C", "#9B59B6"
C_RB = "#58D68D"                 # robust RL policy (random masses)
E_TOP = 24.525


def acrobot(pivot, q1, q2, s, width=7):
    el = pivot + s * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    tip = el + s * L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
    return VGroup(Line(pivot, el, color=C_LINK1, stroke_width=width * 1.5),
                  Line(el, tip, color=C_LINK2, stroke_width=width * 0.8),
                  Dot(el, radius=0.05, color=WHITE))


def mount(pivot):
    return VGroup(Dot(pivot, radius=0.07, color=WHITE), Circle(radius=0.11, color=GRAY_B, stroke_width=2).move_to(pivot))


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


def card(title, rows, colour, width=6.0):
    head = Text(title, font_size=24, weight=BOLD, color=colour)
    body = lines(rows, fs=18, buff=0.12)
    grp = VGroup(head, body).arrange(DOWN, aligned_edge=LEFT, buff=0.25)
    box = SurroundingRectangle(grp, buff=0.3, color=colour, corner_radius=0.15, stroke_width=2)
    return VGroup(box, grp)


# ------------------------------------------------------------------ parts
def part_intro(scene):
    g = VGroup(Text("Swinging up the Acrobot", font_size=46, weight=BOLD),
               Text("a controller designed from physics  vs  a controller learned by trial and error",
                    font_size=22, color=GRAY_B),
               Text("Bhargav · Sreekar · Kaushik · Divijat", font_size=20, color=GRAY_C)).arrange(DOWN, buff=0.3)
    scene.play(FadeIn(g, shift=UP * 0.2))
    scene.wait(2)
    scene.play(FadeOut(g))

    head = title_bar("The task", "same for both methods")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    pivot = np.array([-4.3, 0.6, 0])
    scene.play(FadeIn(mount(pivot)), FadeIn(acrobot(pivot, 0.05, 0.0, 0.8)))
    up = acrobot(pivot + RIGHT * 2.2, np.pi, 0.0, 0.8, width=7).set_opacity(0.45)
    scene.play(FadeIn(mount(pivot + RIGHT * 2.2)), FadeIn(up))
    arrow = Arrow(pivot + RIGHT * 0.4 + DOWN * 1.2, pivot + RIGHT * 1.8 + UP * 1.3, color=YELLOW, buff=0)
    scene.play(GrowArrow(arrow))
    txt = lines([
        ("start hanging, end balanced upright", {}),
        ("only the elbow has a motor, at most 20 N·m", {"20 N·m": YELLOW}),
        ("energy needed: −24.5 J  →  +24.5 J", {}),
        ("", {}),
        ("both methods: swing up, then hand over to the", {}),
        ("same Phase 3 LQR once inside its catch zone", {"same Phase 3 LQR": C_OK}),
        ("→ the only difference is WHO does the swing-up", {"WHO": YELLOW}),
    ], fs=21).to_edge(RIGHT, buff=0.6).shift(DOWN * 0.3)
    for l in txt:
        scene.play(FadeIn(l, shift=LEFT * 0.1), run_time=0.4)
    scene.wait(2.5)
    clear(scene)


def part_methods(scene):
    head = title_bar("Two ways to build the swing-up")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    a = card("Energy swing-up (Phase 4)", [
        ("written by hand from the equations of motion", {}),
        ("v = −k₁q̃₂ − k₂q̇₂ + k₃ q̇₁ (E − E_top)", {}),
        ("τ = (M₂₂ − M₂₁M₁₂/M₁₁) v + h₂ − M₂₁h₁/M₁₁", {}),
        ("", {}),
        ("+ we know WHY it works (energy)", {"WHY": C_OK}),
        ("+ 3 gains to tune", {}),
        ("− needs an exact model (uses M, C, g)", {"− needs an exact model": C_BAD}),
    ], C_EN)
    b = card("RL swing-up (SAC)", [
        ("a neural network: angles & speeds → torque", {}),
        ("6 inputs → 256 → 256 → 1 output", {}),
        ("learned by trying, getting a score, improving", {}),
        ("", {}),
        ("+ needs no equations, only a simulator", {"+ needs no equations": C_OK}),
        ("+ can be trained on many different robots", {}),
        ("− a black box: hard to prove it is safe", {"− a black box": C_BAD}),
    ], C_RL)
    VGroup(a, b).arrange(RIGHT, buff=0.5).next_to(head, DOWN, buff=0.5)
    scene.play(FadeIn(a, shift=UP * 0.2))
    scene.wait(1.5)
    scene.play(FadeIn(b, shift=UP * 0.2))
    scene.wait(4)
    clear(scene)


def part_learning(scene):
    head = title_bar("How the RL controller learns", "reinforcement learning = practice with a score")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    boxes = VGroup(*[RoundedRectangle(width=2.9, height=1.1, corner_radius=0.15, color=c) for c in
                     (C_RL, WHITE, YELLOW)]).arrange(RIGHT, buff=1.4).shift(UP * 1.75)
    names = VGroup(Text("policy\n(neural net)", font_size=19), Text("simulator\n(acrobot model)", font_size=19),
                   Text("reward\n(the score)", font_size=19))
    for n, bx in zip(names, boxes):
        n.move_to(bx)
    arr = VGroup(Arrow(boxes[0].get_right(), boxes[1].get_left(), buff=0.1),
                 Arrow(boxes[1].get_right(), boxes[2].get_left(), buff=0.1),
                 CurvedArrow(boxes[2].get_bottom() + DOWN * 0.05, boxes[0].get_bottom() + DOWN * 0.05, angle=-0.7))
    lab = VGroup(Text("torque τ", font_size=16, color=GRAY_B).next_to(arr[0], UP, buff=0.05),
                 Text("new state", font_size=16, color=GRAY_B).next_to(arr[1], UP, buff=0.05),
                 Text("improve the policy (SAC)", font_size=16, color=GRAY_B).move_to(boxes[1].get_bottom() + DOWN * 0.55))
    scene.play(LaggedStart(*[Create(b) for b in boxes], lag_ratio=0.2), FadeIn(names))
    scene.play(LaggedStart(*[GrowArrow(a) if not isinstance(a, CurvedArrow) else Create(a) for a in arr],
                           lag_ratio=0.3), FadeIn(lab))
    rows = [("reward every 20 ms  (higher is better):", {"reward": YELLOW}),
            ("  −(E − E_top)²/E_top²     get exactly the upright energy", {}),
            ("  −0.3 (1 + cos q₁)        point link 1 up", {}),
            ("  −0.1 (1 − cos q₂)        keep the elbow straight", {}),
            ("  −0.001 (τ/20)²           don't waste torque", {}),
            ("  +100 on reaching the LQR catch zone  → episode ends", {"+100": C_OK})]
    if TRAIN:
        rows.append(("", {}))
        rows.append((f"trained for {TRAIN['steps']:,} steps (≈ {TRAIN['episodes']:,} practice swings, "
                     f"{TRAIN['minutes']:.0f} min on a laptop)", {}))
    txt = lines(rows, fs=18, buff=0.1).to_edge(DOWN, buff=0.3)
    scene.play(FadeIn(txt))
    scene.wait(4)
    clear(scene)

    if TRAIN and len(TRAIN.get("curve", [])) > 2:
        head = title_bar("Learning curve", "share of practice swings that reached the catch zone")
        scene.play(FadeIn(head, shift=DOWN * 0.2))
        steps = np.array([c[0] for c in TRAIN["curve"]], float)
        rate = np.array([c[1] for c in TRAIN["curve"]], float)
        ax = Axes(x_range=[0, steps.max(), steps.max() / 5], y_range=[0, 100, 25], x_length=9, y_length=4.5,
                  tips=False, axis_config=dict(stroke_width=1.5, color=GRAY_B)).shift(DOWN * 0.4)
        xl = Text("training steps", font_size=16, color=GRAY_B).next_to(ax, DOWN, buff=0.2)
        yl = VGroup(*[Text(f"{v}%", font_size=14, color=GRAY_B).next_to(ax.c2p(0, v), LEFT, buff=0.1) for v in (0, 50, 100)])
        scene.play(Create(ax), FadeIn(xl), FadeIn(yl))
        curve = VMobject(stroke_color=C_RL, stroke_width=4).set_points_as_corners([ax.c2p(s, r) for s, r in zip(steps, rate)])
        scene.play(Create(curve), run_time=3)
        scene.wait(2)
        clear(scene)


def part_race(scene):
    head = title_bar("Same start, side by side (PyBullet)", "3° from hanging · real time · purple bar = energy")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    u = ValueTracker(0.0)
    T = float(min(D["energy_t"][-1], D["rl_t"][-1]))
    groups = []
    for pre, name, col, x0 in (("energy", "Energy swing-up", C_EN, -3.4), ("rl", "RL swing-up (SAC)", C_RL, 3.4)):
        pivot = np.array([x0 - 0.6, 0.1, 0])
        scene.play(FadeIn(Text(name, font_size=24, color=col, weight=BOLD).move_to(np.array([x0, 2.35, 0]))),
                   FadeIn(mount(pivot)), run_time=0.4)
        frame = Rectangle(width=0.35, height=3.6, color=GRAY_B).move_to(np.array([x0 + 1.9, -0.3, 0]))
        top = DashedLine(frame.get_left() + LEFT * 0.15, frame.get_right() + RIGHT * 0.15, color=WHITE).move_to(
            np.array([x0 + 1.9, -2.1 + 3.6 * (E_TOP + 30) / 70, 0]))
        scene.play(FadeIn(frame), FadeIn(top), run_time=0.3)
        tc = RES["pybullet"][pre]["t_catch"]

        def dyn(pre=pre, pivot=pivot, frame=frame, x0=x0, tc=tc):
            uu = u.get_value()
            n = len(D[f"{pre}_t"])
            k = min(int(round(uu * T / (D[f"{pre}_t"][1] - D[f"{pre}_t"][0]))), n - 1)
            q1, q2, E, tau = D[f"{pre}_q1"][k], D[f"{pre}_q2"][k], D[f"{pre}_E"][k], D[f"{pre}_tau"][k]
            lqr = D[f"{pre}_lqr"][k] > 0.5
            h = 3.6 * np.clip((E + 30) / 70, 0, 1)
            fill = Rectangle(width=0.35, height=max(h, 0.01), fill_color=C_E, fill_opacity=0.8, stroke_width=0
                             ).align_to(frame, DOWN).set_x(frame.get_x())
            mode = Text("LQR holding" if lqr else "swinging up", font_size=19,
                        color=C_OK if lqr else YELLOW).move_to(np.array([x0 - 0.6, -2.45, 0]))
            info = Text(f"t = {D[f'{pre}_t'][k]:4.1f} s   τ = {tau:+5.1f} N·m", font_size=15, color=GRAY_B
                        ).move_to(np.array([x0, -2.95, 0]))
            return VGroup(acrobot(pivot, q1, q2, 0.62), fill, mode, info)
        groups.append(always_redraw(dyn))
    scene.add(*groups)
    scene.play(u.animate.set_value(1.0), run_time=T, rate_func=linear)
    pe, pr = RES["pybullet"]["energy"], RES["pybullet"]["rl"]

    def res(s):
        return f"caught at {s['t_catch']:.1f} s" if s["t_catch"] is not None else "not caught"
    scene.play(FadeIn(Text(res(pe), font_size=20, color=C_EN).move_to(np.array([-3.4, -3.5, 0]))),
               FadeIn(Text(res(pr), font_size=20, color=C_RL).move_to(np.array([3.4, -3.5, 0]))))
    scene.wait(2.5)
    clear(scene)


def part_numbers(scene):
    head = title_bar("The numbers", "40 random starts near hanging · |τ| ≤ 20 N·m · counted as held if caught and balanced within 30 s")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    n_en, n_rl = RES["nominal"]["energy"], RES["nominal"]["rl"]

    def f(v, unit=""):
        return "—" if v is None else f"{v:g}{unit}"
    rows = [("", "Energy swing-up", "RL swing-up"),
            ("held (of 40)", f"{n_en['held']}", f"{n_rl['held']}"),
            ("median time to catch", f(n_en["median_catch_s"], " s"), f(n_rl["median_catch_s"], " s")),
            ("time at torque limit", f(n_en["saturated_pct"], " %"), f(n_rl["saturated_pct"], " %")),
            ("median elbow work", f(n_en["median_work_J"], " J"), f(n_rl["median_work_J"], " J"))]
    table = VGroup()
    for i, r in enumerate(rows):
        row = VGroup(*[Text(c, font_size=23, weight=BOLD if i == 0 else NORMAL,
                            color=[WHITE, C_EN, C_RL][j] if i == 0 else WHITE) for j, c in enumerate(r)])
        for j, c in enumerate(row):
            c.move_to(np.array([-3.6 + 3.9 * j, 1.6 - 0.75 * i, 0]))
        table.add(row)
    for row in table:
        scene.play(FadeIn(row, shift=UP * 0.1), run_time=0.45)
    better = "RL" if n_rl["held"] > n_en["held"] else ("energy" if n_en["held"] > n_rl["held"] else "neither")
    note = {"RL": "RL catches more starts here, with no physics written into the controller",
            "energy": "the hand-designed law is more reliable here; RL gets close without any equations",
            "neither": "both are equally reliable here"}[better]
    scene.play(FadeIn(Text(note, font_size=20, color=YELLOW).to_edge(DOWN, buff=0.6)))
    scene.wait(4)
    clear(scene)


def part_robust(scene):
    head = title_bar("What if our model is wrong?", "both controllers still assume masses of 1 kg")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    cases = [("mass_minus20", "masses −20 %"), ("nominal", "correct masses"), ("mass_plus20", "masses +20 %")]
    ax_y0, scale = -2.6, 3.4 / 40
    base = Line(np.array([-5.0, ax_y0, 0]), np.array([5.0, ax_y0, 0]), color=GRAY_B)
    scene.play(Create(base))
    bars = VGroup()
    for i, (k, lab) in enumerate(cases):
        x = -3.4 + 3.4 * i
        for j, (ctrl, col) in enumerate((("energy", C_EN), ("rl", C_RL))):
            v = RES[k][ctrl]["held"]
            b = Rectangle(width=0.9, height=max(v * scale, 0.02), fill_color=col, fill_opacity=0.9, stroke_width=0)
            b.move_to(np.array([x + (j - 0.5) * 1.0, ax_y0 + v * scale / 2, 0]))
            t = Text(f"{v}", font_size=18).next_to(b, UP, buff=0.08)
            bars.add(VGroup(b, t))
        bars.add(Text(lab, font_size=19, color=GRAY_B).move_to(np.array([x, ax_y0 - 0.35, 0])))
    leg = VGroup(VGroup(Square(0.25, fill_color=C_EN, fill_opacity=1, stroke_width=0), Text("energy", font_size=17)).arrange(RIGHT, buff=0.12),
                 VGroup(Square(0.25, fill_color=C_RL, fill_opacity=1, stroke_width=0), Text("RL", font_size=17)).arrange(RIGHT, buff=0.12)
                 ).arrange(RIGHT, buff=0.6).next_to(head, DOWN, buff=0.35)
    scene.play(FadeIn(leg), LaggedStart(*[GrowFromEdge(b, DOWN) if isinstance(b, VGroup) and len(b) == 2 else FadeIn(b)
                                          for b in bars], lag_ratio=0.1), run_time=2.5)
    scene.play(FadeIn(Text("· bars = held from 40 random starts", font_size=17, color=GRAY_B).next_to(leg, RIGHT, buff=0.4)))
    scene.wait(2.5)
    en, rl = RES["mass_plus20"]["energy"]["held"], RES["mass_plus20"]["rl"]["held"]
    scene.play(*[FadeOut(m) for m in scene.mobjects if m is not head])
    why = lines([
        (f"heavier links (+20 %): energy law {en}/40,  RL {rl}/40", {"energy law": C_EN, "RL": C_RL}),
        ("", {}),
        ("the energy law is built from physics: it pumps until E = E_top,", {}),
        ("so a heavier arm just takes a little longer", {}),
        ("", {}),
        ("the RL policy only ever practised on 1 kg links:", {}),
        ("it learned the timing of THAT robot, not the physics", {"THAT robot": YELLOW}),
        ("", {}),
        ("can we fix that?", {"can we fix that?": C_OK}),
    ], fs=22).next_to(head, DOWN, buff=0.7)
    for l in why:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.45)
    scene.wait(4)
    clear(scene)


def part_fix(scene):
    head = title_bar("The fix: practise on many robots", "a second RL policy, with only two changes")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    masses = (0.7, 0.85, 1.0, 1.15, 1.3)
    robots = VGroup()
    for i, m in enumerate(masses):
        pv = np.array([-5.2 + 2.6 * i, 2.0, 0])
        q1, q2, sc = 0.35 * (-1) ** i, 0.35 * (-1) ** i, 0.62
        el = pv + sc * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
        mid1 = (pv + el) / 2
        mid2 = el + sc * L2 / 2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
        rad = 0.09 + 0.16 * (m - 0.7) / 0.6              # weight blobs sized by the mass
        body = VGroup(mount(pv), acrobot(pv, q1, q2, sc, width=6),
                      Dot(mid1, radius=rad, color=C_LINK1), Dot(mid2, radius=rad, color=C_LINK2))
        r = VGroup(body, Text(f"{m:g} kg", font_size=19, color=WHITE).move_to(
            np.array([body.get_center()[0], pv[1] - 2.15, 0])))
        robots.add(r)
    scene.play(LaggedStart(*[FadeIn(r, shift=UP * 0.1) for r in robots], lag_ratio=0.15), run_time=1.6)
    rows = [("1.  every practice swing gets NEW random link masses (0.7 – 1.3 kg)", {"NEW random link masses": C_RB}),
            ("→ it cannot memorise the timing of one robot", {}),
            ("2.  one extra input: the missing energy  E_top − E", {"the missing energy": C_E}),
            ("→ the same number the energy law steers by (a hint, not new information)", {}),
            ("", {}),
            ("everything else is unchanged: same network, same reward, same LQR catch", {"unchanged": YELLOW})]
    if TRAIN_R:
        rows.append((f"trained for {TRAIN_R['steps']:,} steps (≈ {TRAIN_R['episodes']:,} practice swings, "
                     f"{TRAIN_R['minutes']:.0f} min on a laptop)", {}))
    txt = lines(rows, fs=19, buff=0.12).to_edge(DOWN, buff=0.4)
    for l, (r, _) in zip(txt, rows):
        if r.startswith("→"):
            l.shift(RIGHT * 0.45)                         # indent the explanation lines
    for l in txt:
        scene.play(FadeIn(l, shift=UP * 0.08), run_time=0.4)
    scene.wait(4)
    clear(scene)


def part_fixed(scene):
    head = title_bar("Did it work?", "held from 40 random starts · every controller still assumes 1 kg links")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    cases = [("mass_minus40", "0.6 kg"), ("mass_minus20", "0.8 kg"), ("nominal", "1 kg"),
             ("mass_plus20", "1.2 kg"), ("mass_plus40", "1.4 kg")]
    ctrls = (("energy", C_EN, "energy law"), ("rl", C_RL, "RL, 1 kg only"), ("rl_robust", C_RB, "RL, random masses"))
    ax_y0, scale, bw = -2.7, 3.0 / 40, 0.5
    shade = VGroup(*[Rectangle(width=2.0, height=3.9, fill_color=GRAY_E, fill_opacity=0.55, stroke_width=0
                               ).move_to(np.array([x, ax_y0 + 1.95, 0])) for x in (-4.8, 4.8)])
    never = VGroup(*[Text("never practised", font_size=15, color=GRAY_B).move_to(np.array([x, ax_y0 + 3.65, 0]))
                     for x in (-4.8, 4.8)])
    base = Line(np.array([-6.0, ax_y0, 0]), np.array([6.0, ax_y0, 0]), color=GRAY_B)
    scene.play(FadeIn(shade), FadeIn(never), Create(base))
    bars = VGroup()
    for i, (k, lab) in enumerate(cases):
        x = -4.8 + 2.4 * i
        for j, (ctrl, col, _) in enumerate(ctrls):
            v = RES[k][ctrl]["held"]
            b = Rectangle(width=bw, height=max(v * scale, 0.02), fill_color=col, fill_opacity=0.9, stroke_width=0)
            b.move_to(np.array([x + (j - 1) * (bw + 0.06), ax_y0 + v * scale / 2, 0]))
            bars.add(VGroup(b, Text(f"{v}", font_size=15).next_to(b, UP, buff=0.06)))
        bars.add(Text(lab, font_size=18, color=GRAY_B).move_to(np.array([x, ax_y0 - 0.32, 0])))
    leg = VGroup(*[VGroup(Square(0.22, fill_color=c, fill_opacity=1, stroke_width=0), Text(n, font_size=16)
                          ).arrange(RIGHT, buff=0.1) for _, c, n in ctrls]).arrange(RIGHT, buff=0.5).next_to(head, DOWN, buff=0.25)
    scene.play(FadeIn(leg), LaggedStart(*[GrowFromEdge(b, DOWN) if isinstance(b, VGroup) and len(b) == 2 else FadeIn(b)
                                          for b in bars], lag_ratio=0.06), run_time=3)
    scene.wait(4)
    scene.play(*[FadeOut(m) for m in scene.mobjects if m is not head])
    p20, n0, p40 = RES["mass_plus20"], RES["nominal"], RES["mass_plus40"]
    m40 = RES["mass_minus40"]
    txt = lines([
        (f"heavier links (+20 %):  RL went from {p20['rl']['held']}/40  to  {p20['rl_robust']['held']}/40",
         {f"{p20['rl_robust']['held']}/40": C_OK}),
        (f"correct robot: still {n0['rl_robust']['held']}/40, same {n0['rl_robust']['median_catch_s']:g} s catch, "
         f"less elbow work", {}),
        (f"lighter than ever practised (0.6 kg):  {m40['rl_robust']['held']}/40", {}),
        ("", {}),
        (f"but much heavier than ever practised (1.4 kg):  RL {p40['rl_robust']['held']}/40,  "
         f"energy law {p40['energy']['held']}/40", {"but": C_BAD}),
        ("it still reaches the top, but keeps arriving where the LQR cannot hold it", {}),
        ("(its reward stopped at 'reach the catch zone', it never practised the hand-over)", {}),
        ("", {}),
        ("lesson: RL is reliable for the robots it practised on;", {"lesson:": YELLOW}),
        ("physics knows WHY it works, so it reaches further", {"WHY": YELLOW}),
    ], fs=20).next_to(head, DOWN, buff=0.55)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.42)
    scene.wait(5)
    clear(scene)


def part_takeaways(scene):
    head = title_bar("So which one?")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    a = card("Choose the energy swing-up when", [
        ("you have a good model of the robot", {}),
        ("you must explain or prove why it works", {}),
        ("you want something you can tune by hand", {}),
    ], C_EN, 5.6)
    b = card("Choose RL when", [
        ("the model is hard to write down", {}),
        ("a simulator exists to practise in", {}),
        ("you can test it a lot before trusting it", {}),
        ("and train it on many versions of the robot", {"many versions": C_RB}),
    ], C_RL, 5.6)
    VGroup(a, b).arrange(RIGHT, buff=0.5).next_to(head, DOWN, buff=0.6)
    scene.play(FadeIn(a, shift=UP * 0.2), FadeIn(b, shift=UP * 0.2))
    t = lines([
        ("Best of both, as built here: learn the hard nonlinear part (swing-up),", {}),
        ("keep the part we can prove (the LQR catch near the top).", {"keep the part we can prove": C_OK}),
    ], fs=21).to_edge(DOWN, buff=0.7)
    scene.play(FadeIn(t))
    scene.wait(5)


class RLvsEnergy(Scene):
    def construct(self):
        part_intro(self)
        part_methods(self)
        part_learning(self)
        part_race(self)
        part_numbers(self)
        part_robust(self)
        part_fix(self)
        part_fixed(self)
        part_takeaways(self)
