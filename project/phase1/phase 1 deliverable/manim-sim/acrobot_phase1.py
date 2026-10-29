"""
Acrobot Project - Phase 1 animation (Manim Community Edition)
==============================================================

Scenes (render one at a time, or all together with AcrobotPhase1):

    manim -pqh acrobot_phase1.py ModelScene          # 1. the system and the model
    manim -pqh acrobot_phase1.py EquilibriaScene     # 2. finding the tau = 0 equilibria
    manim -pqh acrobot_phase1.py LinearizationScene  # 3. linear vs nonlinear near E1 and E3
    manim -pqh acrobot_phase1.py TransferScene       # 4. poles and zeros of G1, G2
    manim -pqh acrobot_phase1.py AcrobotPhase1       # everything in one video

Use -pql for a fast low-quality preview, -pqk for 4K.

Only `Text` is used (no LaTeX needed). All numbers are computed here from the
handout parameters with the same formulas as the report, so the video always
matches the report (P, r, poles, zeros, simulation).
"""

import numpy as np
from manim import *

# ---------------------------------------------------------------------------
# Model (handout parameters) - same equations as the report
# ---------------------------------------------------------------------------
L1, L2 = 1.0, 2.0            # link lengths [m]
G = 9.81

EQUILIBRIA = {               # tau* = 0 equilibria (q1*, q2*)
    "E₁": (0.0, 0.0),
    "E₂": (0.0, np.pi),
    "E₃": (np.pi, 0.0),
    "E₄": (np.pi, np.pi),
}
EQ_NAMES = {"E₁": "down–down", "E₂": "down–up", "E₃": "up–up (target)", "E₄": "up–down"}


def mass_matrix(q2):
    c = np.cos(q2)
    return np.array([[2.663 + 2 * c, 1.33 + c], [1.33 + c, 1.33]])


def potential(q1, q2):
    return -14.715 * np.cos(q1) - 9.81 * np.cos(q1 + q2)


def f_nonlinear(x, tau=0.0):
    """x = [q1, q2, dq1, dq2];  M qdd + C qd = tau_g + B tau."""
    q1, q2, d1, d2 = x
    s = np.sin(q2)
    M = mass_matrix(q2)
    C = np.array([[-2 * s * d2, -s * d2], [s * d1, 0.0]])
    tg = -G * np.array([1.5 * np.sin(q1) + np.sin(q1 + q2), np.sin(q1 + q2)])
    dd = np.linalg.solve(M, tg + np.array([0.0, tau]) - C @ np.array([d1, d2]))
    return np.array([d1, d2, dd[0], dd[1]])


def linearize(q1s, q2s):
    """P = M^-1 K, r = M^-1 B, A = [0 I; P 0]."""
    M = mass_matrix(q2s)
    cp = np.cos(q1s + q2s)
    K = -G * np.array([[1.5 * np.cos(q1s) + cp, cp], [cp, cp]])
    P = np.linalg.solve(M, K)
    r = np.linalg.solve(M, np.array([0.0, 1.0]))
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [P, np.zeros((2, 2))]])
    return A, P, r


def transfer_data(q1s, q2s):
    """Delta(s) = s^4 - tr(P) s^2 + det P,  N1 = r1 s^2 + c1,  N2 = r2 s^2 + c2."""
    _, P, r = linearize(q1s, q2s)
    tr, det = np.trace(P), np.linalg.det(P)
    c1 = P[0, 1] * r[1] - P[1, 1] * r[0]
    c2 = P[1, 0] * r[0] - P[0, 0] * r[1]
    lam = np.roots([1.0, -tr, det]).astype(complex)          # lambda = s^2
    poles = np.concatenate([np.sqrt(lam), -np.sqrt(lam)])
    z1 = np.sqrt(complex(-c1 / r[0]))
    z2 = np.sqrt(complex(-c2 / r[1]))
    return dict(tr=tr, det=det, r=r, c1=c1, c2=c2, poles=poles,
                zeros1=np.array([z1, -z1]), zeros2=np.array([z2, -z2]))


def rk4(fun, x0, T, n):
    h = T / n
    xs = [np.asarray(x0, float)]
    for _ in range(n):
        x = xs[-1]
        k1 = fun(x); k2 = fun(x + h / 2 * k1); k3 = fun(x + h / 2 * k2); k4 = fun(x + h * k3)
        xs.append(x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
    return np.linspace(0, T, n + 1), np.array(xs)


def fmt(v, nd=4):
    return f"{v:+.{nd}f}".replace("+", "+ ").replace("-", "− ")


# ---------------------------------------------------------------------------
# Colours / drawing helpers
# ---------------------------------------------------------------------------
C_LINK1 = "#2E86DE"
C_LINK2 = "#E67E22"
C_TAU = "#E74C3C"
C_COM = "#F1C40F"
C_OK = "#2ECC71"
C_GHOST = WHITE


def tip_positions(pivot, q1, q2, scale):
    elbow = pivot + scale * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    tip = elbow + scale * L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
    return elbow, tip


def com_positions(pivot, q1, q2, scale):
    phi = q1 + q2
    c1 = pivot + scale * 0.5 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    c2 = pivot + scale * np.array([np.sin(q1) + np.sin(phi), -np.cos(q1) - np.cos(phi), 0.0])
    total = (c1 + c2) / 2                                      # m1 = m2
    return c1, c2, total


def acrobot_static(pivot, q1, q2, scale, width=6, opacity=1.0, ghost=False, show_com=True):
    """One acrobot drawing as a VGroup (rebuilt each frame inside always_redraw)."""
    elbow, tip = tip_positions(pivot, q1, q2, scale)
    col1 = C_GHOST if ghost else C_LINK1
    col2 = C_GHOST if ghost else C_LINK2
    g = VGroup(
        Line(pivot, elbow, color=col1, stroke_width=width * 1.5, stroke_opacity=opacity),
        Line(elbow, tip, color=col2, stroke_width=width * 0.8, stroke_opacity=opacity),
        Dot(elbow, radius=0.045 + 0.01 * width / 6, color=WHITE, fill_opacity=opacity),
    )
    if show_com and not ghost:
        c1, c2, _ = com_positions(pivot, q1, q2, scale)
        g.add(Dot(c1, radius=0.04, color=C_COM), Dot(c2, radius=0.04, color=C_COM))
    return g


def ceiling(pivot, half=0.55):
    bar = Line(pivot + LEFT * half, pivot + RIGHT * half, color=GRAY_B, stroke_width=3)
    hatch = VGroup(*[
        Line(pivot + RIGHT * x, pivot + RIGHT * (x + 0.12) + UP * 0.14, color=GRAY_C, stroke_width=1.5)
        for x in np.linspace(-half, half - 0.12, 8)
    ])
    return VGroup(bar, hatch, Dot(pivot, radius=0.07, color=WHITE))


def pivot_mount(pivot):
    """Small pivot marker that does not clash with links pointing upward."""
    return VGroup(Dot(pivot, radius=0.075, color=WHITE),
                  Circle(radius=0.12, color=GRAY_B, stroke_width=2).move_to(pivot))


def title_bar(text, sub=None):
    t = Text(text, font_size=34, weight=BOLD).to_edge(UP, buff=0.3)
    grp = VGroup(t)
    if sub:
        s = Text(sub, font_size=20, color=GRAY_B).next_to(t, DOWN, buff=0.12)
        grp.add(s)
    return grp


def step_lines(lines, font_size=22, buff=0.22):
    """One Text per line; an empty string becomes an invisible spacer."""
    rows = []
    for l, c in lines:
        if l == "":
            rows.append(Line(ORIGIN, UP * font_size / 160, stroke_opacity=0))
        else:
            rows.append(Text(l, font_size=font_size, t2c=c))
    return VGroup(*rows).arrange(DOWN, aligned_edge=LEFT, buff=buff)


# ---------------------------------------------------------------------------
# Scene parts (each is a function so the combined scene can reuse them)
# ---------------------------------------------------------------------------
def part_model(scene: Scene):
    head = title_bar("Phase 1 · The Acrobot", "two links, motor only at the elbow")
    scene.play(FadeIn(head, shift=DOWN * 0.2))

    pivot = np.array([-3.4, 1.7, 0.0])
    scale = 1.0
    Q1, Q2 = 0.6, 0.9                      # pose used for the labelled sketch
    k = ValueTracker(0)                     # frame index into the simulated swing

    # Real free swing of the nonlinear model (tau = 0), used after the labels go
    T_SWING = 6.0
    t_sw, x_sw = rk4(f_nonlinear, np.array([Q1, Q2, 0.0, 0.0]), T_SWING, 720)

    def pose():
        i = int(round(k.get_value()))
        return x_sw[i, 0], x_sw[i, 1]

    robot = always_redraw(lambda: acrobot_static(pivot, *pose(), scale, width=8))
    vert = DashedLine(pivot, pivot + DOWN * 3.3, color=GRAY_C, stroke_width=2)

    # ---- static labels, placed so they never sit on a link ----
    elbow, tip = tip_positions(pivot, Q1, Q2, scale)
    a_link1_back = np.arctan2(*(pivot - elbow)[[1, 0]])        # elbow -> shoulder
    a_link2 = np.arctan2(*(tip - elbow)[[1, 0]])               # elbow -> tip
    a_ext = a_link1_back - PI                                  # extension of link 1

    q1_arc = Arc(radius=0.62, start_angle=-PI / 2, angle=Q1, arc_center=pivot, color=WHITE, stroke_width=3)
    q1_lab = Text("q₁", font_size=24).move_to(pivot + 0.9 * np.array([np.sin(Q1 / 2), -np.cos(Q1 / 2), 0]))

    ext = DashedLine(elbow, elbow + 0.9 * np.array([np.cos(a_ext), np.sin(a_ext), 0]), color=GRAY_C, stroke_width=2)
    q2_arc = Arc(radius=0.62, start_angle=a_ext, angle=a_link2 - a_ext, arc_center=elbow, color=WHITE, stroke_width=3)
    mid2 = (a_ext + a_link2) / 2
    q2_lab = Text("q₂", font_size=24).move_to(elbow + 0.92 * np.array([np.cos(mid2), np.sin(mid2), 0]))

    # tau: short curved arrow on the free (outer) side of the elbow
    free = (a_link1_back + a_link2) / 2 + PI
    tau_arc = Arc(radius=0.36, start_angle=free - 0.9, angle=1.8, arc_center=elbow, color=C_TAU, stroke_width=4)
    tau_arc.add_tip(tip_length=0.13, tip_width=0.13)
    tau_lab = Text("τ", font_size=26, color=C_TAU).move_to(elbow + 0.68 * np.array([np.cos(free), np.sin(free), 0]))
    labels = VGroup(q1_arc, q1_lab, ext, q2_arc, q2_lab, tau_arc, tau_lab)

    note = Text("passive shoulder (no motor)", font_size=18, color=GRAY_B).next_to(pivot, UP, buff=0.3)

    scene.play(FadeIn(ceiling(pivot)), Create(vert), FadeIn(note))
    scene.play(Create(robot), run_time=1.0)
    scene.play(FadeIn(labels), run_time=0.8)

    info = step_lines([
        ("State  x = [q₁, q₂, q̇₁, q̇₂]ᵀ", {"x": YELLOW}),
        ("Input  τ = elbow motor torque,  |τ| ≤ 20 N·m", {"τ": C_TAU}),
        ("m₁ = m₂ = 1 kg,  l₁ = 1 m,  l₂ = 2 m", {}),
        ("I₁ = 0.083,  I₂ = 0.33 kg·m²", {}),
        ("", {}),
        ("M(q) q̈ + C(q,q̇) q̇ = τ_g(q) + B τ", {"τ": C_TAU}),
        ("B = [0, 1]ᵀ   (torque acts on the elbow)", {}),
        ("", {}),
        ("M₁₁ = 2.663 + 2 cos q₂", {}),
        ("M₁₂ = 1.33 + cos q₂,   M₂₂ = 1.33", {}),
        ("U = −14.715 cos q₁ − 9.81 cos(q₁+q₂)", {}),
        ("τ_g = −∂U/∂q", {}),
    ], font_size=21, buff=0.16).to_edge(RIGHT, buff=0.5).shift(DOWN * 0.25)
    for line in info:
        scene.play(FadeIn(line, shift=LEFT * 0.15), run_time=0.4)
    scene.wait(1.5)

    # ---- release it: real nonlinear free swing, labels removed ----
    swing_note = Text("released from rest, τ = 0  (nonlinear model, real time)",
                      font_size=18, color=YELLOW).to_edge(DOWN, buff=0.3).shift(LEFT * 3)
    scene.play(FadeOut(labels), FadeIn(swing_note), run_time=0.6)
    scene.play(k.animate.set_value(len(t_sw) - 1), run_time=T_SWING, rate_func=linear)
    scene.wait(1.0)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def part_equilibria(scene: Scene):
    head = title_bar("Finding the equilibria", "motor torque τ* = 0 (course specification)")
    scene.play(FadeIn(head, shift=DOWN * 0.2))

    steps = step_lines([
        ("1.  At rest:   q̇ = 0  and  q̈ = 0", {"q̇ = 0": YELLOW, "q̈ = 0": YELLOW}),
        ("2.  Model becomes   τ_g(q*) = 0", {"τ_g(q*) = 0": YELLOW}),
        ("     → gravity gives no torque about either joint", {}),
        ("3.  Row 2:   sin(q₁* + q₂*) = 0", {}),
        ("     Row 1:   1.5 sin q₁* + sin(q₁* + q₂*) = 0", {}),
        ("     ⇒  sin q₁* = 0   ⇒   q₁* ∈ {0, π}", {"q₁* ∈ {0, π}": C_OK}),
        ("     ⇒  q₁* + q₂* ∈ {0, π}  ⇒   q₂* ∈ {0, π}", {"q₂* ∈ {0, π}": C_OK}),
        ("4.  2 × 2 combinations  ⇒  exactly 4 equilibria", {"4 equilibria": C_OK}),
    ], font_size=24, buff=0.24).next_to(head, DOWN, buff=0.55)
    for s in steps:
        scene.play(FadeIn(s, shift=RIGHT * 0.15), run_time=0.6)
        scene.wait(0.35)
    scene.wait(1.2)
    scene.play(FadeOut(steps))

    # 2 x 2 grid of the four postures
    scale = 0.6
    centers = [np.array([-5.0, -0.5, 0]), np.array([-1.7, -0.5, 0]),
               np.array([1.7, -0.5, 0]), np.array([5.0, -0.5, 0])]
    cards = VGroup()
    for (name, (a, b)), c in zip(EQUILIBRIA.items(), centers):
        pivot = c + UP * 0.1
        vert = DashedLine(pivot + UP * 1.95, pivot + DOWN * 1.95, color=GRAY_D, stroke_width=1.5)
        rob = acrobot_static(pivot, a, b, scale, width=7)
        _, _, com = com_positions(pivot, a, b, scale)
        com_dot = Dot(com, radius=0.07, color=C_OK)
        lab = Text(name, font_size=30, weight=BOLD).move_to(c + UP * 2.45)
        sub = Text(EQ_NAMES[name], font_size=17, color=GRAY_B).next_to(lab, DOWN, buff=0.08)
        qtxt = Text(f"(q₁*, q₂*) = ({'0' if a == 0 else 'π'}, {'0' if b == 0 else 'π'})", font_size=17).move_to(c + DOWN * 2.3)
        utxt = Text(f"U = {potential(a, b):+.3f} J", font_size=17, color=GRAY_B).next_to(qtxt, DOWN, buff=0.08)
        cards.add(VGroup(vert, rob, pivot_mount(pivot), com_dot, lab, sub, qtxt, utxt))

    for card in cards:
        scene.play(FadeIn(card, shift=UP * 0.2), run_time=0.8)
    note = Text("green dot = total centre of mass: always on the vertical through the pivot",
                font_size=19, color=C_OK).to_edge(DOWN, buff=0.25)
    scene.play(FadeIn(note))
    scene.wait(1.5)

    extra = Text("With a nonzero constant torque further equilibria exist;\n"
                 "following the course specification we consider τ = 0.",
                 font_size=19, color=GRAY_B).to_edge(DOWN, buff=0.25)
    scene.play(ReplacementTransform(note, extra))
    scene.wait(2)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def part_linearization_math(scene: Scene):
    head = title_bar("Linearization", "first-order Taylor expansion around each equilibrium")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    A, P, r = linearize(0.0, 0.0)
    lines = step_lines([
        ("Small deviations:   x̃ = x − x*,   τ̃ = τ − τ* = τ", {}),
        ("ẋ̃  ≈  (∂f/∂x) x̃ + (∂f/∂τ) τ̃  =  A x̃ + B τ̃", {"A": YELLOW, "B": YELLOW}),
        ("", {}),
        ("• Coriolis term C q̇ is quadratic in q̇  →  derivative 0 at q̇* = 0", {}),
        ("• τ_g(q*) = 0  →  only  K = ∂τ_g/∂q  survives", {}),
        ("", {}),
        ("A = [ 0   I ;  P   0 ],     B = [ 0 ;  r ],     y = [q̃₁, q̃₂]ᵀ", {"A": YELLOW, "B": YELLOW}),
        ("P = M(q*)⁻¹ K(q*),     r = M(q*)⁻¹ [0, 1]ᵀ", {}),
        ("", {}),
        (f"E₁:  P = [{P[0,0]:.4f}  {P[0,1]:.4f} ;  {P[1,0]:.4f}  {P[1,1]:.4f}]", {"E₁": C_OK}),
        (f"       r = [{r[0]:.4f} ;  {r[1]:.4f}]", {}),
        ("E₃:  same M, gravity flips  →  P(E₃) = −P(E₁),  same r", {"E₃": C_OK}),
    ], font_size=23, buff=0.2).next_to(head, DOWN, buff=0.5)
    for l in lines:
        scene.play(FadeIn(l, shift=RIGHT * 0.15), run_time=0.5)
    scene.wait(2.5)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def part_linear_vs_nonlinear(scene: Scene):
    head = title_bar("Behaviour near each equilibrium",
                     "small push, τ = 0:   nonlinear model (colour)  vs  linear model (white)")
    scene.play(FadeIn(head, shift=DOWN * 0.2))

    T_RUN = 6.0                            # every panel runs in real time for 6 s
    cases = [
        dict(name="E₁", d0=(0.15, -0.15), T=T_RUN),
        dict(name="E₂", d0=(0.05, 0.0), T=T_RUN),
        dict(name="E₃", d0=(0.05, 0.0), T=T_RUN),
        dict(name="E₄", d0=(0.05, 0.0), T=T_RUN),
    ]
    centres = [np.array([-3.6, 0.95, 0]), np.array([3.6, 0.95, 0]),
               np.array([-3.6, -2.45, 0]), np.array([3.6, -2.45, 0])]
    scale = 0.38
    u = ValueTracker(0.0)                  # normalised time 0..1 for all panels
    static = VGroup()
    dynamic = []
    for c, ctr in zip(cases, centres):
        qs = EQUILIBRIA[c["name"]]
        xs = np.array([*qs, 0.0, 0.0])
        d0 = np.array([*c["d0"], 0.0, 0.0])
        A, _, _ = linearize(*qs)
        t, nl = rk4(f_nonlinear, xs + d0, c["T"], 900)
        _, li = rk4(lambda x, A=A: A @ x, d0, c["T"], 900)
        nl = nl - xs
        ymax = min(200.0, 1.15 * np.degrees(np.abs(nl[:, 0]).max()))
        ymax = float(np.ceil(ymax / 10) * 10)

        pivot = ctr + LEFT * 2.25 + DOWN * 0.15
        stable = c["name"] == "E₁"
        lab = Text(f"near {c['name']}  ({EQ_NAMES[c['name']]})", font_size=20,
                   color=C_OK if stable else ORANGE).move_to(ctr + UP * 1.45)
        tag = Text("oscillates" if stable else "falls away (unstable)", font_size=15,
                   color=GRAY_B).next_to(lab, DOWN, buff=0.06)
        mount = ceiling(pivot, 0.28) if qs[0] == 0 and qs[1] == 0 else pivot_mount(pivot)

        ax = Axes(x_range=[0, c["T"], c["T"] / 4], y_range=[-ymax, ymax, ymax / 2],
                  x_length=3.6, y_length=1.9, tips=False,
                  axis_config=dict(stroke_width=1.5, color=GRAY_B)).move_to(ctr + RIGHT * 1.2 + DOWN * 0.2)
        ylab = Text(f"q̃₁ [deg]  (±{ymax:g})", font_size=13, color=GRAY_B).next_to(ax, UP, buff=0.04).align_to(ax, LEFT)
        xlab = Text(f"t: 0 → {c['T']:g} s", font_size=13, color=GRAY_B).next_to(ax, DOWN, buff=0.05)
        static.add(VGroup(lab, tag, mount, ax, ylab, xlab))

        def robots(t=t, nl=nl, li=li, qs=qs, pivot=pivot):
            i = int(round(u.get_value() * (len(t) - 1)))
            qn = np.array(qs) + nl[i, :2]
            ql = np.array(qs) + li[i, :2]
            g = VGroup()
            if np.abs(li[i, :2]).max() < 0.6:      # linear model only meaningful for small deviations
                g.add(acrobot_static(pivot, ql[0], ql[1], scale, width=4, opacity=0.5, ghost=True))
            g.add(acrobot_static(pivot, qn[0], qn[1], scale, width=6, show_com=False))
            return g

        def curve(t=t, data=nl, ax=ax, col=C_LINK1, w=3.0, ymax=ymax):
            i = max(2, int(round(u.get_value() * (len(t) - 1))))
            ys = np.degrees(data[:i, 0])
            out = np.where(np.abs(ys) > ymax)[0]
            if len(out):                            # stop the curve where it leaves the plot
                i = max(2, out[0]); ys = ys[:i]
            return VMobject(stroke_color=col, stroke_width=w).set_points_as_corners(
                [ax.c2p(tt, yy) for tt, yy in zip(t[:i], ys)])

        dynamic += [always_redraw(robots),
                    always_redraw(lambda t=t, li=li, ax=ax, ymax=ymax: curve(t, li, ax, WHITE, 2.0, ymax)),
                    always_redraw(lambda t=t, nl=nl, ax=ax, ymax=ymax: curve(t, nl, ax, C_LINK1, 3.0, ymax))]

    scene.play(FadeIn(static))
    scene.add(*dynamic)
    scene.wait(0.6)
    scene.play(u.animate.set_value(1.0), run_time=T_RUN, rate_func=linear)   # real time

    res = Text("near the equilibrium the linear model follows the nonlinear one:  "
               "E₁ oscillates,  E₂, E₃, E₄ move away",
               font_size=19, color=YELLOW).move_to(head[1])
    scene.play(ReplacementTransform(head[1], res))
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def part_transfer(scene: Scene):
    head = title_bar("Transfer functions  τ → q₁,  τ → q₂", "(s²I − P) Q̃(s) = r T̃(s)")
    scene.play(FadeIn(head, shift=DOWN * 0.2))

    deriv = step_lines([
        ("G₁(s) = [r₁ s² + (P₁₂r₂ − P₂₂r₁)] / Δ(s)", {}),
        ("G₂(s) = [r₂ s² + (P₂₁r₁ − P₁₁r₂)] / Δ(s)", {}),
        ("Δ(s) = s⁴ − tr(P) s² + det P", {"Δ(s)": YELLOW}),
        ("poles: λ = s² from a quadratic, s = ±√λ", {}),
    ], font_size=21, buff=0.16).to_corner(UL, buff=0.4).shift(DOWN * 1.15)
    scene.play(FadeIn(deriv))

    ax = Axes(x_range=[-7, 7, 1], y_range=[-7, 7, 1], x_length=5.6, y_length=5.6, tips=False,
              axis_config=dict(stroke_width=1.5, color=GRAY_B)).to_edge(RIGHT, buff=0.6).shift(DOWN * 0.45)
    re_l = Text("Re s", font_size=16, color=GRAY_B).next_to(ax.x_axis, RIGHT, buff=0.05).shift(UP * 0.2)
    im_l = Text("Im s", font_size=16, color=GRAY_B).next_to(ax.y_axis, UP, buff=0.05)
    legend = VGroup(
        VGroup(Text("×", font_size=26, color=C_TAU), Text("poles", font_size=16)).arrange(RIGHT, buff=0.1),
        VGroup(Circle(radius=0.08, color=C_LINK1, stroke_width=3), Text("zeros G₁", font_size=16)).arrange(RIGHT, buff=0.1),
        VGroup(Circle(radius=0.08, color=C_LINK2, stroke_width=3), Text("zeros G₂", font_size=16)).arrange(RIGHT, buff=0.1),
    ).arrange(RIGHT, buff=0.35).next_to(ax, DOWN, buff=0.15)
    scene.play(Create(ax), FadeIn(re_l), FadeIn(im_l), FadeIn(legend))

    def cross(p, col=C_TAU, s=0.11):
        return VGroup(Line(p + np.array([-s, -s, 0]), p + np.array([s, s, 0]), color=col, stroke_width=4),
                      Line(p + np.array([-s, s, 0]), p + np.array([s, -s, 0]), color=col, stroke_width=4))

    tf_box = None
    marks = None
    for name, (a, b) in EQUILIBRIA.items():
        d = transfer_data(a, b)
        r1, r2 = d["r"]
        txt = step_lines([
            (f"{name}  ({EQ_NAMES[name]})", {name: C_OK}),
            (f"Δ(s) = s⁴ {fmt(-d['tr'])} s² {fmt(d['det'])}", {}),
            (f"G₁(s) = {r1:.4f} (s² {fmt(d['c1'] / r1)}) / Δ(s)", {}),
            (f"G₂(s) = {r2:.4f} (s² {fmt(d['c2'] / r2)}) / Δ(s)", {}),
            ("poles: " + ",  ".join(sorted({_cstr(p) for p in d["poles"]})), {}),
        ], font_size=20, buff=0.14).next_to(deriv, DOWN, buff=0.55, aligned_edge=LEFT)

        new_marks = VGroup()
        for p in d["poles"]:
            new_marks.add(cross(ax.c2p(p.real, p.imag)))
        for z in d["zeros1"]:
            new_marks.add(Circle(radius=0.1, color=C_LINK1, stroke_width=3).move_to(ax.c2p(z.real, z.imag)))
        for z in d["zeros2"]:
            new_marks.add(Circle(radius=0.15, color=C_LINK2, stroke_width=3).move_to(ax.c2p(z.real, z.imag)))

        if tf_box is None:
            scene.play(FadeIn(txt), FadeIn(new_marks))
        else:
            scene.play(ReplacementTransform(tf_box, txt), ReplacementTransform(marks, new_marks), run_time=1.2)
        tf_box, marks = txt, new_marks
        scene.wait(2.2)

    note = Text("No friction ⇒ only even powers of s\n⇒ poles and zeros come in pairs ±p",
                font_size=19, color=YELLOW).next_to(tf_box, DOWN, buff=0.5, aligned_edge=LEFT)
    scene.play(FadeIn(note))
    scene.wait(2.5)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def _cstr(p):
    if abs(p.imag) < 1e-9:
        return f"±{abs(p.real):.4f}"
    return f"±j{abs(p.imag):.4f}"


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------
class ModelScene(Scene):
    def construct(self):
        part_model(self)


class EquilibriaScene(Scene):
    def construct(self):
        part_equilibria(self)


class LinearizationScene(Scene):
    def construct(self):
        part_linearization_math(self)
        part_linear_vs_nonlinear(self)


class TransferScene(Scene):
    def construct(self):
        part_transfer(self)


class AcrobotPhase1(Scene):
    def construct(self):
        intro = VGroup(
            Text("Acrobot · Phase 1", font_size=48, weight=BOLD),
            Text("Modelling, Equilibria and Linearization", font_size=28, color=GRAY_B),
            Text("Bhargav · Sreekar · Kaushik · Divijat", font_size=22, color=GRAY_C),
        ).arrange(DOWN, buff=0.3)
        self.play(FadeIn(intro, shift=UP * 0.2))
        self.wait(1.5)
        self.play(FadeOut(intro))
        part_model(self)
        part_equilibria(self)
        part_linearization_math(self)
        part_linear_vs_nonlinear(self)
        part_transfer(self)
        end = Text("Four equilibria · linear models · transfer functions  →  Phase 2",
                   font_size=26)
        self.play(FadeIn(end))
        self.wait(2)
