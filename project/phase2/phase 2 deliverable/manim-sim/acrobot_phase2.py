"""
Acrobot Project - Phase 2 animation (Manim Community Edition)
==============================================================

Stability of every equilibrium (transfer functions + state space), simulation
evidence from the PyBullet model, and single-angle feedback at the upright.

    manim -pql acrobot_phase2.py AcrobotPhase2                     # quick preview
    manim -r 2560,1440 --fps 60 acrobot_phase2.py AcrobotPhase2    # 2K final

Individual scenes: CriteriaScene, EnergyScene, OpenLoopScene, PDScene, ObserverScene.

All trajectories come from phase2_data.npz and all quoted numbers from
../code/results.json, both written by ../code/make_phase2_results.py from the
PyBullet simulator, so the video always matches the report. Only `Text` is used
(no LaTeX needed).
"""

import json
import os

import numpy as np
from manim import *

HERE = os.path.dirname(os.path.abspath(__file__))
D = np.load(os.path.join(HERE, "phase2_data.npz"))
with open(os.path.join(HERE, "..", "code", "results.json")) as fh:
    RES = json.load(fh)

# ---------------------------------------------------------------------------
# Model (same as the report)
# ---------------------------------------------------------------------------
L1, L2 = 1.0, 2.0
G = 9.81
EQUILIBRIA = {"E₁": (0.0, 0.0), "E₂": (0.0, np.pi), "E₃": (np.pi, 0.0), "E₄": (np.pi, np.pi)}
EQ_NAMES = {"E₁": "down–down", "E₂": "down–up", "E₃": "up–up (upright)", "E₄": "up–down"}


def mass_matrix(q2):
    c = np.cos(q2)
    return np.array([[2.663 + 2 * c, 1.33 + c], [1.33 + c, 1.33]])


def linearize(q1s, q2s):
    M = mass_matrix(q2s)
    cp = np.cos(q1s + q2s)
    K = -G * np.array([[1.5 * np.cos(q1s) + cp, cp], [cp, cp]])
    P = np.linalg.solve(M, K)
    r = np.linalg.solve(M, np.array([0.0, 1.0]))
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [P, np.zeros((2, 2))]])
    B = np.r_[0.0, 0.0, r].reshape(4, 1)
    return A, B, P, K


def potential(q1, q2):
    return -14.715 * np.cos(q1) - 9.81 * np.cos(q1 + q2)


# ---------------------------------------------------------------------------
# Colours / drawing helpers (same look as the Phase 1 video)
# ---------------------------------------------------------------------------
C_LINK1 = "#2E86DE"
C_LINK2 = "#E67E22"
C_TAU = "#E74C3C"
C_OK = "#2ECC71"
C_BAD = "#E74C3C"
C_WARN = "#F1C40F"
C_GHOST = WHITE


def tip_positions(pivot, q1, q2, scale):
    elbow = pivot + scale * L1 * np.array([np.sin(q1), -np.cos(q1), 0.0])
    tip = elbow + scale * L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2), 0.0])
    return elbow, tip


def acrobot_static(pivot, q1, q2, scale, width=6, opacity=1.0, ghost=False):
    elbow, tip = tip_positions(pivot, q1, q2, scale)
    col1 = C_GHOST if ghost else C_LINK1
    col2 = C_GHOST if ghost else C_LINK2
    return VGroup(
        Line(pivot, elbow, color=col1, stroke_width=width * 1.5, stroke_opacity=opacity),
        Line(elbow, tip, color=col2, stroke_width=width * 0.8, stroke_opacity=opacity),
        Dot(elbow, radius=0.045 + 0.01 * width / 6, color=WHITE, fill_opacity=opacity),
    )


def torque_arc(pivot, q1, q2, scale, tau, radius=0.3):
    """Arc at the elbow whose length is proportional to |tau| / 20."""
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


def step_lines(lines, font_size=22, buff=0.22):
    rows = []
    for l, c in lines:
        if l == "":
            rows.append(Line(ORIGIN, UP * font_size / 160, stroke_opacity=0))
        else:
            rows.append(Text(l, font_size=font_size, t2c=c))
    return VGroup(*rows).arrange(DOWN, aligned_edge=LEFT, buff=buff)


def cross(p, col=C_TAU, s=0.11, w=4):
    return VGroup(Line(p + np.array([-s, -s, 0]), p + np.array([s, s, 0]), color=col, stroke_width=w),
                  Line(p + np.array([-s, s, 0]), p + np.array([s, -s, 0]), color=col, stroke_width=w))


def splane(lim=7.0, size=4.6):
    ax = Axes(x_range=[-lim, lim, lim / 3.5], y_range=[-lim, lim, lim / 3.5], x_length=size, y_length=size,
              tips=False, axis_config=dict(stroke_width=1.5, color=GRAY_B))
    rhp = Polygon(ax.c2p(0, -lim), ax.c2p(lim, -lim), ax.c2p(lim, lim), ax.c2p(0, lim),
                  stroke_width=0, fill_color=C_BAD, fill_opacity=0.12)
    lab = VGroup(Text("Re s", font_size=15, color=GRAY_B).next_to(ax.x_axis, RIGHT, buff=0.05),
                 Text("Im s", font_size=15, color=GRAY_B).next_to(ax.y_axis, UP, buff=0.05),
                 Text("unstable half-plane", font_size=13, color=C_BAD).move_to(ax.c2p(lim * 0.5, lim * 0.85)))
    return ax, VGroup(rhp, ax, lab)


def series(prefix, key):
    return D[f"{prefix}_{key}"]


def sample(prefix, key, u):
    """Value of a logged signal at normalised time u in [0, 1]."""
    arr = series(prefix, key)
    i = int(round(u * (len(arr) - 1)))
    return float(arr[i])


def curve_upto(ax, t, y, u, color, width=3.0, ymax=None, dashed=False):
    i = max(2, int(round(u * (len(t) - 1))))
    ys = np.asarray(y[:i], float)
    ts = np.asarray(t[:i], float)
    ok = np.isfinite(ys)
    if ymax is not None:
        out = np.where(np.abs(ys) > ymax)[0]
        if len(out):
            ok[out[0]:] = False
    ts, ys = ts[ok], ys[ok]
    if len(ts) < 2:
        return VMobject()
    # break the curve where a wrapped angle jumps
    pts = [ax.c2p(a, b) for a, b in zip(ts, ys)]
    m = VMobject(stroke_color=color, stroke_width=width).set_points_as_corners(pts)
    return DashedVMobject(m, num_dashes=40) if dashed else m


# ---------------------------------------------------------------------------
# 1. Stability criteria: transfer function and state space
# ---------------------------------------------------------------------------
def part_criteria(scene: Scene):
    head = title_bar("How do we decide stability?", "same answer from the transfer functions and from A")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    left = step_lines([
        ("Transfer functions (Phase 1)", {"Transfer functions": YELLOW}),
        ("G₁(s) = N₁(s)/Δ(s),   G₂(s) = N₂(s)/Δ(s)", {}),
        ("BIBO stable  ⇔  every pole has Re < 0", {}),
        ("", {}),
        ("State space", {"State space": YELLOW}),
        ("x̃' = A x̃ + B τ̃",  {}),
        ("asymptotically stable  ⇔  every eig(A) has Re < 0", {}),
        ("any Re > 0  ⇒  unstable (also nonlinear: Lyapunov)", {}),
        ("", {}),
        ("rank[B AB A²B A³B] = 4,  rank of observability = 4", {}),
        ("no pole–zero cancellation  ⇒  poles of G = eig(A)", {"poles of G = eig(A)": C_OK}),
    ], font_size=21, buff=0.15).to_edge(LEFT, buff=0.6).shift(DOWN * 0.4)
    for l in left:
        scene.play(FadeIn(l, shift=RIGHT * 0.15), run_time=0.45)
    scene.wait(1.5)

    right = step_lines([
        ("Δ(s) = s⁴ − tr(P) s² + det P", {"Δ(s)": YELLOW}),
        ("no s³ and no s¹ term  (no friction)", {}),
        ("⇒ Routh / Hurwitz fails for every equilibrium", {"fails": C_BAD}),
        ("⇒ none is asymptotically stable with τ = 0", {}),
        ("", {}),
        ("λ = s²:   λ² − tr(P) λ + det P = 0", {}),
        ("λ < 0  →  s = ±jω   (oscillation)", {}),
        ("λ > 0  →  s = ±σ    (one pole in RHP)", {}),
    ], font_size=21, buff=0.15).to_edge(RIGHT, buff=0.6).shift(DOWN * 0.4)
    for l in right:
        scene.play(FadeIn(l, shift=LEFT * 0.15), run_time=0.45)
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])

    # table of the four equilibria
    head = title_bar("Poles at every equilibrium", "transfer function poles = eigenvalues of A")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    ax, plane = splane(7.0, 4.4)
    plane.to_edge(RIGHT, buff=0.7).shift(DOWN * 0.45)
    scene.play(FadeIn(plane))
    verdicts = {"E₁": ("marginally stable", C_WARN), "E₂": ("unstable · 1 RHP pole", C_BAD),
                "E₃": ("unstable · 2 RHP poles", C_BAD), "E₄": ("unstable · 1 RHP pole", C_BAD)}
    marks = None
    rows = VGroup()
    for i, (name, (a, b)) in enumerate(EQUILIBRIA.items()):
        A, _, P, _ = linearize(a, b)
        eig = np.linalg.eigvals(A)
        lam = np.sort(np.linalg.eigvals(P).real)
        new = VGroup(*[cross(ax.c2p(e.real, e.imag), C_OK if e.real < 1e-9 else C_BAD) for e in eig])
        txt = step_lines([
            (f"{name}  {EQ_NAMES[name]}", {name: C_OK}),
            (f"tr P = {np.trace(P):+.3f},  det P = {np.linalg.det(P):+.3f}", {}),
            (f"λ = s² = {lam[0]:+.3f}, {lam[1]:+.3f}", {}),
            (verdicts[name][0], {verdicts[name][0]: verdicts[name][1]}),
        ], font_size=19, buff=0.08)
        rows.add(txt)
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.3).to_edge(LEFT, buff=0.6).shift(DOWN * 0.35)
        if marks is None:
            scene.play(FadeIn(txt), FadeIn(new))
        else:
            scene.play(FadeIn(txt), ReplacementTransform(marks, new), run_time=1.0)
        marks = new
        scene.wait(1.6)
    note = Text("E₁ → E₃: gravity flips, P → −P, so ±jω become ±ω", font_size=19, color=YELLOW
                ).to_edge(DOWN, buff=0.25)
    scene.play(FadeIn(note))
    scene.wait(2.5)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


# ---------------------------------------------------------------------------
# 2. Energy landscape: number of RHP poles = number of "downhill" directions of U
# ---------------------------------------------------------------------------
def part_energy(scene: Scene):
    head = title_bar("Why: the shape of the potential energy",
                     "K = ∂τ_g/∂q = −Hessian(U),   P = M⁻¹K,   M > 0")
    scene.play(FadeIn(head, shift=DOWN * 0.2))

    n = 240
    q1 = np.linspace(-0.5 * np.pi, 1.5 * np.pi, n)
    q2 = np.linspace(-0.5 * np.pi, 1.5 * np.pi, n)
    Q1, Q2 = np.meshgrid(q1, q2)
    U = potential(Q1, Q2)
    z = (U - U.min()) / (U.max() - U.min())
    cmap = [np.array(color_to_rgb(c)) for c in ("#0B3C5D", "#328CC1", "#D9B310", "#E74C3C")]
    idx = z * (len(cmap) - 1)
    lo = np.clip(np.floor(idx).astype(int), 0, len(cmap) - 2)
    f = (idx - lo)[..., None]
    rgb = np.stack([np.array(cmap)[lo][..., c] for c in range(3)], -1) * (1 - f) + \
        np.stack([np.array(cmap)[lo + 1][..., c] for c in range(3)], -1) * f
    img = ImageMobject((np.flipud(rgb) * 255).astype(np.uint8))
    img.height = 5.0
    img.to_edge(LEFT, buff=0.8).shift(DOWN * 0.45)
    frame = SurroundingRectangle(img, buff=0, color=GRAY_B, stroke_width=1.5)

    def to_img(a, b):
        x = (a - q1[0]) / (q1[-1] - q1[0])
        y = (b - q2[0]) / (q2[-1] - q2[0])
        return img.get_corner(DL) + RIGHT * x * img.width + UP * y * img.height

    labx = Text("q₁ →", font_size=17, color=GRAY_B).next_to(img, DOWN, buff=0.1)
    laby = Text("q₂ →", font_size=17, color=GRAY_B).rotate(PI / 2).next_to(img, LEFT, buff=0.1)
    scene.play(FadeIn(img), Create(frame), FadeIn(labx), FadeIn(laby))
    kinds = {"E₁": ("min", C_OK), "E₂": ("saddle", C_WARN), "E₃": ("max", C_BAD), "E₄": ("saddle", C_WARN)}
    dots = VGroup()
    for name, (a, b) in EQUILIBRIA.items():
        pnt = to_img(a, b)
        dots.add(Dot(pnt, radius=0.09, color=WHITE),
                 Text(f"{name} {kinds[name][0]}", font_size=17, color=kinds[name][1], weight=BOLD
                      ).next_to(pnt, UR, buff=0.06))
    scene.play(FadeIn(dots, lag_ratio=0.2), run_time=1.5)

    rows = [("Sylvester / inertia: M > 0, so the signs of", {}),
            ("eig(P) = eig(M⁻¹K) are the signs of eig(K) = −eig(∇²U)", {}),
            ("", {})]
    for k, name in enumerate(EQUILIBRIA):
        h = RES[f"E{k + 1}"]["hessU"]
        rows.append((f"{name}: eig ∇²U = {h[0]:+.2f}, {h[1]:+.2f} → {RES[f'E{k + 1}']['n_rhp']} RHP",
                     {name: kinds[name][1]}))
    rows += [("", {}),
             ("each 'downhill' direction of U", {}),
             ("= one real pole in the right half-plane", {"one real pole in the right half-plane": C_BAD}),
             ("", {}),
             ("E₁: U strict minimum and E = T + U conserved", {"E₁": C_OK}),
             ("⇒ V = E − E* is a Lyapunov function, V' = 0", {}),
             ("⇒ stable (Lyapunov), not asymptotically", {"stable (Lyapunov)": C_OK})]
    txt = step_lines(rows, font_size=18, buff=0.11).to_edge(RIGHT, buff=0.45).shift(DOWN * 0.45)
    for l in txt:
        scene.play(FadeIn(l, shift=LEFT * 0.1), run_time=0.35)
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


# ---------------------------------------------------------------------------
# 3. Simulation evidence (PyBullet): open loop at every equilibrium
# ---------------------------------------------------------------------------
def part_openloop(scene: Scene):
    head = title_bar("Simulation evidence (PyBullet, τ = 0)",
                     "nonlinear model (colour)  vs  linearized model (white)  ·  real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    centres = [np.array([-3.6, 0.95, 0]), np.array([3.6, 0.95, 0]),
               np.array([-3.6, -2.45, 0]), np.array([3.6, -2.45, 0])]
    starts = {1: "q̃(0) = (10°, −10°)", 2: "q̃₁(0) = 2°", 3: "q̃₁(0) = 1°", 4: "q̃₁(0) = 2°"}
    scale = 0.38
    u = ValueTracker(0.0)
    T_SHOW = 8.0
    static, dynamic = VGroup(), []
    for k, (name, qs), ctr in zip((1, 2, 3, 4), EQUILIBRIA.items(), centres):
        pre = f"open{k}"
        t = series(pre, "t")
        T = float(t[-1])
        ymax = 25.0 if k == 1 else 190.0
        pivot = ctr + LEFT * 2.25 + DOWN * 0.15
        stable = k == 1
        lab = Text(f"{name} {EQ_NAMES[name]} · {starts[k]}", font_size=18,
                   color=C_OK if stable else ORANGE).move_to(ctr + UP * 1.45)
        tag = Text("oscillates forever" if stable else f"falls in ≈ {RES[f'E{k}'].get('t_to_30deg', 0):.1f} s from a tiny push"
                   if k != 1 else "", font_size=14, color=GRAY_B).next_to(lab, DOWN, buff=0.05)
        ax = Axes(x_range=[0, T, T / 4], y_range=[-ymax, ymax, ymax / 2], x_length=3.6, y_length=1.9,
                  tips=False, axis_config=dict(stroke_width=1.5, color=GRAY_B)
                  ).move_to(ctr + RIGHT * 1.2 + DOWN * 0.2)
        ylab = Text(f"q̃₁, q̃₂ [deg] (±{ymax:g})", font_size=12, color=GRAY_B).next_to(ax, UP, buff=0.03).align_to(ax, LEFT)
        xlab = Text(f"t: 0 → {T:.1f} s", font_size=12, color=GRAY_B).next_to(ax, DOWN, buff=0.04)
        static.add(VGroup(lab, tag, pivot_mount(pivot), ax, ylab, xlab))

        def robots(pre=pre, qs=qs, pivot=pivot, T=T):
            uu = min(1.0, u.get_value() * T_SHOW / T) if T < T_SHOW else u.get_value()
            q1n = qs[0] + np.radians(sample(pre, "q1", uu))
            q2n = qs[1] + np.radians(sample(pre, "q2", uu))
            g = VGroup()
            l1, l2 = sample(pre, "q1_lin", uu), sample(pre, "q2_lin", uu)
            if np.isfinite(l1) and max(abs(l1), abs(l2)) < 60:
                g.add(acrobot_static(pivot, qs[0] + np.radians(l1), qs[1] + np.radians(l2), scale,
                                     width=4, opacity=0.5, ghost=True))
            g.add(acrobot_static(pivot, q1n, q2n, scale, width=6))
            return g

        def curves(pre=pre, ax=ax, ymax=ymax, T=T):
            uu = min(1.0, u.get_value() * T_SHOW / T) if T < T_SHOW else u.get_value()
            t = series(pre, "t")
            return VGroup(curve_upto(ax, t, series(pre, "q1_lin"), uu, WHITE, 1.6, ymax),
                          curve_upto(ax, t, series(pre, "q2_lin"), uu, GRAY_B, 1.6, ymax),
                          curve_upto(ax, t, series(pre, "q1"), uu, C_LINK1, 2.8, ymax),
                          curve_upto(ax, t, series(pre, "q2"), uu, C_LINK2, 2.8, ymax))

        dynamic += [always_redraw(robots), always_redraw(curves)]
    scene.play(FadeIn(static))
    scene.add(*dynamic)
    scene.wait(0.5)
    # the unstable runs are shorter than 8 s; they stop at their end
    scene.play(u.animate.set_value(1.0), run_time=T_SHOW, rate_func=linear)
    scene.wait(2)
    scene.play(*[FadeOut(m) for m in scene.mobjects])

    # growth rate = largest RHP pole
    head = title_bar("The measured growth rate is the RHP pole",
                     "‖q̃‖ from a tiny offset, log scale:  straight line of slope p_max")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    ax = Axes(x_range=[0, 4, 0.5], y_range=[-2, 2.5, 1], x_length=7.5, y_length=4.6, tips=False,
              axis_config=dict(stroke_width=1.5, color=GRAY_B)).to_edge(LEFT, buff=0.9).shift(DOWN * 0.5)
    ticks = VGroup(*[Text(s, font_size=14, color=GRAY_B).next_to(ax.c2p(0, e), LEFT, buff=0.1)
                     for e, s in ((-2, "0.01°"), (-1, "0.1°"), (0, "1°"), (1, "10°"), (2, "100°"))])
    xt = VGroup(*[Text(f"{v:g}", font_size=14, color=GRAY_B).next_to(ax.c2p(v, -2), DOWN, buff=0.1)
                  for v in (0, 1, 2, 3, 4)])
    xl = Text("time [s]", font_size=15, color=GRAY_B).next_to(ax, DOWN, buff=0.45)
    scene.play(Create(ax), FadeIn(ticks), FadeIn(xt), FadeIn(xl))
    cols = {2: "#16A34A", 3: "#E74C3C", 4: "#9B59B6"}
    w = ValueTracker(0.0)
    groups = []
    legend = VGroup()
    for k in (2, 3, 4):
        t = series(f"grow{k}", "t")
        mag = np.hypot(series(f"grow{k}", "q1"), series(f"grow{k}", "q2"))
        ly = np.log10(np.maximum(mag, 1e-3))

        def c(t=t, ly=ly, col=cols[k]):
            i = max(2, int(round(w.get_value() * (len(t) - 1))))
            keep = ly[:i] < 2.45
            pts = [ax.c2p(a, b) for a, b in zip(t[:i][keep], ly[:i][keep])]
            return VMobject(stroke_color=col, stroke_width=3.5).set_points_as_corners(pts) if len(pts) > 1 else VMobject()
        groups.append(always_redraw(c))
        r = RES[f"E{k}"]
        legend.add(Text(f"E{'₂₃₄'[k - 2]}: slope {r['growth_fit']:.2f} 1/s   pole {r['growth_pole']:.2f} 1/s"
                        f"   (error {r['growth_err_pct']:.1f} %)", font_size=19, color=cols[k]))
    legend.arrange(DOWN, aligned_edge=LEFT, buff=0.25).to_edge(RIGHT, buff=0.4).shift(UP * 0.6)
    scene.add(*groups)
    scene.play(w.animate.set_value(1.0), run_time=5, rate_func=linear)
    for l in legend:
        scene.play(FadeIn(l, shift=LEFT * 0.1), run_time=0.5)
    e1 = RES["E1"]
    more = step_lines([
        ("E₁ over 60 s:", {"E₁": C_OK}),
        (f"spectrum peaks {e1['fft_peaks'][0]:.2f}, {e1['fft_peaks'][1]:.2f} rad/s", {}),
        ("= poles ±j2.24, ±j6.10", {}),
        (f"energy drift {e1['energy_drift_60s']:.1e} J  (conserved)", {}),
    ], font_size=18, buff=0.1).next_to(legend, DOWN, buff=0.6, aligned_edge=LEFT)
    scene.play(FadeIn(more))
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


# ---------------------------------------------------------------------------
# 4. Upright: PD feedback from one angle
# ---------------------------------------------------------------------------
def part_pd(scene: Scene):
    head = title_bar("Upright E₃: feedback from ONE measured angle?",
                     "first try static / PD feedback   τ = −(Kp + Kd s) q̃ⱼ")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    A, B, P, _ = linearize(np.pi, 0.0)
    lines = [("G₁ = −3.0147 (s² − 4.2103) / Δ(s)    zeros ±2.0519", {}),
             ("G₂ = +6.0332 (s² − 5.2595) / Δ(s)    zeros ±2.2934", {}),
             ("Δ(s) = s⁴ − 42.241 s² + 186.772", {}),
             ("", {}),
             ("closed loop:  Δ(s) + (Kp + Kd s)(b₂ s² + b₀) = 0", {"closed loop": YELLOW}),
             ("s⁴ + Kd b₂ s³ + (a₂ + Kp b₂) s² + Kd b₀ s + (a₀ + Kp b₀)", {}),
             ("", {}),
             ("Hurwitz needs  Kd b₂ > 0  AND  Kd b₀ > 0", {}),
             ("q₁: b₂ = −3.01, b₀ = +12.69    → opposite signs", {"opposite signs": C_BAD}),
             ("q₂: b₂ = +6.03, b₀ = −31.73    → opposite signs", {"opposite signs": C_BAD}),
             ("", {}),
             ("b₀/b₂ < 0  ⇔  the zeros are REAL: ±z in the RHP", {"REAL": C_BAD}),
             ("⇒ no Kp, Kd works, for either angle", {"no Kp, Kd works": C_BAD})]
    txt = step_lines(lines, font_size=20, buff=0.12).to_edge(LEFT, buff=0.5).shift(DOWN * 0.35)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.4)
    scene.wait(1)

    # root locus: sweep Kp with a damping-type Kd for q1, one pole never leaves the RHP
    ax, plane = splane(9.0, 4.4)
    plane.to_edge(RIGHT, buff=0.5).shift(DOWN * 0.4)
    scene.play(FadeIn(plane))
    KD = -10.0                         # sign that makes the s^3 coefficient Kd*b2 positive for q1
    kpv = ValueTracker(-400.0)

    def poles():
        F = np.zeros((1, 4))
        F[0, 0], F[0, 2] = kpv.get_value(), KD
        e = np.linalg.eigvals(A - B @ F)
        g = VGroup()
        for z in e:
            if abs(z.real) < 9 and abs(z.imag) < 9:
                g.add(cross(ax.c2p(z.real, z.imag), C_OK if z.real < 0 else C_BAD, 0.1, 4))
        mx = np.max(e.real)
        g.add(Text(f"PD on q₁:  Kd = {KD:.0f},  Kp = {kpv.get_value():+.0f}   "
                   f"max Re = {mx:+.2f}", font_size=16, color=C_BAD if mx > 0 else C_OK
                   ).next_to(plane, DOWN, buff=0.15))
        return g
    zeros = VGroup(*[Circle(radius=0.11, color=C_LINK1, stroke_width=3).move_to(ax.c2p(s, 0)) for s in (-2.0519, 2.0519)])
    rl = always_redraw(poles)
    scene.play(FadeIn(zeros), FadeIn(rl))
    scene.play(kpv.animate.set_value(400.0), run_time=6, rate_func=there_and_back_with_pause)
    scene.wait(0.5)
    best_txt = Text(f"best gains on a 321×241 grid: max Re = {RES['PD']['q1']['min_max_re']:.2f} (q₁), "
                    f"{RES['PD']['q2']['min_max_re']:.2f} (q₂)  > 0", font_size=19, color=YELLOW
                    ).to_edge(DOWN, buff=0.2)
    scene.play(FadeIn(best_txt))
    scene.wait(2.5)
    scene.play(*[FadeOut(m) for m in scene.mobjects])

    # PyBullet: best PD still falls
    head = title_bar("PyBullet: PD with the best gains still falls", "q̃₁(0) = 1°, τ limited to ±20 N·m")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    u = ValueTracker(0.0)
    groups = []
    for j, x0 in ((1, -3.2), (2, 3.2)):
        pre = f"pd{j}"
        pivot = np.array([x0, -0.4, 0])
        sc = 0.9
        lab = Text(f"PD on q{'₁₂'[j - 1]}  (Kp = {RES['PD'][f'q{j}']['best_kp']:.0f}, Kd = {RES['PD'][f'q{j}']['best_kd']:.0f})",
                   font_size=20).move_to(np.array([x0, 2.4, 0]))
        scene.play(FadeIn(lab), FadeIn(pivot_mount(pivot)), run_time=0.5)
        T = float(series(pre, "t")[-1])

        def rob(pre=pre, pivot=pivot, sc=sc, T=T):
            uu = min(1.0, u.get_value() * 4.0 / T)
            q1 = np.pi + np.radians(sample(pre, "q1", uu))
            q2 = np.radians(sample(pre, "q2", uu))
            tau = sample(pre, "tau", uu)
            return VGroup(acrobot_static(pivot, q1, q2, sc, width=7), torque_arc(pivot, q1, q2, sc, tau))
        groups.append(always_redraw(rob))
    scene.add(*groups)
    scene.play(u.animate.set_value(1.0), run_time=4.0, rate_func=linear)
    t30 = RES["PD"]
    scene.play(FadeIn(Text(f"both leave 30° within {max(t30['q1']['t_to_30deg_from_1deg'], t30['q2']['t_to_30deg_from_1deg']):.1f} s",
                           font_size=22, color=C_BAD).to_edge(DOWN, buff=0.4)))
    scene.wait(2)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


# ---------------------------------------------------------------------------
# 5. Upright: dynamic output feedback (observer + LQR)
# ---------------------------------------------------------------------------
def part_observer(scene: Scene):
    head = title_bar("A dynamic controller can use one angle", "Luenberger observer from q̃ⱼ + LQR on the estimate")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    o1, o2 = RES["observer"]["q1"], RES["observer"]["q2"]
    lines = [("(A, Cⱼ) observable for j = 1 and j = 2  (rank 4)", {"observable": C_OK}),
             ("⇒ x̂' = A x̂ + B τ + L (y − Cⱼ x̂),   τ = −K x̂", {}),
             ("⇒ closed-loop poles = eig(A − BK) ∪ eig(A − LCⱼ), all Re < 0", {"all Re < 0": C_OK}),
             ("", {}),
             ("but the RHP zero sits next to an RHP pole:", {"RHP zero": C_BAD}),
             (f"q₁: z = {o1['rhp_zero']:.3f} vs p = 2.240, 6.101  →  ‖S‖∞ ≥ {o1['sens_bound']:.0f}", {}),
             (f"q₂: z = {o2['rhp_zero']:.3f} vs p = 2.240, 6.101  →  ‖S‖∞ ≥ {o2['sens_bound']:.0f}", {}),
             ("parity interlacing: q₁ OK (stable controller),", {"q₁ OK": C_OK}),
             ("   q₂ violated (the controller itself must be unstable)", {"violated": C_BAD}),
             ("", {}),
             ("⇒ possible, but extremely fragile with |τ| ≤ 20 N·m", {"extremely fragile": C_WARN})]
    txt = step_lines(lines, font_size=21, buff=0.14).next_to(head, DOWN, buff=0.5)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.45)
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])

    head = title_bar("PyBullet: observer + LQR at the upright", "torque arc at the elbow  ·  real time")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    cases = [("obs_q1_ok", "measure q₁ · start 1°", "held", C_OK, -4.6),
             ("obs_q2_ok", "measure q₂ · start 0.1°", "held", C_OK, 0.0),
             ("obs_q1_fail", "measure q₁ · start 3°", "τ saturates → falls", C_BAD, 4.6)]
    u = ValueTracker(0.0)
    groups = []
    for pre, label, res, col, x0 in cases:
        pivot = np.array([x0, -0.9, 0])
        sc = 0.75
        scene.play(FadeIn(Text(label, font_size=20).move_to(np.array([x0, 2.35, 0]))),
                   FadeIn(pivot_mount(pivot)),
                   FadeIn(DashedLine(pivot, pivot + UP * 2.4, color=GRAY_D, stroke_width=1.5)), run_time=0.4)
        T = float(series(pre, "t")[-1])

        def rob(pre=pre, pivot=pivot, sc=sc, T=T, x0=x0):
            uu = min(1.0, u.get_value() * 8.0 / T)
            q1 = np.pi + np.radians(sample(pre, "q1", uu))
            q2 = np.radians(sample(pre, "q2", uu))
            tau = sample(pre, "tau", uu)
            info = Text(f"q̃₁ = {sample(pre, 'q1', uu):+6.2f}°   τ = {tau:+5.1f}", font_size=15,
                        color=GRAY_B).move_to(np.array([x0, -3.2, 0]))
            return VGroup(acrobot_static(pivot, q1, q2, sc, width=7), torque_arc(pivot, q1, q2, sc, tau), info)
        groups.append((always_redraw(rob), res, col, x0))
    scene.add(*[g[0] for g in groups])
    scene.play(u.animate.set_value(1.0), run_time=8.0, rate_func=linear)
    scene.play(*[FadeIn(Text(r, font_size=20, color=c, weight=BOLD).move_to(np.array([x0, 1.95, 0])))
                 for _, r, c, x0 in groups])
    roa = RES["roa_deg"]
    summ = Text(f"largest offset held:  measure q₁ {roa['obs_q1_offset_q1']:.2g}°,  measure q₂ "
                f"{roa['obs_q2_offset_q1']:.2g}°,  full state {roa['full_offset_q1']:.2g}°",
                font_size=20, color=YELLOW).to_edge(DOWN, buff=0.25)
    scene.play(FadeIn(summ))
    scene.wait(3)
    scene.play(*[FadeOut(m) for m in scene.mobjects])


def part_conclusion(scene: Scene):
    head = title_bar("Phase 2 · Conclusions")
    scene.play(FadeIn(head, shift=DOWN * 0.2))
    lines = [("E₁ down–down: poles ±j2.24, ±j6.10 → marginally stable;", {"E₁": C_OK}),
             ("     Lyapunov stable via energy, oscillates forever", {}),
             ("E₂, E₄: one RHP pole (saddle of U) → unstable", {"E₂, E₄": C_BAD}),
             ("E₃ upright: two RHP poles (maximum of U) → unstable", {"E₃": C_BAD}),
             ("transfer functions and A agree (minimal realisation),", {}),
             ("     and PyBullet growth rates match the poles", {}),
             ("", {}),
             ("single angle at E₃:", {"single angle": YELLOW}),
             ("  static / PD: impossible (real RHP zeros ⇒ sign conflict)", {"impossible": C_BAD}),
             ("  dynamic (observer): possible, q₁ better than q₂,", {"possible": C_OK}),
             (f"     but only for ≈{RES['roa_deg']['obs_q1_offset_q1']:.1f}° (q₁) / "
              f"≈{RES['roa_deg']['obs_q2_offset_q1']:.2f}° (q₂) with |τ| ≤ 20 N·m", {}),
             ("", {}),
             ("interactive PyBullet simulator: phase2/acrobot_phase2_sim.py", {"phase2/acrobot_phase2_sim.py": C_LINK1})]
    txt = step_lines(lines, font_size=22, buff=0.14).next_to(head, DOWN, buff=0.45)
    for l in txt:
        scene.play(FadeIn(l, shift=RIGHT * 0.1), run_time=0.4)
    scene.wait(4)


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------
class CriteriaScene(Scene):
    def construct(self):
        part_criteria(self)


class EnergyScene(Scene):
    def construct(self):
        part_energy(self)


class OpenLoopScene(Scene):
    def construct(self):
        part_openloop(self)


class PDScene(Scene):
    def construct(self):
        part_pd(self)


class ObserverScene(Scene):
    def construct(self):
        part_observer(self)


class AcrobotPhase2(Scene):
    def construct(self):
        intro = VGroup(
            Text("Acrobot · Phase 2", font_size=48, weight=BOLD),
            Text("Stability Analysis", font_size=28, color=GRAY_B),
            Text("Bhargav · Sreekar · Kaushik · Divijat", font_size=22, color=GRAY_C),
        ).arrange(DOWN, buff=0.3)
        self.play(FadeIn(intro, shift=UP * 0.2))
        self.wait(1.5)
        self.play(FadeOut(intro))
        part_criteria(self)
        part_energy(self)
        part_openloop(self)
        part_pd(self)
        part_observer(self)
        part_conclusion(self)
