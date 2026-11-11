"""
Build one report per phase (Report_Phase2..5.tex) from the shared sections in
sources/shared_sections.tex, then compile them.

    python3 build_phase_reports.py

Each report: title page, contents, the model introduction, that phase's
analysis, its simulator and MATLAB code, a tabulation, a conclusion,
contributions and references (same format as the combined report).
"""

import os
import shutil
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
REPORTS = os.path.join(HERE, "..", "reports")       # finished PDFs go here
SRC = open(os.path.join(HERE, "sources", "shared_sections.tex")).read()


def between(start, end):
    i = SRC.index(start)
    j = SRC.index(end, i)
    return SRC[i:j]


PREAMBLE = SRC[:SRC.index(r"\begin{document}")]
INTRO = between(r"\section{Introduction}", r"\subsection{Objective}")
BODY = {
    2: between(r"\section{Phase 2: Stability Analysis}", r"\section{Phase 3: Control Around the Equilibria}"),
    3: between(r"\section{Phase 3: Control Around the Equilibria}", r"\section{Phase 4: Swing-Up}"),
    4: between(r"\section{Phase 4: Swing-Up}", r"\section{Simulations and Code}"),
}
REFS = SRC[SRC.index(r"\begin{thebibliography}"):]
BODY[5] = open(os.path.join(HERE, "phase5_body.tex")).read()

TITLE = {2: "Phase 2: Stability Analysis", 3: "Phase 3: Control Around the Equilibria", 4: "Phase 4: Swing-Up",
         5: "Phase 5: Reinforcement-Learning Swing-Up"}
SECTION = {2: "Stability Analysis", 3: "Control Around the Equilibria", 4: "Swing-Up",
           5: "Reinforcement-Learning Swing-Up"}

OBJECTIVE = {
    5: r"""Replace the hand-designed Phase~4 swing-up with one learned by reinforcement learning, keep the same Phase~3 LQR for the final catch, and compare the two under $|\tau|\le20\Nm$: reliability, speed, control effort, and robustness when the real robot differs from the model.

Phase~4 provides the baseline: an energy-pumping swing-up with partial feedback linearization, gains $k=(20,2,3)$, which switches to the Phase~3 LQR ($Q=I$, $R=0.03$) when $\tilde x^TS\tilde x<5$.""",
    2: r"""Determine the stability of every equilibrium using both the transfer functions and the state-space model, and support each conclusion with simulation. At the upright equilibrium $E_3$, determine whether feedback from a \emph{single} measured joint angle can stabilize the system, and explain why.""",
    3: r"""Design controllers that stabilize the acrobot at the upright equilibrium $E_3$ and at least one other equilibrium, and justify the design choices. Characterize the limits of each controller: how far from the equilibrium it still works with $\tau_{\max}=20\Nm$.

Phase~2 showed that $E_1$ is only marginally stable, that $E_2$, $E_3$, $E_4$ are unstable (1, 2 and 1 RHP poles), and that one measured angle cannot stabilize $E_3$ with PD feedback. That result guides the design here.""",
    4: r"""Starting from hanging straight down ($E_1$), bring the acrobot to the upright equilibrium $E_3$ and hold it, with $|\tau|\le20\Nm$. Whether the attempt works or not, explain with evidence why it behaved as it did.

Phase~3 provides the controller that holds the upright: LQR with $Q=I$, $R=0.03$, $K=[-503.0,\,-203.9,\,-218.3,\,-101.5]$. It recovers only from a thin region around $E_3$ ($6^\circ$ in $q_1$, $13^\circ$ in $q_2$), so the swing-up must deliver the arm into that region.""",
}

SIM = {
    2: ("phase2/acrobot\\_phase2\\_sim.py", "phase2/README.md", "dashboard_p2.png",
        "open loop, PD on one angle (live $K_p$, $K_d$), or observer + LQR on one angle",
        "Phase 2 dashboard: PD on $q_1$ at the upright; a closed-loop pole stays in the right half-plane."),
    3: ("phase3/acrobot\\_phase3\\_sim.py", "phase3/README.md", "dashboard_p3.png",
        "open loop, full-state LQR, pole placement, elbow PD (for $E_1$), or the Phase 2 single-angle controllers",
        "Phase 3 dashboard: full-state LQR catching the upright from $4^\\circ$."),
    4: ("phase4/acrobot\\_phase4\\_sim.py", "phase4/README.md", "dashboard.png",
        "swing-up $\\to$ LQR, or any of the Phase 2 and 3 controllers",
        "Phase 4 dashboard during a swing-up from hanging."),
}

FILES = {
    2: [("\\texttt{Acrobot\\_Phase2\\_Main.m}", "poles, zeros, ranks, verdicts, growth rates, single-angle PD test")],
    3: [("\\texttt{Acrobot\\_Phase3\\_Design.m}", "the controllers (LQR, pole placement, elbow PD)"),
        ("\\texttt{Acrobot\\_Phase3\\_Main.m}", "guaranteed regions, nonlinear runs, region-of-attraction maps")],
    4: [("\\texttt{Acrobot\\_Phase4\\_Controller.m}", "hybrid swing-up $\\to$ LQR controller"),
        ("\\texttt{Acrobot\\_Phase4\\_Main.m}", "swing-up runs and the 40-start robustness test")],
    5: [("\\texttt{acrobot\\_env.py}", "Gymnasium environment, mass-parametric model, reward, RL $\\to$ LQR controller"),
        ("\\texttt{train.py}", "SAC training; saves the policy and the learning curve"),
        ("\\texttt{evaluate.py}", "40-start and wrong-mass tests, PyBullet runs, figures"),
        ("\\texttt{run\\_pipeline.sh}", "runs model check, training, evaluation and video in order")],
}
PY_CAPTION = r"Python files (\texttt{phase5\_rl/})"
COMMON_FILES = [("\\texttt{Acrobot\\_Nonlinear\\_Dynamics.m}", "nonlinear model $\\dot x=f(x,\\tau)$ with the $20\\Nm$ limit (Phase 1)"),
                ("\\texttt{Acrobot\\_Linearize.m}", "$A$, $B_\\ell$ at an equilibrium (Phase 1)")]


def file_table(rows, caption):
    body = "\n".join(f"{a} & {b}\\\\" for a, b in rows)
    return (f"\\begin{{table}}[H]\n\\centering\n\\caption{{{caption}}}\n\\resizebox{{\\textwidth}}{{!}}{{\\begin{{tabular}}{{|l|l|}}\n"
            f"\\hline\nFile & Purpose\\\\ \\hline\n{body} \\hline\n\\end{{tabular}}}}\n\\end{{table}}\n")


CODE = {
    5: [("Environment, dynamics and hybrid controllers (acrobot\\_env.py)", "acrobot_env.py"),
        ("SAC training (train.py)", "train.py"),
        ("Evaluation (evaluate.py)", "evaluate.py")],
    2: [("Phase 2: Stability Analysis", "Acrobot_Phase2_Main.m")],
    3: [("Phase 3: Controller Designs", "Acrobot_Phase3_Design.m"),
        ("Phase 3: Simulations and Region of Attraction", "Acrobot_Phase3_Main.m")],
    4: [("Phase 4: Swing-Up Controller", "Acrobot_Phase4_Controller.m"),
        ("Phase 4: Swing-Up Simulations", "Acrobot_Phase4_Main.m")],
}
CODE_NOTE = {
    2: "It prints the poles, zeros, ranks and verdict for every equilibrium, measures the growth rates with \\texttt{ode45}, and runs the single-angle PD search. Every number matches the tables above.",
    3: "It designs every controller (\\texttt{lqr}, \\texttt{place}), prints the gains, poles and guaranteed regions, runs the nonlinear saturated loops with \\texttt{ode45}, and maps the regions of attraction. Gains, poles and guaranteed regions match the tables above exactly.",
    4: "It runs the hybrid controller from three starts (RK4, 1~ms) and the 40-start robustness test: 39/40 for the chosen gains, as in the tables above.",
}
VIDEO = {
    5: None,
    2: "https://drive.google.com/file/d/19THfoXXzbdMtbteCKE4v0VRLpV869wE0/view?usp=sharing",
    3: "https://drive.google.com/file/d/1c-7Yy664ZP4TCJIeDbrAmFozHC8frao1/view?usp=sharing",
    4: "https://drive.google.com/file/d/1jPQRn3W-bvUyvSKlHvYmX74bfPX9N16P/view?usp=sharing",
}

TABULATION = {
    5: r"""\begin{table}[H]
\centering
\caption{Energy swing-up vs.\ the two RL swing-ups: summary (40 random starts, held within 30 s)}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|c|c|c|}
\hline
Criterion & Energy swing-up & RL, 1 kg only & RL, random masses\\ \hline
Held, correct masses & 35/40 & 40/40 & 40/40\\
Median time to catch & 3.98 s & 1.71 s & 1.71 s\\
Time at the torque limit & 8.8\% & 0.1\% & 0.0\%\\
Median elbow work & 244.9 J & 144.5 J & 130.0 J\\
Held, masses $-40\%$ (never practised) & 39/40 & 2/40 & 40/40\\
Held, masses $-20\%$ & 37/40 & 38/40 & 40/40\\
Held, masses $+20\%$ & 35/40 & 15/40 & 40/40\\
Held, masses $+40\%$ (never practised) & 33/40 & 3/40 & 2/40\\
Needs the equations of motion & yes & no (only a simulator) & no (only a simulator)\\
Explainable / provable & yes (energy argument) & no (tested only) & no (tested only)\\
Design effort & 3 gains + search & reward design + 12 min training & same + 26 min training\\ \hline
\end{tabular}}
\end{table}""",
    2: r"""\begin{table}[H]
\centering
\caption{Summary of the stability analysis}
\begin{tabular}{|c|c|c|c|}
\hline
Equilibrium & RHP poles & Energy shape & Stability\\ \hline
$E_1$ down--down & 0 & minimum & marginally (Lyapunov) stable\\
$E_2$ down--up & 1 & saddle & unstable\\
$E_3$ up--up & 2 & maximum & unstable\\
$E_4$ up--down & 1 & saddle & unstable\\ \hline
\end{tabular}
\end{table}
\begin{table}[H]
\centering
\caption{Single-angle feedback at the upright $E_3$ ($|\tau|\le20\Nm$)}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|c|c|}
\hline
Controller & Measuring $q_1$ & Measuring $q_2$\\ \hline
PD (any $K_p$, $K_d$) & impossible (best $\max\operatorname{Re}\lambda=0.96$) & impossible (best $2.29$)\\
Observer + LQR & held up to $1.42^\circ$ & held up to $0.23^\circ$\\
Stable controller possible? & yes & no (must be unstable)\\
Sensitivity bound $\|S\|_\infty\ge$ & $45.9$ & $187$\\ \hline
\end{tabular}}
\end{table}""",
    3: r"""\begin{table}[H]
\centering
\caption{Summary of the Phase 3 controllers ($|\tau|\le20\Nm$)}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|c|c|c|}
\hline
Controller & Design & Recovered (grid) & Along $q_1$ / $q_2$ alone & Guaranteed\\ \hline
$E_3$ LQR & $Q=I$, $R=0.03$ & 26\% ($\pm40^\circ$) & $6^\circ$ / $13^\circ$ & $0.72^\circ$ / $1.66^\circ$\\
$E_3$ pole placement & $\{-3,-4,-5,-6\}$ & 20\% ($\pm40^\circ$) & $5^\circ$ / $9^\circ$ & --\\
$E_2$ LQR & $Q=I$, $R=0.1$ & 99.7\% ($\pm180^\circ$) & $175^\circ$ / $180^\circ$ & $45.6^\circ$ / $19.9^\circ$\\
$E_4$ LQR & $Q=I$, $R=0.1$ & 20.5\% ($\pm180^\circ$) & $35^\circ$ / $55^\circ$ & $20.0^\circ$ / $23.1^\circ$\\
$E_1$ elbow PD & $K_p=-5$, $K_d=0.4$ & 77.8\% ($\pm180^\circ$) & $40^\circ$ / $115^\circ$ & --\\ \hline
\end{tabular}}
\end{table}""",
    4: r"""\begin{table}[H]
\centering
\caption{Summary of the swing-up designs (40 random starts near hanging)}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|c|c|c|}
\hline
Gains $(k_1,k_2,k_3)$, $c_{sw}$ & Saturated during swing-up & Held & Median first catch\\ \hline
$(20,2,3)$, 5 (chosen) & 26\% & \textbf{39/40} & $4.0$ s\\
$(50,5,10)$, 5 (textbook) & 33\% & 25/40 & $5.5$ s\\
$(100,2,3)$, 5 (stiff elbow) & 45\% & 2/40 & --\\
$(20,2,0.3)$, 5 (weak pumping) & 25\% & 0/40 & --\\ \hline
\end{tabular}}
\end{table}""",
}

CONCLUSION = {
    5: r"""A swing-up policy learned with Soft Actor-Critic, handing over to the same Phase~3 LQR, outperformed the hand-designed energy swing-up on the model it was trained on. It held 40 of 40 random starts instead of 35, reached the top more than twice as fast (1.7~s vs.\ 4.0~s), almost never saturated the motor, and used 41\% less work. It learned this in 12 minutes from a reward alone, with no equations of motion in the controller.

The first policy paid for this with robustness. With links 20\% heavier than in training it held only 15 of 40 starts, while the physics-based law still held 35, because the policy had learned one robot's timing rather than the energy principle. A second policy, trained with only two changes (random link masses between 0.7 and 1.3~kg in every episode, and the missing energy as one extra input), held 40 of 40 at every mass from 0.6 to 1.2~kg at no cost on the correct model. At 1.4~kg, far heavier than anything it practised on, it still failed (2 of 40) where the energy law held 33: a learned controller is reliable over the variety it practised on, while a physics-based law extrapolates further. The hybrid design (learn the hard nonlinear swing-up, keep the provable LQR catch) gives the speed of learning with a guaranteed final balance.""",
    2: r"""The acrobot has four zero-torque equilibria. Only the hanging pose $E_1$ is stable, and only in the Lyapunov sense: its poles lie on the imaginary axis and, with no friction, it oscillates forever. $E_2$ and $E_4$ have one RHP pole each (saddles of the potential energy) and the upright $E_3$ has two (a maximum), so all three are unstable. The transfer functions and the state matrix give the same verdicts because each realization is minimal, and PyBullet confirmed the predicted growth rates to within $0.25\%$.

A single measured angle cannot stabilize the upright with static or PD feedback: the real right-half-plane zeros of $G_1$ and $G_2$ make the required coefficient signs contradict each other. An observer-based dynamic controller can stabilize it, because each angle alone makes the system observable. With $|\tau|\le20\Nm$ it holds only about a degree, and via $q_2$ the controller would itself have to be unstable. This motivates full-state feedback in Phase~3.""",
    3: r"""All four equilibria were stabilized. LQR was chosen for $E_3$, $E_2$ and $E_4$ because it is guaranteed stable with good margins and trades speed against torque with one weight $R$. The weight was chosen by measuring the region of attraction of the nonlinear saturated loop. At the upright that region hardly depends on $R$: the $20\Nm$ limit, not the tuning, restricts LQR to $6^\circ$ in $q_1$ or $13^\circ$ in $q_2$, and to a thin band along the line where the centre of mass is over the pivot, because gravity twists the motorless shoulder in proportion to that offset. Pole placement settles faster but rescues less. $E_2$ is recovered from almost any rest position and $E_4$ from about $35^\circ$/$55^\circ$. $E_1$ is stabilized from the elbow angle alone, with an energy-shaping Lyapunov proof.""",
    4: r"""An energy-pumping swing-up based on partial feedback linearization brings the acrobot from hanging to the upright energy, and the Phase~3 LQR catches and holds it. With $|\tau|\le20\Nm$ the chosen gains $k=(20,2,3)$ succeed from 39 of 40 random starts, typically within about 4~s. Because swing-up resumes whenever the arm leaves the catch region, a missed catch or a push costs time, not the task.

The textbook gains, and stiffer elbow gains in general, do worse because they ask for more torque than the motor can give. Once the torque saturates, the elbow no longer follows the linearizing law, the energy is no longer steered precisely, and the arm reaches the top outside the small region the LQR can catch. Too weak a pumping gain fails for the opposite reason: the energy never reaches the target.""",
}

CONTRIB = {
    5: ["Report compilation", "RL problem and environment design (Sections 2.1--2.4)", "SAC training (Sections 2.5--2.6)",
        "Results and robust policy (Sections 2.7--2.9)", "Discussion (Section 2.10)", "Python code and figures", "Animation (Manim)"],
    2: ["Report compilation", "Stability criteria and analysis (Sections 2.1--2.5)", "Simulation evidence (Section 2.6)",
        "Single-angle feedback (Section 2.7)", "PyBullet simulator and figures", "MATLAB codes", "Animation (Manim)"],
    3: ["Report compilation", "Controller design (Sections 2.1--2.5)", "Limits and region of attraction (Section 2.6)",
        "Time responses (Section 2.7)", "PyBullet simulator and figures", "MATLAB codes", "Animation (Manim)"],
    4: ["Report compilation", "Swing-up controller (Sections 2.1--2.4)", "Results (Section 2.5)",
        "Analysis of why it works or fails (Section 2.6)", "PyBullet simulator and figures", "MATLAB codes", "Animation (Manim)"],
}


def sim_section(ph, code):
    """Phases 2-4: the PyBullet dashboard, the MATLAB code and the video link."""
    sim_file, readme, shot, modes, cap = SIM[ph]
    return rf"""% =====================================================================
\section{{Simulations and Code}}
% =====================================================================
\subsection{{PyBullet Simulator}}
All simulations use the full nonlinear model in PyBullet with an interactive dashboard (Fig.~\ref{{fig:dash}}). The user picks an equilibrium, an initial condition (slider, typed value or by dragging the arm) and a controller: {modes}. The dashboard shows $\tilde q(t)$, $\tau(t)$ and the closed-loop poles live. File: \texttt{{{sim_file}}}; instructions in \texttt{{{readme}}}.
\begin{{figure}}[H]
\centering
\includegraphics[width=\textwidth]{{figures/{shot}}}
\caption{{{cap}}}
\label{{fig:dash}}
\end{{figure}}

\subsection{{MATLAB Codes}}
{CODE_NOTE[ph]}

The files are listed below; the algorithms they implement follow as pseudo-code.
{file_table(COMMON_FILES + FILES[ph], "MATLAB files")}
\input{{pseudo/model}}
\input{{pseudo/phase{ph}}}

\subsection{{Animation}}
A 2K Manim animation of this phase: \href{{{VIDEO[ph]}}}{{\textcolor{{blue}}{{watch the video}}}}.
"""


def code_section(ph, code):
    """Phase 5: training pipeline and Python code (RL is done in Python, not MATLAB)."""
    return rf"""% =====================================================================
\section{{Simulations and Code}}
% =====================================================================
\subsection{{Training and Evaluation Pipeline}}
The work is in \texttt{{phase5\_rl/}}; \texttt{{bash run\_pipeline.sh 1000000}} runs every step in order (about 50 minutes on a laptop CPU):
\begin{{enumerate}}
\item \textbf{{Model check:}} the mass-parametric model reproduces the given equations of motion to $10^{{-13}}$.
\item \textbf{{Training}} (\texttt{{train.py}}): SAC for $10^6$ steps on 1~kg links, then the robust policy (option \texttt{{-{{}}-robust}}) for $2\times10^6$ steps. Both policies are saved in the folder \texttt{{policies/}}, each with its learning curve.
\item \textbf{{Evaluation}} (\texttt{{evaluate.py}}): all three swing-ups from 40 random starts with five link masses (0.6--1.4~kg), plus PyBullet runs; writes \texttt{{results.json}} and the figures above.
\item \textbf{{Video}} (\texttt{{manim-sim/rl\_vs\_energy.py}}): the explanatory comparison animation.
\end{{enumerate}}
Libraries: Gymnasium 1.4 (environment interface), stable-baselines3 2.9 (SAC), PyTorch 2.15, PyBullet.

\subsection{{Python Codes}}
The files are listed below; the algorithms they implement follow as pseudo-code.
{file_table(FILES[ph], PY_CAPTION)}
\input{{pseudo/phase5}}

\subsection{{Animation}}
A 2K Manim animation comparing the RL and energy swing-ups is in \url{{phase5_rl/RL_vs_Energy_SwingUp_2K.mp4}}.
"""


EXTRA_REFS = {5: r"""\bibitem{sac} T.~Haarnoja, A.~Zhou, P.~Abbeel and S.~Levine, ``Soft actor-critic: Off-policy maximum entropy deep reinforcement learning with a stochastic actor,'' in \emph{Proc. ICML}, 2018.
\bibitem{sb3} A.~Raffin, A.~Hill, A.~Gleave, A.~Kanervisto, M.~Ernestus and N.~Dormann, ``Stable-Baselines3: Reliable reinforcement learning implementations,'' \emph{JMLR}, vol.~22, no.~268, pp.~1--8, 2021.
"""}


def build(ph):
    body = BODY[ph].replace(rf"\section{{{TITLE[ph]}}}", rf"\section{{{SECTION[ph]}}}", 1)
    if ph == 3:      # the PD sign test lives in the Phase 2 report
        body = body.replace(r"so the sign conflict of~\eqref{eq:pd} does not arise",
                            r"so the PD sign conflict found at $E_3$ in Phase~2 does not arise")
    if ph == 4:      # Phase 3 section references become plain text
        body = body.replace(r"(Fig.~\ref{fig:kick}a)", r"(Fig.~\ref{fig:kick}a)")
    code = ""
    contrib = "\n".join(rf"\item {c} -- \underline{{\hspace{{4cm}}}}" for c in CONTRIB[ph])
    title = rf"""\begin{{titlepage}}
\centering
\vspace*{{2cm}}
\includegraphics[width=4.2cm]{{figures/logo-000.jpg}}\\[3cm]
{{\Huge Systems Thinking Project}}\\[0.5cm]
{{\Large Acrobot}}\\[0.25cm]
{{\large {TITLE[ph]}}}\\[2.2cm]
\begin{{tabular}}{{cc}}
\textbf{{Author:}} Bhargav & \textbf{{Author:}} Sreekar\\
ID: 2025102061 & ID: 2025102074\\[0.8cm]
\textbf{{Author:}} Kaushik & \textbf{{Author:}} Divijat\\
ID: 2025102071 & ID: 2025102040
\end{{tabular}}
\vfill
{{\large November 11, 2026}}
\end{{titlepage}}
"""
    doc = (PREAMBLE + "\\begin{document}\n\n" + title + "\n\\tableofcontents\n\\newpage\n\n" + INTRO
           + "\\subsection{Objective}\n" + OBJECTIVE[ph] + "\n\n"
           + "% " + "=" * 69 + "\n" + body
           + (code_section(ph, code) if ph == 5 else sim_section(ph, code))
           + rf"""
% =====================================================================
\section{{Tabulation}}
% =====================================================================
{TABULATION[ph]}

% =====================================================================
\section{{Conclusion}}
% =====================================================================
{CONCLUSION[ph]}

% =====================================================================
\section{{Contributions}}
% =====================================================================
% Fill in who did what before submitting.
\begin{{enumerate}}
{contrib}
\end{{enumerate}}

""" + (REFS.replace(r"\end{thebibliography}", EXTRA_REFS.get(ph, "") + r"\end{thebibliography}")))
    name = f"Report_Phase{ph}.tex"
    open(os.path.join(HERE, name), "w").write(doc)
    return name


if __name__ == "__main__":
    for ph in (2, 3, 4, 5):
        name = build(ph)
        r = subprocess.run(["latexmk", "-pdf", "-interaction=nonstopmode", "-halt-on-error", name],
                           cwd=HERE, capture_output=True, text=True)
        log = open(os.path.join(HERE, name.replace(".tex", ".log"))).read()
        pages = re.search(r"Output written on .*?\((\d+) pages", log)
        warn = re.findall(r"(Overfull \\hbox.*|LaTeX Warning: Reference.*undefined.*|^!.*)", log, re.M)
        print(name, "->", f"{pages.group(1)} pages" if pages else "FAILED", "|", len(warn), "warnings")
        for w in warn[:5]:
            print("   ", w)
        subprocess.run(["latexmk", "-c", name], cwd=HERE, capture_output=True)
        if pages:                                   # finished PDF -> ../reports
            os.makedirs(REPORTS, exist_ok=True)
            os.replace(os.path.join(HERE, name.replace(".tex", ".pdf")), os.path.join(REPORTS, f"Phase{ph}_Report.pdf"))
            rl_dir = os.path.join(HERE, "..", "..", "phase5_rl")         # working folder, if present
            if ph == 5 and os.path.isdir(rl_dir):
                shutil.copy(os.path.join(REPORTS, "Phase5_Report.pdf"), os.path.join(rl_dir, "Phase5_Report.pdf"))
