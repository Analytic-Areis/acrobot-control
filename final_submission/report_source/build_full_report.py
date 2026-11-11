"""
Build the full project report (Acrobot_Full_Report.tex), organised by topic
rather than by phase, from the same material as the phase reports.

    python3 build_full_report.py

Sources: sources/shared_sections.tex (stability, control, swing-up, simulator text),
phase5_body.tex (learning-based swing-up), pseudo/*.tex (pseudo-code).
"""

import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
REPORTS = os.path.join(HERE, "..", "reports")       # finished PDF goes here
SRC = open(os.path.join(HERE, "sources", "shared_sections.tex")).read()
P5 = open(os.path.join(HERE, "phase5_body.tex")).read()


def between(text, start, end):
    i = text.index(start)
    return text[i:text.index(end, i)]


def body_after(piece, first_line):
    """Drop the first heading line of a piece."""
    return piece[piece.index(first_line) + len(first_line):]


def promote(piece):
    """subsubsections become subsections when a piece becomes its own section."""
    return piece.replace(r"\subsubsection{", r"\subsection{")


PREAMBLE = SRC[:SRC.index(r"\begin{document}")]
WHAT = between(SRC, r"\subsection{What is an Acrobot?}", r"\subsection{Model}")
MODEL = between(SRC, r"\subsection{Model}", r"\subsection{Equations of Motion}")
EOM = between(SRC, r"\subsection{Equations of Motion}", r"\subsection{Equilibria and Linear Model")
STAB = between(SRC, r"\subsection{Stability Criteria}", r"\subsection{Simulation Evidence}")
SIMEV = between(SRC, r"\subsection{Simulation Evidence}", r"\subsection{Can One Measured Angle")
SINGLE = between(SRC, r"\subsection{Can One Measured Angle Stabilize the Upright?}", r"\section{Phase 3")
DESIGN = between(SRC, r"\subsection{Controller Design}", r"\subsection{Limits with")
LIMITS = between(SRC, r"\subsection{Limits with", r"\subsection{Time Responses}")
TIMERESP = between(SRC, r"\subsection{Time Responses}", r"\section{Phase 4")
SWING = between(SRC, r"\subsection{Energy Target}", r"\section{Simulations and Code}")
PYB = between(SRC, r"\subsection{PyBullet Simulator}", r"\subsection{MATLAB Codes}")
REFS = SRC[SRC.index(r"\begin{thebibliography}"):]
RL = body_after(P5, r"\section{Reinforcement-Learning Swing-Up}")

# ------------------------------------------------------------- new sections
TITLE = r"""\begin{titlepage}
\centering
\vspace*{2cm}
\includegraphics[width=4.2cm]{figures/logo-000.jpg}\\[3cm]
{\Huge Systems Thinking Project}\\[0.5cm]
{\Large Acrobot: Modelling, Stability, Control and Swing-Up}\\[0.25cm]
{\large Full Project Report}\\[2.2cm]
\begin{tabular}{cc}
\textbf{Author:} Bhargav & \textbf{Author:} Sreekar\\
ID: 2025102061 & ID: 2025102074\\[0.8cm]
\textbf{Author:} Kaushik & \textbf{Author:} Divijat\\
ID: 2025102071 & ID: 2025102040
\end{tabular}
\vfill
{\large November 11, 2026}
\end{titlepage}

\tableofcontents
\newpage
"""

INTRO = r"""% =====================================================================
\section{Introduction}\label{sec:intro}
% =====================================================================
""" + WHAT + r"""\subsection{Objective}
The project builds up, step by step, from a model to a robot that swings itself up and balances:
\begin{enumerate}
\item derive and verify the nonlinear model, find every equilibrium, and linearize about each one (Sections~\ref{sec:model}--\ref{sec:lin});
\item determine the stability of every equilibrium from the transfer functions and the state-space model, supported by simulation (Section~\ref{sec:stab});
\item decide whether feedback from a single measured joint angle can stabilize the upright, and explain why (Section~\ref{sec:single});
\item design controllers for the upright and the other equilibria, and characterize how far from each equilibrium they still work with $\tau_{\max}=20\Nm$ (Sections~\ref{sec:ctrl}--\ref{sec:limits});
\item starting from hanging, swing the acrobot up to the upright and hold it, and explain why the controller behaves as it does (Section~\ref{sec:swing});
\item compare the hand-designed swing-up with one learned by reinforcement learning (Section~\ref{sec:rl});
\item rebuild every model and controller (except the learned one) in Simulink and cross-check it against the Python results (Section~\ref{sec:simulink}).
\end{enumerate}

"""

MODELLING = r"""% =====================================================================
\section{System Modelling}\label{sec:model}
% =====================================================================
""" + MODEL + EOM + r"""\subsection{Derivation of the Model}
The given model was checked from first principles with the Euler--Lagrange equations. Write $c_2=\cos q_2$, $s_2=\sin q_2$ and $\varphi=q_1+q_2$.

\textbf{1) Potential energy and gravity torque:}
\begin{equation}
U=-(m_1l_{c1}+m_2l_1)\,g\cos q_1-m_2l_{c2}\,g\cos\varphi,\qquad \tau_g=-\frac{\partial U}{\partial q} .
\end{equation}
\textbf{2) Kinetic energy and inertia matrix:} the velocity of the centre of link~2 is the elbow velocity ($l_1\dot q_1$) plus the rotation about the elbow ($l_{c2}\dot\varphi$), at angle $q_2$ apart (law of cosines):
\begin{equation}
T=\tfrac12(m_1l_{c1}^2+I_1)\dot q_1^2+\tfrac12I_2\dot\varphi^2+\tfrac12m_2\big(l_1^2\dot q_1^2+l_{c2}^2\dot\varphi^2+2l_1l_{c2}c_2\dot q_1\dot\varphi\big)=\tfrac12\dot q^TM\dot q .
\end{equation}
\textbf{3) Coriolis term:} with $\partial T/\partial\dot q=M\dot q$ and $\dot M=\frac{\partial M}{\partial q_2}\dot q_2=-s_2\dot q_2\left[\begin{smallmatrix}2&1\\1&0\end{smallmatrix}\right]$,
\begin{equation}
\frac{d}{dt}\frac{\partial T}{\partial\dot q}-\frac{\partial T}{\partial q}=\tau_g+B\tau
\;\Rightarrow\; M\ddot q+\underbrace{\Big(\dot M\dot q-\frac{\partial T}{\partial q}\Big)}_{=\,C\dot q}=\tau_g+B\tau .
\end{equation}
These give exactly the $M$, $C$ and $\tau_g$ of the given model.

\subsection{State-Space Form}
$M$ is always invertible, so~\eqref{eq:eom} can be solved for $\ddot q$:
\begin{equation}
\det M=1.7729-\cos^2q_2\ge0.7729>0,\qquad
\dot x=f(x,\tau)=\begin{bmatrix}\dot q\\ M^{-1}\big(\tau_g+B\tau-C\dot q\big)\end{bmatrix}.\label{eq:f}
\end{equation}

"""

EQUILIBRIA = r"""% =====================================================================
\section{Equilibria}\label{sec:eq}
% =====================================================================
\subsection{Conditions}
At an equilibrium the arm is at rest, $\dot q=\ddot q=0$, and the motor torque is $\tau^*=0$ (course specification). Then~\eqref{eq:eom} reduces to $\tau_g(q^*)=0$:
\begin{equation*}
\text{row 2: }\sin(q_1^*+q_2^*)=0,\qquad
\text{row 1: }1.5\sin q_1^*+\underbrace{\sin(q_1^*+q_2^*)}_{=0}=0\;\Rightarrow\;\sin q_1^*=0 .
\end{equation*}
\subsection{Solutions}
$q_1^*\in\{0,\pi\}$ and $q_1^*+q_2^*\in\{0,\pi\}$, so $q_2^*\in\{0,\pi\}$ (mod $2\pi$). This gives \textbf{four equilibria}, all with both links vertical:
\begin{table}[H]
\centering
\caption{The four equilibria ($\tau^*=0$)}
\label{tab:eq}
\begin{tabular}{|c|c|c|c|l|}
\hline
 & Posture & $(q_1^*,q_2^*)$ & $U$ [J] & Description\\ \hline
$E_1$ & down--down & $(0,0)$ & $-24.525$ & both links hanging\\
$E_2$ & down--up & $(0,\pi)$ & $-4.905$ & link 1 down, link 2 up\\
$E_3$ & up--up & $(\pi,0)$ & $+24.525$ & fully upright (target)\\
$E_4$ & up--down & $(\pi,\pi)$ & $+4.905$ & link 1 up, link 2 down\\ \hline
\end{tabular}
\end{table}

"""

LINEAR = r"""% =====================================================================
\section{Linearization and Transfer Functions}\label{sec:lin}
% =====================================================================
\subsection{Deviation Variables and Jacobians}
With $\tilde q=q-q^*$, $\tilde x=x-x^*$ and $\tilde\tau=\tau$, a first-order Taylor expansion of~\eqref{eq:f} about $(x^*,0)$ gives $\dot{\tilde x}\approx\frac{\partial f}{\partial x}\big|_*\tilde x+\frac{\partial f}{\partial\tau}\big|_*\tilde\tau$.
\begin{itemize}
\item \textbf{Top half:} $\frac{d}{dt}\tilde q=\dot{\tilde q}$, so the rows are $[\,0\ \ I\,]$.
\item \textbf{Bottom half:} at $(x^*,0)$ the Coriolis term drops out, $\partial(C\dot q)/\partial\dot q=0$, because $C\dot q$ is quadratic in $\dot q$. The derivative of $M^{-1}$ multiplies $\tau_g+B\tau=0$, so it drops out too. Only the gravity stiffness survives:
\end{itemize}
\begin{gather}
\ddot{\tilde q}=P\tilde q+r\tilde\tau,\qquad P=M(q^*)^{-1}K(q^*),\qquad r=M(q^*)^{-1}B,\\
K=\frac{\partial\tau_g}{\partial q}\Big|_{q^*}=-9.81\begin{bmatrix}1.5\cos q_1^*+\cos\varphi^* & \cos\varphi^*\\ \cos\varphi^* & \cos\varphi^*\end{bmatrix}.\nonumber
\end{gather}

\subsection{Linear State-Space Model}
\begin{equation}
\dot{\tilde x}=A\tilde x+B_\ell\tilde\tau,\quad y=C_y\tilde x,\qquad
A=\begin{bmatrix}0&I\\P&0\end{bmatrix},\quad B_\ell=\begin{bmatrix}0\\r\end{bmatrix},\quad C_y=\begin{bmatrix}I&0\end{bmatrix}.\label{eq:lin}
\end{equation}
\textbf{Example, $E_1$:} $K=\left[\begin{smallmatrix}-24.525&-9.81\\-9.81&-9.81\end{smallmatrix}\right]$, $M=\left[\begin{smallmatrix}4.663&2.33\\2.33&1.33\end{smallmatrix}\right]$, $\det M=0.7729$, so
\[
P_{E1}=\begin{bmatrix}-12.6292&12.6926\\14.7488&-29.6119\end{bmatrix},\qquad r_{E1}=\begin{bmatrix}-3.0147\\6.0332\end{bmatrix}.
\]
$M$ depends only on $q_2^*$ and $K$ only on the signs of $\cos q_1^*$ and $\cos\varphi^*$. Upside down, gravity flips: $P_{E3}=-P_{E1}$ and $P_{E4}=-P_{E2}$.
\begin{table}[H]
\centering
\caption{Linearization at the four equilibria ($\det M=0.7729$ at all four)}
\label{tab:lin}
\resizebox{\textwidth}{!}{\begin{tabular}{|c|c|c|c|c|}
\hline
 & $(\cos q_1^*,\cos\varphi^*)$ & $M(q^*)$ & $P=M^{-1}K$ & $r=M^{-1}B$\\ \hline
$E_1$ & $(1,1)$ & $\left[\begin{smallmatrix}4.663&2.33\\2.33&1.33\end{smallmatrix}\right]$ & $\left[\begin{smallmatrix}-12.6292&12.6926\\14.7488&-29.6119\end{smallmatrix}\right]$ & $\left[\begin{smallmatrix}-3.0147\\6.0332\end{smallmatrix}\right]$\\
$E_2$ & $(1,-1)$ & $\left[\begin{smallmatrix}0.663&0.33\\0.33&1.33\end{smallmatrix}\right]$ & $\left[\begin{smallmatrix}-12.6292&12.6926\\10.5095&4.2266\end{smallmatrix}\right]$ & $\left[\begin{smallmatrix}-0.4270\\0.8578\end{smallmatrix}\right]$\\
$E_3$ & $(-1,-1)$ & as $E_1$ & $-P_{E1}$ & $r_{E1}$\\
$E_4$ & $(-1,1)$ & as $E_2$ & $-P_{E2}$ & $r_{E2}$\\ \hline
\end{tabular}}
\end{table}

\subsection{Transfer Functions}
With zero initial conditions, $s^2\tilde Q=P\tilde Q+r\tilde{\mathcal T}$, so $(s^2I-P)\tilde Q(s)=r\,\tilde{\mathcal T}(s)$ and
\begin{gather}
G_1(s)=\frac{r_1s^2+(P_{12}r_2-P_{22}r_1)}{\Delta(s)},\qquad
G_2(s)=\frac{r_2s^2+(P_{21}r_1-P_{11}r_2)}{\Delta(s)},\label{eq:tf}\\
\Delta(s)=s^4-\operatorname{tr}(P)\,s^2+\det P .\nonumber
\end{gather}
\begin{itemize}
\item \textbf{Poles:} the roots of $\Delta$. With $\lambda=s^2$, $\lambda^2-\operatorname{tr}(P)\lambda+\det P=0$, then $s=\pm\sqrt\lambda$.
\item \textbf{Zeros:} $s^2=-(P_{12}r_2-P_{22}r_1)/r_1$ for $G_1$ and $s^2=-(P_{21}r_1-P_{11}r_2)/r_2$ for $G_2$.
\item With no friction only even powers of $s$ occur, so poles and zeros come in pairs $\pm p$.
\end{itemize}
\begin{table}[H]
\centering
\caption{Transfer functions at the four equilibria}
\label{tab:tf}
\resizebox{\textwidth}{!}{\begin{tabular}{|c|l|l|l|l|l|}
\hline
 & $\Delta(s)$ & $N_1(s)$ ($\tau\to q_1$) & $N_2(s)$ ($\tau\to q_2$) & Zeros of $G_1$ & Zeros of $G_2$\\ \hline
$E_1$ & $s^4+42.2410\,s^2+186.7719$ & $-3.0147\,(s^2+4.2103)$ & $6.0332\,(s^2+5.2595)$ & $\pm j2.0519$ & $\pm j2.2934$\\
$E_2$ & $s^4+8.4025\,s^2-186.7719$ & $-0.4270\,(s^2-29.7273)$ & $0.8578\,(s^2+7.3982)$ & $\pm5.4523$ & $\pm j2.7200$\\
$E_3$ & $s^4-42.2410\,s^2+186.7719$ & $-3.0147\,(s^2-4.2103)$ & $6.0332\,(s^2-5.2595)$ & $\pm2.0519$ & $\pm2.2934$\\
$E_4$ & $s^4-8.4025\,s^2-186.7719$ & $-0.4270\,(s^2+29.7273)$ & $0.8578\,(s^2-7.3982)$ & $\pm j5.4523$ & $\pm2.7200$\\ \hline
\end{tabular}}
\end{table}

"""

SIMENV = r"""% =====================================================================
\section{Simulation Environment and Model Verification}\label{sec:sim}
% =====================================================================
\subsection{PyBullet Model}
All closed-loop and open-loop results in this report come from the full nonlinear model in \textbf{PyBullet} [5]: uniform rods, no joint damping, step $0.25$~ms, torque applied at the elbow only.
\subsection{Verification}
\begin{itemize}
\item \textbf{Model:} the derived $M$, $C$, $\tau_g$ match the given model term by term, and MATLAB reproduces every value in Tables~\ref{tab:lin} and~\ref{tab:tf} to four decimals.
\item \textbf{PyBullet vs.\ the equations:} with $q(0)=(0.8,-0.5)$~rad and $\tau=3\sin2t$, PyBullet and an RK4 integration of~\eqref{eq:eom} agree within $0.52^\circ$ over 5~s. The error shrinks with the step size, so it is integration error, not a model mismatch.
\item \textbf{Energy:} with $\tau=0$, $T+U$ stays constant (no friction): to $10^{-8}$ with RK4 and to $1.2\times10^{-3}$~J over 60~s in PyBullet.
\item \textbf{Simulink:} an independent Simulink implementation reproduces the open-loop and closed-loop results to $10^{-10}$ degrees (Section~\ref{sec:simulink}).
\item \textbf{Linear vs.\ nonlinear:} from the same small deviation the two agree within $1^\circ$ ($q_1$) and $2^\circ$ ($q_2$) over 8~s near $E_1$ (Fig.~\ref{fig:sim}).
\end{itemize}
\begin{figure}[H]
\centering
\includegraphics[width=0.55\textwidth]{figures/sim.pdf}
\caption{Nonlinear (colour) vs.\ linear (black) free responses near (a) $E_1$ and (b) $E_3$.}
\label{fig:sim}
\end{figure}
""" + PYB.replace(r"\subsection{PyBullet Simulator}", r"\subsection{Interactive Simulator}", 1) + "\n"


def section(title, label, piece):
    return ("% " + "=" * 69 + "\n" + rf"\section{{{title}}}\label{{{label}}}" + "\n" + "% " + "=" * 69 + "\n" + piece)


STABILITY = section("Stability Analysis", "sec:stab", STAB + SIMEV)
SINGLE_S = section("Feedback from a Single Measured Angle", "sec:single",
                   promote(body_after(SINGLE, r"\subsection{Can One Measured Angle Stabilize the Upright?}")))
CONTROL = section("Controller Design Around the Equilibria", "sec:ctrl", DESIGN)
LIMITS_S = section("Performance Limits under the Torque Limit", "sec:limits",
                   promote(body_after(LIMITS, LIMITS.split("\n", 1)[0])) + TIMERESP)
SWING_S = section("Swing-Up Control", "sec:swing", SWING)
RL_S = section("Learning-Based Swing-Up", "sec:rl", RL)

SIMULINK_S = r"""% =====================================================================
\section{Simulink Implementation and Cross-Validation}\label{sec:simulink}
% =====================================================================
Every result above was also rebuilt as a \textbf{Simulink} block diagram (MATLAB R2025b, Simulink 25.2). Simulink is the standard environment for control design in industry. A block diagram reads like the control loop on paper, the same model can later be turned into embedded code, and it gives a third, independent implementation of the plant and controllers next to the Python/PyBullet and MATLAB-script versions. Four models cover the open-loop, single-sensor, LQR and swing-up results; the learned (RL) controller is not included because it is a neural-network policy trained in Python.

\subsection{Common Plant Structure}
All four models share the same nonlinear plant loop. The torque command passes through a saturation block and enters a MATLAB Function block that evaluates the given equations of motion~\eqref{eq:eom}; an integrator closes the loop:
\begin{equation}
\tau_{\text{cmd}}\ \xrightarrow{\ \operatorname{sat}_{\pm20}\ }\ \tau\ \longrightarrow\ \dot x=f(x,\tau)\ \longrightarrow\ \int\!dt\ \longrightarrow\ x,\qquad x(0)=x_0 .
\end{equation}
\begin{table}[H]
\centering
\caption{Blocks of the shared plant}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|l|}
\hline
Block & Simulink library block & Role\\ \hline
\texttt{tau limit} & Discontinuities / Saturation & actuator limit $|\tau|\le20\Nm$\\
\texttt{dynamics} & User-Defined Functions / MATLAB Function & $\ddot q=M(q)^{-1}\big(\tau_g(q)+B\tau-C(q,\dot q)\dot q\big)$, returns $\dot x$\\
\texttt{x} & Continuous / Integrator ($1/s$) & state $x=(q_1,q_2,\dot q_1,\dot q_2)$, initial condition \texttt{x0}\\
\texttt{controller} & User-Defined Functions / MATLAB Function & control law (PD, LQR or hybrid swing-up)\\
Constants & Sources / Constant & gains and equilibrium ($K$, $S$, $q^\ast$, $K_p$, $K_d$, \ldots) from the workspace\\
\texttt{x\_out}, \texttt{tau\_out} & Sinks / To Workspace & logged state and torque for analysis\\
\texttt{angles} & Sinks / Scope & live plot of $q_1$, $q_2$\\ \hline
\end{tabular}}
\end{table}
The solver is the variable-step \texttt{ode45} (Dormand--Prince) with relative tolerance $10^{-7}$, absolute tolerance $10^{-9}$ and a maximum step of 1~ms. Controller gains are not typed into the diagrams: they are computed in MATLAB (for example with \texttt{lqr}) and read from the workspace, so the same model runs at any equilibrium and with any gains.

\subsection{Models}
\begin{table}[H]
\centering
\caption{Simulink models}
\label{tab:slxmodels}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|l|l|}
\hline
Model & Topic & Controller block & Workspace inputs\\ \hline
\texttt{Acrobot\_OpenLoop} & stability (Sec.~\ref{sec:stab}) & none, $\tau=0$; plus a linear State-Space block $(A,B)$ & \texttt{A}, \texttt{B}, \texttt{x0}, \texttt{x0\_lin}\\
\texttt{Acrobot\_SingleAnglePD} & single sensor (Sec.~\ref{sec:single}) & $\tau=-(K_pe_j+K_d\dot e_j)$ on one angle $j$ & \texttt{qs}, \texttt{Kp}, \texttt{Kd}, \texttt{j\_meas}\\
\texttt{Acrobot\_LQR} & control (Secs.~\ref{sec:ctrl}--\ref{sec:limits}) & $\tau=-K\tilde x$, angles wrapped to $(-\pi,\pi]$ & \texttt{qs}, \texttt{K}\\
\texttt{Acrobot\_SwingUp} & swing-up (Sec.~\ref{sec:swing}) & energy pumping, LQR when $\tilde x^TS\tilde x<5$ & \texttt{K}, \texttt{S}, \texttt{gains}, \texttt{c\_switch}\\ \hline
\end{tabular}}
\end{table}
\begin{figure}[H]
\centering
\includegraphics[width=0.82\textwidth]{figures/slx_openloop.png}
\caption{\texttt{Acrobot\_OpenLoop}: the nonlinear plant (top) and the linearized State-Space model (bottom) driven by the same $\tau=0$, so the two can be compared from the same deviation.}
\label{fig:slx_ol}
\end{figure}
\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/slx_pd.png}
\caption{\texttt{Acrobot\_SingleAnglePD}: PD feedback from one measured angle (\texttt{j\_meas} $=1$ or $2$) at the upright.}
\label{fig:slx_pd}
\end{figure}
\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/slx_lqr.png}
\caption{\texttt{Acrobot\_LQR}: full-state feedback $\tau=\operatorname{sat}(-K\tilde x)$ about the equilibrium \texttt{qs}.}
\label{fig:slx_lqr}
\end{figure}
\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/slx_swingup.png}
\caption{\texttt{Acrobot\_SwingUp}: the hybrid controller has two outputs, the torque and the mode flag \texttt{isLQR} (logged as \texttt{lqr\_on}), which becomes 1 once the state enters the catch region $\tilde x^TS\tilde x<5$.}
\label{fig:slx_swing}
\end{figure}
In the swing-up model the controller block holds both laws of Section~\ref{sec:swing}. Outside the catch region it computes the energy error $E-E_{top}$ and the collocated partial-feedback-linearization torque; inside it applies the upright LQR. Because the switch is evaluated at every solver step, the solver handles the change of law as a discontinuity in $\tau$, exactly like the real hybrid controller.

\subsection{Results}
The script \texttt{run\_simulink\_models.m} simulates every model with the gains and initial states used in this report and checks each outcome. All 15 checks pass (Table~\ref{tab:slx}, Fig.~\ref{fig:slx_res}).
\begin{table}[H]
\centering
\caption{Simulink results (all checks pass)}
\label{tab:slx}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|l|l|}
\hline
Model & Case & Simulink result & Expected (this report)\\ \hline
OpenLoop & $E_1$, from $(10^\circ,-10^\circ)$ & bounded, max.\ $14.7^\circ$ over 8 s & marginally stable\\
OpenLoop & $E_2$ & growth rate $3.171$ /s & pole $3.1775$ (error $0.19\%$)\\
OpenLoop & $E_3$ & growth rate $6.088$ /s & pole $6.1011$ (error $0.22\%$)\\
OpenLoop & $E_4$ & growth rate $4.300$ /s & pole $4.3010$ (error $0.03\%$)\\
SingleAnglePD & $E_3$, PD on $q_1$ ($K_p=-15$, $K_d=0$) & falls & no single-angle PD works\\
SingleAnglePD & $E_3$, PD on $q_2$ ($K_p=400$, $K_d=150$) & falls & no single-angle PD works\\
LQR & $E_3$, $R=0.03$, from $2^\circ$ & held, peak $|\tau|=17.56\Nm$ & held\\
LQR & $E_3$, from $5.5^\circ$ & held, $|\tau|$ saturates & held\\
LQR & $E_3$, from $7.5^\circ$ & falls & falls (beyond the $6^\circ$ limit)\\
LQR & $E_3$, from $(-10^\circ,25^\circ)$ & held, peak $3.34\Nm$ & held (centre-of-mass line)\\
LQR & $E_2$, $R=0.1$, from $(60^\circ,90^\circ)$ & held & held\\
LQR & $E_4$, $R=0.1$, from $(20^\circ,30^\circ)$ & held, peak $4.47\Nm$ & held\\
SwingUp & from $(3^\circ,0^\circ)$ & caught at $2.17$ s, held & caught at $2.17$ s\\
SwingUp & from $(0.5^\circ,0^\circ)$ & caught at $1.90$ s, held & held\\
SwingUp & from $(-10^\circ,20^\circ)$ & caught at $8.10$ s, held & held\\ \hline
\end{tabular}}
\end{table}
\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/fig_simulink.pdf}
\caption{Simulink responses (lines) with the Python simulation of the same case (markers). (a) Free oscillation near $E_1$, nonlinear vs.\ linear State-Space block. (b) Near $E_3$ the deviation grows at the unstable pole $6.10$/s until the nonlinearity takes over. (c) Upright LQR from $2^\circ$, $5.5^\circ$ and $7.5^\circ$. (d) Swing-up from $3^\circ$ and LQR catch at $2.17$ s, with the saturated torque (grey, right axis).}
\label{fig:slx_res}
\end{figure}

\subsection{Cross-Validation with the Python Model}
Each Simulink trajectory was re-simulated from the same initial state with the Python model and controllers used for the rest of the report, integrated independently with fixed-step RK4 (steps of at most 0.1~ms, landing exactly on the Simulink time points). The two implementations share only the equations, not the code or the solver.
\begin{table}[H]
\centering
\caption{Simulink vs.\ Python (maximum angle difference over the whole run)}
\label{tab:slxpy}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|c|l|}
\hline
Case & Max.\ difference & Note\\ \hline
LQR gain $K$ (MATLAB vs.\ Python \texttt{lqr}) & $1.2\times10^{-13}$ (relative) & identical design\\
Open loop near $E_1$, 8 s & $6.9\times10^{-11}$ deg & smooth\\
Open loop near $E_3$, 2.5 s & $1.1\times10^{-10}$ deg & smooth; falls from $0.5^\circ$\\
LQR at $E_3$ from $2^\circ$ & $4.2\times10^{-11}$ deg & smooth\\
LQR at $E_3$ from $5.5^\circ$ & $6.0\times10^{-11}$ deg & saturates briefly\\
LQR at $E_3$ from $7.5^\circ$ & $0.17$ deg & both fall; long saturation\\
Swing-up from $3^\circ$, 10 s & $0.0066$ deg & catch $2.16691$ s vs.\ $2.16693$ s\\ \hline
\end{tabular}}
\end{table}
Where the torque is smooth the two agree to $10^{-10}$ degrees, which is the level of the integration error of the two solvers. They differ more (but still well below a degree) only where the torque is discontinuous: at saturation and at the swing-up/LQR switch. There the variable-step solver and the fixed-step RK4 place the corner at slightly different instants, and the difference is then amplified by the unstable dynamics. Even so, both give the same outcome in every case, and the swing-up catch times agree to $2\times10^{-5}$~s. The Simulink models therefore confirm the model, the stability results, the controllers and the swing-up independently of the Python and PyBullet code.

\subsection{Reproducing the Simulink Results}
The models are generated by a script rather than drawn by hand, so they can always be rebuilt identically:
\begin{itemize}
\item \texttt{build\_simulink\_models.m} creates the four \texttt{.slx} models and copies them into the code folders of the corresponding parts of the project;
\item \texttt{run\_simulink\_models.m} runs the 15 checks of Table~\ref{tab:slx} and prints PASS/FAIL;
\item \texttt{make\_simulink\_figures.m} exports the diagrams and the trajectories;
\item \texttt{compare\_with\_python.py} re-simulates them in Python and produces Table~\ref{tab:slxpy} and Fig.~\ref{fig:slx_res}.
\end{itemize}
Each model can also be opened and run by hand after setting its workspace inputs (Table~\ref{tab:slxmodels}).

"""

SOFTWARE = r"""% =====================================================================
\section{Software and Code}\label{sec:code}
% =====================================================================
\subsection{Files}
\begin{table}[H]
\centering
\caption{Code files}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|l|}
\hline
Topic & File & Purpose\\ \hline
Model & \texttt{Acrobot\_Nonlinear\_Dynamics.m} & nonlinear model $\dot x=f(x,\tau)$ with the $20\Nm$ limit\\
Model & \texttt{Acrobot\_Linearize.m} & $A$, $B_\ell$ at an equilibrium\\
Stability & \texttt{Acrobot\_Phase2\_Main.m} & poles, zeros, ranks, verdicts, growth rates, single-angle PD test\\
Control & \texttt{Acrobot\_Phase3\_Design.m}, \texttt{Acrobot\_Phase3\_Main.m} & controllers, guaranteed regions, regions of attraction\\
Swing-up & \texttt{Acrobot\_Phase4\_Controller.m}, \texttt{Acrobot\_Phase4\_Main.m} & hybrid controller, swing-up and robustness tests\\
Learned swing-up & \texttt{acrobot\_env.py}, \texttt{train.py}, \texttt{evaluate.py} & environment, SAC training, comparison (Python)\\
Simulator & \texttt{acrobot\_phase\{2,3,4\}\_sim.py} & interactive PyBullet dashboards\\
Simulink & \texttt{Acrobot\_OpenLoop.slx}, \texttt{Acrobot\_SingleAnglePD.slx} & open loop vs.\ linear model; single-angle PD\\
Simulink & \texttt{Acrobot\_LQR.slx}, \texttt{Acrobot\_SwingUp.slx} & full-state LQR; hybrid swing-up $\to$ LQR\\
Simulink & \texttt{build\_simulink\_models.m}, \texttt{run\_simulink\_models.m} & build the models; run the 15 checks\\
Simulink & \texttt{make\_simulink\_figures.m}, \texttt{compare\_with\_python.py} & diagrams, trajectories, Simulink vs.\ Python\\ \hline
\end{tabular}}
\end{table}
\subsection{Pseudo-Code}
\input{pseudo/model}
\input{pseudo/phase2}
\input{pseudo/phase3}
\input{pseudo/phase4}
\input{pseudo/phase5}
\subsection{Animations}
2K Manim animations of the work:
\href{https://drive.google.com/file/d/18Ic6QdsY37QBfvFIwmVA_8Z-u4AyNPdg/view?usp=sharing}{\textcolor{blue}{model and equilibria}},
\href{https://drive.google.com/file/d/19THfoXXzbdMtbteCKE4v0VRLpV869wE0/view?usp=sharing}{\textcolor{blue}{stability}},
\href{https://drive.google.com/file/d/1c-7Yy664ZP4TCJIeDbrAmFozHC8frao1/view?usp=sharing}{\textcolor{blue}{control}},
\href{https://drive.google.com/file/d/1jPQRn3W-bvUyvSKlHvYmX74bfPX9N16P/view?usp=sharing}{\textcolor{blue}{swing-up}};
the comparison of the learned and energy swing-ups is \url{phase5_rl/RL_vs_Energy_SwingUp_2K.mp4}.

"""

TABULATION = r"""% =====================================================================
\section{Tabulation}
% =====================================================================
\begin{table}[H]
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
\caption{Summary of every controller ($|\tau|\le20\Nm$)}
\resizebox{\textwidth}{!}{\begin{tabular}{|l|l|l|}
\hline
Controller & Works? & Limit\\ \hline
PD on one angle at $E_3$ & no (proved) & always unstable\\
Observer + LQR, one angle, at $E_3$ & yes & $1.42^\circ$ ($q_1$), $0.23^\circ$ ($q_2$)\\
Full-state LQR at $E_3$ & yes & $6^\circ$ / $13^\circ$, $\ge43^\circ$ along the CoM line\\
Pole placement at $E_3$ & yes & $5^\circ$ / $9^\circ$\\
LQR at $E_2$ & yes & almost any rest position (99.7\%)\\
LQR at $E_4$ & yes & $35^\circ$ / $55^\circ$\\
Elbow PD at $E_1$ & yes & 78\% of $\pm180^\circ$\\
Energy swing-up $\to$ LQR & yes & 39/40 random starts (40 s), 35/40 (30 s)\\
RL (SAC) swing-up $\to$ LQR, 1 kg only & yes & 40/40 (30 s); 15/40 with links $+20\%$ heavier\\
RL swing-up $\to$ LQR, random masses & yes & 40/40 from 0.6 to 1.2 kg; 2/40 at 1.4 kg\\ \hline
\end{tabular}}
\end{table}

"""

CONCLUSION = r"""% =====================================================================
\section{Conclusion}
% =====================================================================
The given acrobot model was verified from first principles. It has four zero-torque equilibria, all with both links vertical. Only the hanging pose is stable, and only in the Lyapunov sense: with no friction it oscillates forever. The other three are unstable, with one, two and one right-half-plane poles, matching the number of directions in which the potential energy decreases. The transfer functions and the state matrix agree because every realization is minimal, and PyBullet confirmed the predicted growth rates to within $0.25\%$.

A single measured angle cannot stabilize the upright with PD feedback, because the transfer functions have real right-half-plane zeros. An observer-based controller can, but only for about a degree. With full-state feedback all four equilibria were stabilized. At the upright the $20\Nm$ limit, not the tuning, restricts LQR to a thin band around the line where the centre of mass is over the pivot.

An energy-pumping swing-up with partial feedback linearization brings the arm from hanging to the upright, and the LQR catches and holds it from 39 of 40 random starts. Stiffer gains fail because saturation breaks the linearization. A swing-up learned with reinforcement learning was faster, gentler and more reliable on the model it was trained on, but much less robust when the link masses changed. Training it on random link masses, with the missing energy as one extra input, fixed this over the practised range (40 of 40 from 0.6 to 1.2~kg) at no cost on the correct model; far outside that range (1.4~kg) the physics-based law remained the more reliable one. The hybrid design (learn or design the swing-up, keep the provable LQR catch) proved the most useful structure throughout.

Finally, every model and controller except the learned one was rebuilt in Simulink. The Simulink models pass all 15 checks and agree with the independent Python simulation to $10^{-10}$ degrees wherever the torque is smooth, and to well below a degree across saturation and the swing-up switch, so the results do not depend on one particular piece of code.

% =====================================================================
\section{Contributions}
% =====================================================================
% Fill in who did what before submitting.
\begin{enumerate}
\item Report compilation -- \underline{\hspace{4cm}}
\item Modelling, equilibria and linearization (Sections 2--4) -- \underline{\hspace{4cm}}
\item Simulation environment and verification (Section 5) -- \underline{\hspace{4cm}}
\item Stability analysis and single-sensor feedback (Sections 6--7) -- \underline{\hspace{4cm}}
\item Controller design and limits (Sections 8--9) -- \underline{\hspace{4cm}}
\item Swing-up control (Section 10) -- \underline{\hspace{4cm}}
\item Learning-based swing-up (Section 11) -- \underline{\hspace{4cm}}
\item Simulink models and cross-validation (Section 12) -- \underline{\hspace{4cm}}
\item Code, simulator and animations (Section 13) -- \underline{\hspace{4cm}}
\end{enumerate}

"""

EXTRA_REFS = r"""\bibitem{sac} T.~Haarnoja, A.~Zhou, P.~Abbeel and S.~Levine, ``Soft actor-critic: Off-policy maximum entropy deep reinforcement learning with a stochastic actor,'' in \emph{Proc. ICML}, 2018.
\bibitem{sb3} A.~Raffin, A.~Hill, A.~Gleave, A.~Kanervisto, M.~Ernestus and N.~Dormann, ``Stable-Baselines3: Reliable reinforcement learning implementations,'' \emph{JMLR}, vol.~22, no.~268, pp.~1--8, 2021.
"""

# -------------------------------------------------- phase wording -> topics
REWORD = [
    (r"(catch: Phase 3 LQR at $E_3$)", r"(catch: LQR at $E_3$, Sec.~\ref{sec:ctrl})"),
    (r"(catch: Phase 3 LQR, } R=0.03\text{)}", r"(catch: LQR, } R=0.03\text{, Sec.~\ref{sec:ctrl})}"),
    (r"\caption{Phase 3 controllers and their closed-loop poles}", r"\caption{Designed controllers and their closed-loop poles}"),
    (r"The dynamics (given in the Phase~1 helper and verified in Phase~1) are", r"The dynamics (given in the course helper document and verified in Section~\ref{sec:sim}) are"),
    (r"All Phase~2--4 simulations use the full nonlinear model", r"All simulations use the full nonlinear model"),
    (r"so Phase~3 uses full-state feedback.", r"so full-state feedback is used from here on (Section~\ref{sec:ctrl})."),
    (r"(from Phase 1)", r""),
    (r"Phase~3 LQR", r"upright LQR of Section~\ref{sec:ctrl}"),
    (r"Phase 3 LQR", r"upright LQR of Section~\ref{sec:ctrl}"),
    (r"the Phase~4 swing-up", r"the energy swing-up of Section~\ref{sec:swing}"),
    (r"The Phase~4 swing-up", r"The energy swing-up of Section~\ref{sec:swing}"),
    (r"Phase~4 robustness test", r"robustness test of Section~\ref{sec:swing}"),
    (r"Phase 4 robustness test", r"robustness test of Section~\ref{sec:swing}"),
    (r"same 40 random starts as Phase~4", r"same 40 random starts as in Section~\ref{sec:swing}"),
    (r"the Phase~4 value of 39", r"the value of 39 in Section~\ref{sec:swing}"),
    (r"retry logic as Phase~4", r"retry logic as the energy swing-up"),
    (r"the same target as Phase~4", r"the same target as the energy swing-up"),
    (r"In Phase~4 saturation was what broke the energy law", r"Saturation was what broke the energy law (Section~\ref{sec:swing})"),
    (r"(Phase~3)", r"(Section~\ref{sec:limits})"),
    (r"the Phase 3 limit", r"the LQR limit"),
    (r"The catch is the Phase 3 limit.", r"The catch is the LQR limit."),
    (r"found at $E_3$ in Phase~2", r"found at $E_3$ in Section~\ref{sec:single}"),
]


def build():
    doc = (PREAMBLE + "\\begin{document}\n\n" + TITLE + INTRO + MODELLING + EQUILIBRIA + LINEAR + SIMENV
           + STABILITY + SINGLE_S + CONTROL + LIMITS_S + SWING_S + RL_S + SIMULINK_S + SOFTWARE + TABULATION + CONCLUSION
           + REFS.replace(r"\end{thebibliography}", EXTRA_REFS + r"\end{thebibliography}"))
    for a, b in REWORD:
        doc = doc.replace(a, b)
    name = "Acrobot_Full_Report.tex"
    open(os.path.join(HERE, name), "w").write(doc)
    left = sorted(set(m.group(0) for m in re.finditer(r"[^\n]{0,60}Phase[~ ]?\d[^\n]{0,40}", doc.split(r"\begin{document}")[1])))
    return name, left


if __name__ == "__main__":
    name, left = build()
    print("phase mentions left in the text:", len(left))
    for l in left:
        print("   ", l.strip())
    subprocess.run(["latexmk", "-pdf", "-interaction=nonstopmode", "-halt-on-error", name], cwd=HERE,
                   capture_output=True)
    log = open(os.path.join(HERE, name.replace(".tex", ".log"))).read()
    pages = re.search(r"Output written on .*?\((\d+) pages", log)
    warn = re.findall(r"(Overfull \\hbox.*|LaTeX Warning: (?:Reference|Citation).*undefined.*|^!.*)", log, re.M)
    print(name, "->", f"{pages.group(1)} pages" if pages else "FAILED", "|", len(warn), "warnings")
    for w in warn[:8]:
        print("   ", w)
    subprocess.run(["latexmk", "-c", name], cwd=HERE, capture_output=True)
    if pages:                                       # finished PDF -> ../reports
        os.makedirs(REPORTS, exist_ok=True)
        os.replace(os.path.join(HERE, name.replace(".tex", ".pdf")), os.path.join(REPORTS, "Acrobot_Full_Report.pdf"))
