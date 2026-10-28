%% =========================================================================
%  ACROBOT PROJECT - PHASE 1: MODEL CHECK, EQUILIBRIA, LINEARIZATION, TFs
%  =========================================================================
%  Team:  Bhargav (2025102061), Sreekar (2025102074),
%         Kaushik (2025102071), Divijat (2025102040)
%
%  What this script does
%    1. Defines the handout parameters.
%    2. Verifies the given nonlinear model: derives M(q), C(q,dq)*dq and
%       tau_g(q) symbolically from the Lagrangian and checks they equal the
%       handout expressions.
%    3. Finds the equilibria for zero input torque (tau* = 0).
%    4. Linearizes the model at each equilibrium (symbolic Jacobians) and
%       prints P = M^-1 K, r = M^-1 B and the state-space model (A,B,C,D)
%       with output y = [q1; q2].
%    5. Computes the transfer functions tau -> q1 and tau -> q2 (monic
%       denominator), their poles and zeros, and cross-checks them against
%       MATLAB's ss -> tf conversion.
%
%  Requires: Symbolic Math Toolbox, Control System Toolbox.
%  =========================================================================
clear; clc;

%% 1. Parameters (from the project handout)
m1 = 1;     m2 = 1;          % link masses            [kg]
l1 = 1;     l2 = 2;          % link lengths           [m]
lc1 = l1/2; lc2 = l2/2;      % joint-to-COM distances [m] (uniform rods)
I1 = 0.083; I2 = 0.33;       % inertias about COM     [kg m^2]
g  = 9.81;                   % gravity                [m/s^2]

%% 2. Verify the given model against a Lagrangian derivation
syms q1 q2 dq1 dq2 tau real
q  = [q1; q2];   dq = [dq1; dq2];   x = [q; dq];
B  = [0; 1];
phi = q1 + q2;                                   % absolute angle of link 2

% --- Model as given in the handout / helper document --------------------
c2 = cos(q2);  s2 = sin(q2);
M_H = [I1 + I2 + m1*l1^2/4 + m2*(l1^2 + l2^2/4 + l1*l2*c2), I2 + m2*(l2^2/4 + l1*l2/2*c2);
       I2 + m2*(l2^2/4 + l1*l2/2*c2),                         I2 + m2*l2^2/4];
C_H = [-m2*l1*l2*s2*dq2, -m2*l1*l2/2*s2*dq2;
        m2*l1*l2/2*s2*dq1, 0];
U_H = -(m1*l1/2 + m2*l1)*g*cos(q1) - m2*l2/2*g*cos(phi);
tg_H = -jacobian(U_H, q).';                      % tau_g = -grad U

% --- Independent Lagrangian derivation ------------------------------------
pc1 = lc1*[sin(q1); -cos(q1)];
pc2 = l1*[sin(q1); -cos(q1)] + lc2*[sin(phi); -cos(phi)];
v1  = jacobian(pc1, q)*dq;   v2 = jacobian(pc2, q)*dq;
T   = 1/2*m1*(v1.'*v1) + 1/2*m2*(v2.'*v2) + 1/2*I1*dq1^2 + 1/2*I2*(dq1 + dq2)^2;
U   = -m1*g*lc1*cos(q1) - m2*g*(l1*cos(q1) + lc2*cos(phi));
M_L = simplify(hessian(T, dq));                         % T = 1/2 dq' M dq
h_L = simplify(jacobian(M_L*dq, q)*dq - jacobian(T, q).'); % Coriolis/centrifugal vector
tg_L = simplify(-jacobian(U, q).');

ok_M = all(isAlways(simplify(M_L - M_H) == 0), 'all');
ok_C = all(isAlways(simplify(h_L - C_H*dq) == 0), 'all');
ok_U = isAlways(simplify(U - U_H) == 0);
fprintf('1. MODEL VERIFICATION (Lagrangian vs handout)\n');
fprintf('   M(q)         identical: %d\n', ok_M);
fprintf('   C(q,dq)*dq   identical: %d\n', ok_C);
fprintf('   U(q)         identical: %d\n', ok_U);
fprintf('   det M(q) = %s  (> 0 for all q)\n\n', char(vpa(simplify(det(M_H)), 6)));

%% 3. Nonlinear state equation dx/dt = f(x, tau)
ddq = M_H \ (tg_H + B*tau - C_H*dq);
f   = [dq; ddq];
A_sym = jacobian(f, x);
B_sym = jacobian(f, tau);

%% 4. Equilibria with zero input torque (tau* = 0)
%  dq = ddq = 0  ->  tau_g(q*) = 0
%    row 2:  sin(q1 + q2) = 0
%    row 1:  1.5 sin(q1) + sin(q1 + q2) = 0  ->  sin(q1) = 0
%  => q1* in {0, pi}, q2* in {0, pi}  (angles modulo 2*pi)
eq_name = {'E1 (down-down)', 'E2 (down-up)', 'E3 (up-up)', 'E4 (up-down)'};
eq_q    = [0 0; 0 pi; pi 0; pi pi];

fprintf('2. EQUILIBRIA (tau* = 0)\n');
for k = 1:4
    res = double(subs(tg_H, q, eq_q(k,:).'));
    Uk  = double(subs(U_H,  q, eq_q(k,:).'));
    fprintf('   %-15s q* = (%4.2f, %4.2f) rad   |tau_g(q*)| = %.1e   U = %+8.3f J\n', ...
            eq_name{k}, eq_q(k,1), eq_q(k,2), norm(res), Uk);
end
fprintf('\n');

%% 5. Linearization, state-space models and transfer functions
Cy = [eye(2), zeros(2)];     % output y = [q1; q2]
Dy = zeros(2, 1);
fprintf('3. LINEAR MODELS AND TRANSFER FUNCTIONS\n');
for k = 1:4
    xs = [eq_q(k,:).'; 0; 0];
    A  = double(subs(A_sym, [x; tau], [xs; 0]));
    Bl = double(subs(B_sym, [x; tau], [xs; 0]));
    P  = A(3:4, 1:2);
    r  = Bl(3:4);

    % Transfer functions from (s^2 I - P) Q(s) = r T(s)
    den  = [1, 0, -trace(P), 0, det(P)];            % monic denominator
    num1 = [r(1), 0, P(1,2)*r(2) - P(2,2)*r(1)];     % tau -> q1
    num2 = [r(2), 0, P(2,1)*r(1) - P(1,1)*r(2)];     % tau -> q2
    G1 = tf(num1, den);   G2 = tf(num2, den);

    % Cross-check against MATLAB's ss -> tf conversion
    Gss  = tf(ss(A, Bl, Cy, Dy));
    wchk = [0.3 1.1 3.7 9.3];                        % test frequencies [rad/s]
    err  = max(abs([squeeze(freqresp(Gss(1), wchk)) - squeeze(freqresp(G1, wchk)); ...
                    squeeze(freqresp(Gss(2), wchk)) - squeeze(freqresp(G2, wchk))]));

    fprintf('=== %s ===\n', eq_name{k});
    fprintf('   P = M^-1 K = [%9.4f %9.4f; %9.4f %9.4f]\n', P(1,1), P(1,2), P(2,1), P(2,2));
    fprintf('   r = M^-1 B = [%7.4f; %7.4f]\n', r(1), r(2));
    fprintf('   A = [0 I; P 0],  B = [0; r],  C = [I 0],  D = 0\n');
    fprintf('   Delta(s) = s^4 %+.4f s^2 %+.4f\n', den(3), den(5));
    fprintf('   G1(s) = %.4f (s^2 %+.4f) / Delta(s)\n', r(1), num1(3)/r(1));
    fprintf('   G2(s) = %.4f (s^2 %+.4f) / Delta(s)\n', r(2), num2(3)/r(2));
    fprintf('   poles   : %s\n', num2str(roots(den).', '%9.4f '));
    fprintf('   zeros G1: %s\n', num2str(roots(num1).', '%9.4f '));
    fprintf('   zeros G2: %s\n', num2str(roots(num2).', '%9.4f '));
    fprintf('   max |G - tf(ss(A,B,C,D))| at test points = %.1e\n\n', err);
end
