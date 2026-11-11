%% RUN_SIMULINK_MODELS  Simulate every acrobot Simulink model and check it against the reports.
%
%   Run build_simulink_models first. All gains and initial states are the
%   ones used in the reports, so each line printed below should say PASS.

clear; clc;
here = fileparts(mfilename('fullpath'));
cd(here);
wrapA = @(a) mod(a + pi, 2*pi) - pi;
lin = @(q1s, q2s) linearize_at(q1s, q2s);
pass = @(c) ternary(c, 'PASS', 'FAIL');

%% 1. Open loop (Phase 2): tau = 0 near each equilibrium
eqs = [0 0; 0 pi; pi 0; pi pi]; names = {'E1', 'E2', 'E3', 'E4'};
fprintf('=== Acrobot_OpenLoop (tau = 0) ===\n');
for k = 1:4
    [A, B] = lin(eqs(k, 1), eqs(k, 2));                         %#ok<ASGLU>
    if k == 1, d0 = deg2rad([10; -10]); Tstop = 8; else, d0 = deg2rad([0.01; 0]); Tstop = 3; end
    x0 = [eqs(k, :)' + d0; 0; 0];  x0_lin = [d0; 0; 0];            %#ok<NASGU>
    out = sim('Acrobot_OpenLoop');
    X = states(out.x_nl); t = out.x_nl.Time;
    dev = rad2deg(hypot(wrapA(X(:, 1) - eqs(k, 1)), wrapA(X(:, 2) - eqs(k, 2))));
    if k == 1
        fprintf('%s: stays bounded, max deviation %.1f deg over 8 s            %s\n', names{k}, max(dev), pass(max(dev) < 30));
    else
        use = dev > 0.2 & dev < 15;
        c = polyfit(t(use), log(dev(use)), 1);
        pmax = max(real(eig(A)));
        fprintf('%s: growth rate %.3f 1/s vs pole %.4f (error %.2f%%)        %s\n', names{k}, c(1), pmax, ...
                100 * abs(c(1) - pmax) / pmax, pass(abs(c(1) - pmax) / pmax < 0.02));
    end
end

%% 2. Single-angle PD at the upright (Phase 2): best gains still fall
fprintf('\n=== Acrobot_SingleAnglePD (upright E3, 1 deg offset) ===\n');
qs = [pi; 0]; x0 = [pi + deg2rad(1); 0; 0; 0]; Tstop = 4;
for g = [1 -15 0; 2 400 150]'                                 % [j; Kp; Kd]
    j_meas = g(1); Kp = g(2); Kd = g(3);                       %#ok<NASGU>
    out = sim('Acrobot_SingleAnglePD');
    X = states(out.x_nl);
    dev = rad2deg(max(abs([wrapA(X(:, 1) - pi), wrapA(X(:, 2))]), [], 2));
    fprintf('PD on q%d (Kp = %g, Kd = %g): max deviation %.0f deg -> falls       %s\n', j_meas, Kp, Kd, ...
            max(dev), pass(max(dev) > 30));
end

%% 3. Full-state LQR (Phase 3)
fprintf('\n=== Acrobot_LQR ===\n');
cases = { 'E3 LQR R=0.03', [pi; 0],  0.03, [2; 0],    true;
          'E3 LQR R=0.03', [pi; 0],  0.03, [5.5; 0],  true;
          'E3 LQR R=0.03', [pi; 0],  0.03, [7.5; 0],  false;
          'E3 LQR R=0.03', [pi; 0],  0.03, [-10; 25], true;
          'E2 LQR R=0.1',  [0; pi],  0.1,  [60; 90],  true;
          'E4 LQR R=0.1',  [pi; pi], 0.1,  [20; 30],  true };
Tstop = 10;
for r = 1:size(cases, 1)
    qs = cases{r, 2};
    [A, B] = lin(qs(1), qs(2));
    K = lqr(A, B, eye(4), cases{r, 3});                        %#ok<NASGU>
    x0 = [qs + deg2rad(cases{r, 4}); 0; 0];
    out = sim('Acrobot_LQR');
    X = states(out.x_nl); X = X(end, :)';
    held = all(abs(rad2deg([wrapA(X(1) - qs(1)), wrapA(X(2) - qs(2))])) < 1) && all(abs(X(3:4)) < 0.05);
    fprintf('%-14s from (%5.1f, %5.1f) deg: %-8s (expected %-8s) peak |tau| %5.2f   %s\n', cases{r, 1}, ...
            cases{r, 4}, ternary(held, 'held', 'not held'), ternary(cases{r, 5}, 'held', 'not held'), ...
            max(abs(out.tau_log.Data)), pass(held == cases{r, 5}));
end

%% 4. Swing-up (Phase 4)
fprintf('\n=== Acrobot_SwingUp (from hanging) ===\n');
[A, B] = lin(pi, 0);
[K, S] = lqr(A, B, eye(4), 0.03);                              %#ok<ASGLU>
gains = [20 2 3]; c_switch = 5; Tstop = 30;                    %#ok<NASGU>
for d0 = [3 0; 0.5 0; -10 20]'
    x0 = [deg2rad(d0); 0; 0];
    out = sim('Acrobot_SwingUp');
    X = states(out.x_nl); X = X(end, :)';
    on = out.lqr_on.Data(:); tt = out.lqr_on.Time;
    tc = tt(find(on > 0.5, 1));
    held = all(abs(rad2deg([wrapA(X(1) - pi), wrapA(X(2))])) < 1) && all(abs(X(3:4)) < 0.05);
    fprintf('start (%5.1f, %5.1f) deg: first catch %5.2f s, %s                 %s\n', d0, tc, ...
            ternary(held, 'held upright', 'NOT held'), pass(held));
end

function [A, B] = linearize_at(q1s, q2s)
    c2 = cos(q2s); M = [2.663 + 2*c2, 1.33 + c2; 1.33 + c2, 1.33];
    cp = cos(q1s + q2s); Kg = -9.81*[1.5*cos(q1s) + cp, cp; cp, cp];
    A = [zeros(2), eye(2); M \ Kg, zeros(2)];
    B = [0; 0; M \ [0; 1]];
end

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end

function X = states(ts)
% logged state as an N-by-4 array (Simulink stores a 4-vector as 4x1xN)
    X = ts.Data;
    if ndims(X) == 3, X = permute(X, [3 1 2]); end
end
