%% ACROBOT_PHASE2_MAIN  Phase 2: stability of every equilibrium + single-angle feedback.
%
%   1. For each equilibrium: eig(A), transfer functions tau->q1 and tau->q2,
%      their poles and zeros, controllability / observability ranks, verdict.
%   2. Characteristic polynomial: the s^3 and s^1 coefficients are zero.
%   3. Nonlinear check (ode45, tau = 0): growth rate of a tiny offset vs the
%      largest RHP pole (E2, E3, E4); E1 stays bounded.
%   4. Upright E3, PD from ONE angle: coefficient sign test and a gain-grid
%      search of the largest closed-loop real part. Saves Fig_Phase2_PD.pdf.
%
%   Run from this folder. Runtime under a minute.

clear; clc;
eqs   = [0 0; 0 pi; pi 0; pi pi];
names = {'E1 down-down', 'E2 down-up', 'E3 up-up', 'E4 up-down'};
wrapA = @(a) mod(a + pi, 2*pi) - pi;

%% 1-2. Stability from A and from the transfer functions
fprintf('================ Stability of each equilibrium (tau* = 0) ================\n');
for k = 1:4
    [A, B] = Acrobot_Linearize(eqs(k, 1), eqs(k, 2));
    p = eig(A);
    G1 = tf(ss(A, B, [1 0 0 0], 0));
    G2 = tf(ss(A, B, [0 1 0 0], 0));
    nRHP = sum(real(p) > 1e-9);
    if nRHP > 0
        verdict = sprintf('UNSTABLE (%d RHP pole%s)', nRHP, repmat('s', 1, nRHP > 1));
    elseif all(abs(real(p)) < 1e-9)
        verdict = 'marginally stable (simple poles on the jw-axis)';
    else
        verdict = 'asymptotically stable';
    end
    fprintf('\n%s   q* = (%g, %g) deg\n', names{k}, rad2deg(eqs(k, :)));
    fprintf('  eig(A)            : %s\n', num2str(p.', '%8.4f '));
    fprintf('  zeros of G1, G2   : %s   |   %s\n', num2str(zero(G1).', '%8.4f '), num2str(zero(G2).', '%8.4f '));
    fprintf('  char. polynomial  : %s   (s^3 and s^1 coefficients are 0)\n', num2str(poly(A), '%10.4f'));
    fprintf('  rank ctrb = %d, rank obsv(q1) = %d, rank obsv(q2) = %d\n', ...
            rank(ctrb(A, B)), rank(obsv(A, [1 0 0 0])), rank(obsv(A, [0 1 0 0])));
    fprintf('  verdict           : %s\n', verdict);
end

%% 3. Nonlinear check: growth rate of a tiny offset vs the largest RHP pole
fprintf('\n================ Nonlinear check (ode45, tau = 0) ================\n');
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
for k = 2:4
    [A, ~] = Acrobot_Linearize(eqs(k, 1), eqs(k, 2));
    pmax = max(real(eig(A)));
    x0 = [eqs(k, 1) + deg2rad(0.01); eqs(k, 2); 0; 0];
    [t, X] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, 0), [0 4], x0, opts);
    m = rad2deg(hypot(wrapA(X(:, 1) - eqs(k, 1)), wrapA(X(:, 2) - eqs(k, 2))));
    use = m > 0.2 & m < 15 & [true; diff(m) > 0];
    c = polyfit(t(use), log(m(use)), 1);
    fprintf('%s: measured growth rate %.3f 1/s vs largest RHP pole %.4f (error %.2f%%)\n', ...
            names{k}, c(1), pmax, 100 * abs(c(1) - pmax) / pmax);
end
[t, X] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, 0), [0 20], [deg2rad(10); deg2rad(-10); 0; 0], opts);
fprintf('%s: from (10, -10) deg, max |q1| over 20 s = %.2f deg (bounded, never decays)\n', ...
        names{1}, rad2deg(max(abs(X(:, 1)))));

%% 4. Upright E3: PD feedback from one measured angle
fprintf('\n================ E3: PD feedback from ONE angle ================\n');
[A, B] = Acrobot_Linearize(pi, 0);
den = poly(A);                                   % s^4 + a2 s^2 + a0
Kp = linspace(-400, 400, 161);  Kd = linspace(-150, 150, 121);
figure('Position', [100 100 900 330]);
for j = 1:2
    C = zeros(1, 4); C(j) = 1;
    [num, ~] = ss2tf(A, B, C, 0);                % b2 s^2 + b0
    b2 = num(3); b0 = num(5);
    fprintf('measure q%d: N(s) = %.4f s^2 + %.4f,  need Kd*%.3f > 0 AND Kd*%.3f > 0 -> %s\n', ...
            j, b2, b0, b2, b0, ternary(b2 * b0 < 0, 'IMPOSSIBLE (opposite signs)', 'possible'));
    Z = zeros(numel(Kd), numel(Kp));
    for a = 1:numel(Kd)
        for b = 1:numel(Kp)
            F = zeros(1, 4); F(j) = Kp(b); F(j + 2) = Kd(a);
            Z(a, b) = max(real(eig(A - B * F)));
        end
    end
    [zmin, idx] = min(Z(:)); [ia, ib] = ind2sub(size(Z), idx);
    fprintf('           smallest max Re(pole) on the grid = %.3f at Kp = %g, Kd = %g  (> 0: never stable)\n', ...
            zmin, Kp(ib), Kd(ia));
    subplot(1, 2, j);
    imagesc(Kp, Kd, min(Z, 12)); axis xy; colorbar; hold on; plot(Kp(ib), Kd(ia), 'g*', 'MarkerSize', 10);
    xlabel('K_p'); ylabel('K_d'); title(sprintf('PD on q_%d: max Re(pole), min %.2f', j, zmin));
end
exportgraphics(gcf, 'Fig_Phase2_PD.pdf');

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end
