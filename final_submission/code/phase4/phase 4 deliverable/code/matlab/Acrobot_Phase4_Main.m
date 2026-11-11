%% ACROBOT_PHASE4_MAIN  Phase 4: swing-up from hanging to the upright, |tau| <= 20 N m.
%
%   1. LQR at the upright E3 (Phase 3 design: Q = I, R = 0.03).
%   2. Hybrid swing-up -> LQR (Acrobot_Phase4_Controller) from three starts,
%      fixed-step RK4 (1 ms; the controller switches, so a fixed step is used).
%      Prints the catch time and final deviation; saves Fig_Phase4_SwingUp.pdf.
%   3. Robustness: 40 random starts near hanging (rng(0)), chosen gains vs
%      the textbook gains [50 5 10].  Set DO_ROBUST = false to skip (slowest part).
%
%   Run from this folder. Runtime about 1-3 minutes.

clear; clc;
DO_ROBUST = true;
gains = [20 2 3];  c_switch = 5;
[A, B] = Acrobot_Linearize(pi, 0);
[K, S] = lqr(A, B, eye(4), 0.03);
dt = 1e-3;
wrapA = @(a) mod(a + pi, 2*pi) - pi;
fprintf('LQR at E3: K = [%s]\n', sprintf('%.2f ', K));

%% 2. Three swing-ups
starts = [3 0; 0.5 0; -10 20];          % offsets from hanging [deg]
Tsim = [30 30 30];
figure('Position', [80 80 1150 330]);
for r = 1:size(starts, 1)
    x = [deg2rad(starts(r, :))'; 0; 0];
    n = round(Tsim(r) / dt);
    X = zeros(4, n); U = zeros(1, n); L = false(1, n); Ev = zeros(1, n);
    tcatch = NaN;
    for i = 1:n
        [tau, isL] = Acrobot_Phase4_Controller(x, K, S, gains, c_switch);
        if isL && isnan(tcatch), tcatch = (i - 1) * dt; end
        x = rk4(x, tau, dt);
        X(:, i) = x; U(i) = tau; L(i) = isL;
        Ev(i) = 0.5*[x(3) x(4)]*[2.663+2*cos(x(2)) 1.33+cos(x(2)); 1.33+cos(x(2)) 1.33]*[x(3); x(4)] ...
                - 14.715*cos(x(1)) - 9.81*cos(x(1) + x(2));
    end
    e = rad2deg([wrapA(x(1) - pi), wrapA(x(2))]);
    fprintf('start (%5.1f, %5.1f) deg: first catch %5.2f s, final deviation from upright (%7.3f, %7.3f) deg, saturated %4.1f%%\n', ...
            starts(r, :), tcatch, e, 100 * mean(abs(U) >= 20 - 1e-9));
    if r == 1
        t = (1:n) * dt;
        subplot(1, 3, 1); plot(t, rad2deg(X(1, :)), t, rad2deg(X(2, :))); yline(180, ':');
        xlabel('time [s]'); ylabel('[deg]'); legend('q_1', 'q_2', 'Location', 'best'); title('angles');
        subplot(1, 3, 2); plot(t, Ev); yline(24.525, ':'); xlabel('time [s]'); ylabel('E [J]'); title('energy pumped to E_{top}');
        subplot(1, 3, 3); plot(t, U); yline([-20 20], ':'); xlabel('time [s]'); ylabel('\tau [N m]'); title('torque');
    end
end
exportgraphics(gcf, 'Fig_Phase4_SwingUp.pdf');

%% 3. Robustness from 40 random starts
if DO_ROBUST
    rng(0);
    X0 = [ -0.6 + 1.2*rand(2, 40); -0.5 + rand(2, 40) ];
    X0(:, 1:3) = [0.01 -0.01 0.05; 0 0 0; 0 0 0; 0 0 0];
    for G = {gains, [50 5 10]}
        g = G{1}; okN = 0;
        for k = 1:40
            x = X0(:, k);
            for i = 1:round(40 / 2e-3)
                x = rk4(x, Acrobot_Phase4_Controller(x, K, S, g, c_switch), 2e-3);
            end
            e = [wrapA(x(1) - pi), wrapA(x(2))];
            okN = okN + (all(abs(e) < deg2rad(1)) && all(abs(x(3:4)) < 0.05));
        end
        fprintf('gains [%g %g %g]: swung up and held from %d of 40 random starts\n', g, okN);
    end
end

function x = rk4(x, tau, h)
    f = @(y) Acrobot_Nonlinear_Dynamics(0, y, tau);
    k1 = f(x); k2 = f(x + h/2*k1); k3 = f(x + h/2*k2); k4 = f(x + h*k3);
    x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
