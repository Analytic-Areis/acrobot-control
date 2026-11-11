%% ACROBOT_PHASE3_MAIN  Phase 3: control around the equilibria with |tau| <= 20 N m.
%
%   1. Designs every controller (Acrobot_Phase3_Design) and prints K and the
%      closed-loop poles.
%   2. Prints the guaranteed (never-saturating) ellipsoid for each LQR design.
%   3. Simulates the nonlinear model (ode45, Acrobot_Nonlinear_Dynamics, which
%      saturates tau at 20 N m) from the initial offsets used in the report.
%   4. Maps the region of attraction (which initial offsets are recovered) on a
%      coarse grid and saves Fig_Phase3_ROA.pdf.
%
%   Run from this folder. Runtime about 1-3 minutes (set DO_MAPS = false to skip 4).

clear; clc;
DO_MAPS = true;
D = Acrobot_Phase3_Design();
wrapA = @(a) mod(a + pi, 2*pi) - pi;
ctrl = @(K, qs) @(t, x) -K * [wrapA(x(1) - qs(1)); wrapA(x(2) - qs(2)); x(3); x(4)];
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);

%% 1-2. Designs
fprintf('================ Phase 3 designs ================\n');
for k = 1:numel(D)
    fprintf('\n%s  (q* = [%g %g] deg)\n', D(k).name, rad2deg(D(k).qs));
    fprintf('  K      = [%s]\n', sprintf('%9.2f ', D(k).K));
    p = sort(D(k).poles, 'ComparisonMethod', 'real');
    fprintf('  poles  = %s\n', sprintf('%s  ', num2str(p.', '%.2f ')));
    if ~isempty(D(k).S)
        S = D(k).S; c = D(k).c;
        fprintf('  never-saturating ellipsoid: c = %.4f -> |dq1| <= %.2f deg (dq2 = 0), |dq2| <= %.2f deg (dq1 = 0)\n', ...
                c, rad2deg(sqrt(c / S(1,1))), rad2deg(sqrt(c / S(2,2))));
    end
end

%% 3. Nonlinear closed-loop runs (offsets in deg)
runs = { 1, [2 0],    5;     % E3 LQR, small offset
         1, [5.5 0],  8;     % E3 LQR, just inside the q1 limit (6 deg)
         1, [7.5 0],  8;     % E3 LQR, just outside (falls)
         1, [-10 25], 6;     % E3 LQR, along the centre-of-mass line
         2, [2 0],    5;     % E3 pole placement
         3, [60 90],  8;     % E2 LQR
         4, [20 30],  8;     % E4 LQR
         5, [60 -60], 25 };  % E1 elbow PD
fprintf('\n================ Nonlinear runs (ode45, |tau| <= 20) ================\n');
for r = 1:size(runs, 1)
    k = runs{r, 1}; d0 = deg2rad(runs{r, 2}); T = runs{r, 3};
    x0 = [D(k).qs(1) + d0(1); D(k).qs(2) + d0(2); 0; 0];
    u = ctrl(D(k).K, D(k).qs);
    [t, X] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, u), [0 T], x0, opts);
    tau = zeros(size(t));
    for i = 1:numel(t), tau(i) = max(min(u(t(i), X(i,:)'), 20), -20); end
    e = rad2deg([wrapA(X(end,1) - D(k).qs(1)), wrapA(X(end,2) - D(k).qs(2))]);
    held = all(abs(e) < 1) && all(abs(X(end, 3:4)) < 0.05);
    fprintf('%-18s from (%6.1f, %6.1f) deg: end (%8.3f, %8.3f) deg, peak |tau| = %5.2f  -> %s\n', ...
            D(k).name, runs{r, 2}, e, max(abs(tau)), ternary(held, 'held', 'NOT held'));
end

%% 4. Region of attraction maps (coarse grid; the report uses a finer one)
if DO_MAPS
    cases = { 1, 40, 21; 3, 180, 19; 4, 180, 19 };
    figure('Position', [100 100 1100 340]);
    for c = 1:size(cases, 1)
        k = cases{c, 1}; lim = cases{c, 2}; n = cases{c, 3};
        g = linspace(-lim, lim, n);
        ok = false(n);
        u = ctrl(D(k).K, D(k).qs);
        for i = 1:n
            for j = 1:n
                x0 = [D(k).qs(1) + deg2rad(g(j)); D(k).qs(2) + deg2rad(g(i)); 0; 0];
                [~, X] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, u), [0 10], x0);
                e = [wrapA(X(end,1) - D(k).qs(1)), wrapA(X(end,2) - D(k).qs(2))];
                ok(i, j) = all(abs(e) < deg2rad(1)) && all(abs(X(end, 3:4)) < 0.05);
            end
        end
        subplot(1, 3, c);
        imagesc(g, g, ok); axis xy equal tight; colormap([1 .89 .89; .73 .97 .82]);
        title(sprintf('%s: %.0f%% recovered', D(k).name, 100 * mean(ok(:))));
        xlabel('\Delta q_1(0) [deg]'); ylabel('\Delta q_2(0) [deg]');
        fprintf('ROA %-10s (grid +-%d deg, %dx%d): %.1f%% of initial offsets recovered\n', ...
                D(k).name, lim, n, n, 100 * mean(ok(:)));
    end
    exportgraphics(gcf, 'Fig_Phase3_ROA.pdf');
end

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end
