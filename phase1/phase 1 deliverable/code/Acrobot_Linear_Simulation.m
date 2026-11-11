%% =========================================================================
%  ACROBOT PROJECT - PHASE 1: CHECK OF THE LINEARIZATION BY SIMULATION
%  =========================================================================
%  Compares the free response (tau = 0) of the nonlinear model with that of
%  the linearized model near E1 = (0,0) and E3 = (pi,0), and checks that the
%  nonlinear model conserves energy when tau = 0.
%  Saves a comparison plot: Fig_linear_vs_nonlinear.pdf (numbers in Sec. V-C of the report)
%
%  Uses: Acrobot_Nonlinear_Dynamics.m, Acrobot_Linearize.m
%  =========================================================================
clear; clc; close all;

opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
cases = struct('name', {'E1', 'E3'}, ...
               'qs',   {[0; 0], [pi; 0]}, ...
               'dx0',  {[0.1; -0.1; 0; 0], [1e-3; 0; 0; 0]}, ...
               'T',    {8, 1.2});

fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 3.45 2.4]);
tl  = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:2
    c  = cases(k);
    xs = [c.qs; 0; 0];
    A  = Acrobot_Linearize(c.qs(1), c.qs(2));
    t  = linspace(0, c.T, 1201).';

    % Nonlinear response (tau = 0)
    [~, xn] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, 0), t, xs + c.dx0, opts);
    dxn = xn.' - xs;                                  % deviation from equilibrium
    % Linear response dx(t) = expm(A t) dx0
    dxl = zeros(4, numel(t));
    for i = 1:numel(t), dxl(:, i) = expm(A*t(i))*c.dx0; end

    err = max(abs(dxn(1:2, :) - dxl(1:2, :)), [], 2);
    fprintf('%s: max |nonlinear - linear| = %.2f deg (q1), %.2f deg (q2) over %.1f s\n', ...
            c.name, rad2deg(err(1)), rad2deg(err(2)), c.T);
    if k == 2
        i1  = find(abs(dxn(1, :)) > deg2rad(1), 1);
        i10 = find(abs(dxn(1, :)) > deg2rad(10), 1);
        fprintf('    at |dq1| = 1 deg  (t = %.2f s): relative error %.3f %%\n', ...
                t(i1), 100*abs(dxn(1,i1) - dxl(1,i1))/abs(dxn(1,i1)));
        if ~isempty(i10)
            fprintf('    at |dq1| = 10 deg (t = %.2f s): error %.2f deg\n', ...
                    t(i10), rad2deg(abs(dxn(1,i10) - dxl(1,i10))));
        end
    end

    nexttile; hold on; grid on; box on;
    plot(t, rad2deg(dxn(1,:)), '-',  'Color', [0.12 0.47 0.71], 'LineWidth', 1.2);
    plot(t, rad2deg(dxl(1,:)), '--k', 'LineWidth', 0.8);
    plot(t, rad2deg(dxn(2,:)), '-',  'Color', [0.90 0.38 0.00], 'LineWidth', 1.2);
    plot(t, rad2deg(dxl(2,:)), ':k',  'LineWidth', 0.8);
    xlim([0 c.T]); ylabel('[deg]');
    if k == 1
        title('(a) Near $E_1$: $\tilde q(0) = (0.1, -0.1)$ rad', 'Interpreter', 'latex', 'FontWeight', 'normal');
        legend({'$\tilde q_1$ nonlinear', '$\tilde q_1$ linear', '$\tilde q_2$ nonlinear', '$\tilde q_2$ linear'}, ...
               'Interpreter', 'latex', 'NumColumns', 2, 'FontSize', 6, 'Location', 'northoutside');
    else
        title('(b) Near $E_3$: $\tilde q_1(0) = 10^{-3}$ rad', 'Interpreter', 'latex', 'FontWeight', 'normal');
        ylim([-15 15]); xlabel('time [s]');
    end
end
exportgraphics(fig, 'Fig_linear_vs_nonlinear.pdf', 'ContentType', 'vector');

%% Energy conservation check (tau = 0): E = 1/2 dq' M dq + U must stay constant
x0 = [2; -1; 0.5; 1];
[~, xe] = ode45(@(t, x) Acrobot_Nonlinear_Dynamics(t, x, 0), linspace(0, 20, 2001), x0, opts);
E = zeros(size(xe, 1), 1);
for i = 1:size(xe, 1), E(i) = Acrobot_Energy(xe(i, :).'); end
fprintf('Energy check (tau = 0, 20 s): max relative drift = %.1e\n', max(abs(E - E(1)))/abs(E(1)));

function E = Acrobot_Energy(x)
    q1 = x(1); q2 = x(2); dq = x(3:4);
    M = [2.663 + 2*cos(q2), 1.33 + cos(q2); 1.33 + cos(q2), 1.33];
    E = 0.5*dq.'*M*dq - 9.81*(1.5*cos(q1) + cos(q1 + q2));
end
