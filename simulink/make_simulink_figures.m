%% MAKE_SIMULINK_FIGURES  Export the model diagrams and the Simulink trajectories used in the full report.
%
%   Run build_simulink_models first. Run this from the MATLAB desktop (not
%   "matlab -batch"): in batch mode Simulink leaves the block icons blank.
%   Writes
%     <final_submission>/report_source/figures/slx_<model>.png   model diagrams
%     results/*.csv                                   trajectories, read by compare_with_python.py

clear; clc;
here = fileparts(mfilename('fullpath'));
cd(here);
figdir = fullfile(here, '..', '..', 'report_source', 'figures');            % inside final_submission/code
if ~exist(figdir, 'dir'), figdir = fullfile(here, '..', 'final_submission', 'report_source', 'figures'); end
resdir = fullfile(here, 'results');
if ~exist(resdir, 'dir'), mkdir(resdir); end

%% 1. diagrams
models = {'Acrobot_OpenLoop', 'Acrobot_SingleAnglePD', 'Acrobot_LQR', 'Acrobot_SwingUp'};
tags   = {'openloop', 'pd', 'lqr', 'swingup'};
for k = 1:numel(models)
    open_system(models{k}); pause(2);
    print(['-s' models{k}], '-dpng', '-r300', fullfile(figdir, ['slx_' tags{k} '.png']));
    close_system(models{k}, 0);
end

%% 2. trajectories
% open loop near E1 and E3, nonlinear and linear from the same start
for c = {{'E1', [0; 0], [10; -10], 8}, {'E3', [pi; 0], [0.5; 0], 2.5}}
    c = c{1};                                                    %#ok<FXSET>
    [A, B] = linearize_at(c{2}(1), c{2}(2));                     %#ok<ASGLU>
    d0 = deg2rad(c{3}); Tstop = c{4};                            %#ok<NASGU>
    x0 = [c{2} + d0; 0; 0]; x0_lin = [d0; 0; 0];                 %#ok<NASGU>
    out = sim('Acrobot_OpenLoop');
    X = states(out.x_nl); L = states(out.x_lin);
    Lt = interp1(out.x_lin.Time, L, out.x_nl.Time);
    writematrix([out.x_nl.Time, X, Lt], fullfile(resdir, ['openloop_' c{1} '.csv']));
end

% LQR at the upright from three tilts
[A, B] = linearize_at(pi, 0);
[K, S] = lqr(A, B, eye(4), 0.03);
qs = [pi; 0]; Tstop = 6;                                         %#ok<NASGU>
for a = [2 5.5 7.5]
    x0 = [pi + deg2rad(a); 0; 0; 0];                             %#ok<NASGU>
    out = sim('Acrobot_LQR');
    tau = interp1(out.tau_log.Time, squeeze(out.tau_log.Data), out.x_nl.Time);
    writematrix([out.x_nl.Time, states(out.x_nl), tau], fullfile(resdir, sprintf('lqr_E3_%g.csv', a)));
end

% swing-up from 3 deg off hanging
gains = [20 2 3]; c_switch = 5; Tstop = 10;                      %#ok<NASGU>
x0 = [deg2rad(3); 0; 0; 0];                                      %#ok<NASGU>
out = sim('Acrobot_SwingUp');
tau = interp1(out.tau_log.Time, squeeze(out.tau_log.Data), out.x_nl.Time);
on  = interp1(out.lqr_on.Time, squeeze(out.lqr_on.Data), out.x_nl.Time, 'previous');
writematrix([out.x_nl.Time, states(out.x_nl), tau, on], fullfile(resdir, 'swingup_3.csv'));
writematrix([K; S], fullfile(resdir, 'lqr_K_S.csv'));
fprintf('diagrams -> %s\ntrajectories -> %s\n', figdir, resdir);


function [A, B] = linearize_at(q1s, q2s)
    c2 = cos(q2s); M = [2.663 + 2*c2, 1.33 + c2; 1.33 + c2, 1.33];
    cp = cos(q1s + q2s); Kg = -9.81*[1.5*cos(q1s) + cp, cp; cp, cp];
    A = [zeros(2), eye(2); M \ Kg, zeros(2)];
    B = [0; 0; M \ [0; 1]];
end

function X = states(ts)
% logged state as an N-by-4 array (Simulink stores a 4-vector as 4x1xN)
    X = ts.Data;
    if ndims(X) == 3, X = permute(X, [3 1 2]); end
end
