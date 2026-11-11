%% BUILD_SIMULINK_MODELS  Create the acrobot Simulink models (needs Simulink).
%
%   Every model contains the same nonlinear plant: a MATLAB Function block
%   computing x_dot = f(x, tau) from the given equations of motion, fed back
%   through an Integrator whose initial condition is the workspace variable x0.
%   The torque is always saturated at +-20 N m before it enters the plant.
%
%   Models (saved next to this script, and copied into the phase folders):
%     Acrobot_OpenLoop.slx        Phase 2: tau = 0, nonlinear plant vs linear State-Space block
%     Acrobot_SingleAnglePD.slx   Phase 2: PD from ONE measured angle at the upright
%     Acrobot_LQR.slx             Phase 3: full-state feedback tau = sat(-K x~) at any equilibrium
%     Acrobot_SwingUp.slx         Phase 4: energy-pumping swing-up -> LQR catch (hybrid)
%
%   Run from this folder:  build_simulink_models   then   run_simulink_models

here = fileparts(mfilename('fullpath'));
cd(here);
models = {'Acrobot_OpenLoop', 'Acrobot_SingleAnglePD', 'Acrobot_LQR', 'Acrobot_SwingUp'};
for k = 1:numel(models)
    if bdIsLoaded(models{k}), close_system(models{k}, 0); end
    if exist([models{k} '.slx'], 'file'), delete([models{k} '.slx']); end
end

%% ---------------------------------------------------------- shared code
PLANT = strjoin({
    'function dx = dynamics(x, tau)'
    '%#codegen'
    '% Given acrobot model: M(q) qdd + C(q,qd) qd = tau_g(q) + B tau'
    'q1 = x(1); q2 = x(2); d1 = x(3); d2 = x(4);'
    'c2 = cos(q2); s2 = sin(q2);'
    'M = [2.663 + 2*c2, 1.33 + c2; 1.33 + c2, 1.33];'
    'Cqd = [-s2*(2*d1*d2 + d2^2); s2*d1^2];'
    'tg = -9.81*[1.5*sin(q1) + sin(q1 + q2); sin(q1 + q2)];'
    'qdd = M \ (tg + [0; tau] - Cqd);'
    'dx = [d1; d2; qdd(1); qdd(2)];'
    }, newline);

PD = strjoin({
    'function tau = pd_one_angle(x, qs, Kp, Kd, j)'
    '%#codegen'
    '% PD on ONE measured joint angle: tau = -(Kp e + Kd e_dot)'
    'e = mod(x(j) - qs(j) + pi, 2*pi) - pi;'
    'tau = -(Kp*e + Kd*x(j + 2));'
    }, newline);

LQR = strjoin({
    'function tau = full_state(x, qs, K)'
    '%#codegen'
    '% Full-state feedback tau = -K x~, x~ = deviation from the equilibrium qs'
    'e = [mod(x(1) - qs(1) + pi, 2*pi) - pi; mod(x(2) - qs(2) + pi, 2*pi) - pi; x(3); x(4)];'
    'tau = -dot(K(:), e);'
    }, newline);

SWING = strjoin({
    'function [tau, isLQR] = swing_up(x, K, S, gains, c_switch)'
    '%#codegen'
    '% Hybrid controller: energy-pumping swing-up (partial feedback'
    '% linearization) until x~''S x~ < c_switch, then the upright LQR.'
    'q1 = x(1); q2 = x(2); d1 = x(3); d2 = x(4);'
    'e = [mod(q1 - pi + pi, 2*pi) - pi; mod(q2 + pi, 2*pi) - pi; d1; d2];'
    'isLQR = double((e''*S*e) < c_switch);'
    'if isLQR > 0'
    '    tau = -dot(K(:), e);'
    'else'
    '    c2 = cos(q2); s2 = sin(q2);'
    '    m11 = 2.663 + 2*c2; m12 = 1.33 + c2; m22 = 1.33;'
    '    E = 0.5*(m11*d1^2 + 2*m12*d1*d2 + m22*d2^2) - 14.715*cos(q1) - 9.81*cos(q1 + q2);'
    '    h1 = -s2*(2*d1*d2 + d2^2) + 9.81*(1.5*sin(q1) + sin(q1 + q2));'
    '    h2 = s2*d1^2 + 9.81*sin(q1 + q2);'
    '    v = -gains(1)*(mod(q2 + pi, 2*pi) - pi) - gains(2)*d2 + gains(3)*d1*(E - 24.525);'
    '    tau = (m22 - m12^2/m11)*v + h2 - m12*h1/m11;'
    'end'
    }, newline);

%% ------------------------------------------------------------- models
build_model('Acrobot_OpenLoop', PLANT, '', {}, true);
build_model('Acrobot_SingleAnglePD', PLANT, PD, {'qs', 'Kp', 'Kd', 'j_meas'}, false);
build_model('Acrobot_LQR', PLANT, LQR, {'qs', 'K'}, false);
build_model('Acrobot_SwingUp', PLANT, SWING, {'K', 'S', 'gains', 'c_switch'}, false);

%% copy each model into the phase folder it belongs to
dest = {fullfile(here, '..', 'phase2', 'phase 2 deliverable', 'code', 'simulink'), {'Acrobot_OpenLoop', 'Acrobot_SingleAnglePD'};
        fullfile(here, '..', 'phase3', 'phase 3 deliverable', 'code', 'simulink'), {'Acrobot_LQR'};
        fullfile(here, '..', 'phase4', 'phase 4 deliverable', 'code', 'simulink'), {'Acrobot_SwingUp'}};
for r = 1:size(dest, 1)
    if ~exist(dest{r, 1}, 'dir'), mkdir(dest{r, 1}); end
    for m = dest{r, 2}
        copyfile([m{1} '.slx'], dest{r, 1});
    end
end
fprintf('built %d models in %s (and copied them into the phase folders)\n', numel(models), here);


%% =====================================================================
function build_model(name, plantCode, ctrlCode, ctrlParams, openLoop)
% Plant: [tau] -> Saturation -> MATLAB Function dynamics(x, tau) -> Integrator -> x
    new_system(name);
    open_system(name);
    set_param(name, 'StopTime', 'Tstop', 'Solver', 'ode45', 'MaxStep', '1e-3', ...
              'RelTol', '1e-7', 'AbsTol', '1e-9');

    add_block('simulink/User-Defined Functions/MATLAB Function', [name '/dynamics'], 'Position', [420 80 540 160]);
    set_script([name '/dynamics'], plantCode);
    add_block('simulink/Continuous/Integrator', [name '/x'], 'InitialCondition', 'x0', 'Position', [600 100 640 140]);
    add_block('simulink/Discontinuities/Saturation', [name '/tau limit'], ...
              'UpperLimit', '20', 'LowerLimit', '-20', 'Position', [320 120 360 150]);
    add_block('simulink/Sinks/To Workspace', [name '/x_out'], 'VariableName', 'x_nl', ...
              'SaveFormat', 'Timeseries', 'Position', [720 60 790 90]);
    add_block('simulink/Sinks/To Workspace', [name '/tau_out'], 'VariableName', 'tau_log', ...
              'SaveFormat', 'Timeseries', 'Position', [420 200 490 230]);
    add_block('simulink/Sinks/Scope', [name '/angles'], 'NumInputPorts', '1', 'Position', [720 150 750 180]);
    add_block('simulink/Signal Routing/Selector', [name '/q'], 'InputPortWidth', '4', ...
              'Indices', '[1 2]', 'Position', [670 155 700 175]);

    add_line(name, 'dynamics/1', 'x/1', 'autorouting', 'on');
    add_line(name, 'x/1', 'dynamics/1', 'autorouting', 'on');
    add_line(name, 'tau limit/1', 'dynamics/2', 'autorouting', 'on');
    add_line(name, 'tau limit/1', 'tau_out/1', 'autorouting', 'on');
    add_line(name, 'x/1', 'x_out/1', 'autorouting', 'on');
    add_line(name, 'x/1', 'q/1', 'autorouting', 'on');
    add_line(name, 'q/1', 'angles/1', 'autorouting', 'on');

    if openLoop
        % tau = 0, plus the linearized model next to the nonlinear one
        add_block('simulink/Sources/Constant', [name '/tau = 0'], 'Value', '0', 'Position', [220 120 260 150]);
        add_line(name, 'tau = 0/1', 'tau limit/1', 'autorouting', 'on');
        add_block('simulink/Continuous/State-Space', [name '/linear model'], 'A', 'A', 'B', 'B', ...
                  'C', 'eye(4)', 'D', 'zeros(4,1)', 'InitialCondition', 'x0_lin', 'Position', [420 280 540 330]);
        add_line(name, 'tau = 0/1', 'linear model/1', 'autorouting', 'on');
        add_block('simulink/Sinks/To Workspace', [name '/x_lin_out'], 'VariableName', 'x_lin', ...
                  'SaveFormat', 'Timeseries', 'Position', [720 290 790 320]);
        add_line(name, 'linear model/1', 'x_lin_out/1', 'autorouting', 'on');
    else
        % controller: MATLAB Function block fed by x and by Constant blocks holding the gains
        add_block('simulink/User-Defined Functions/MATLAB Function', [name '/controller'], 'Position', [120 100 240 200]);
        set_script([name '/controller'], ctrlCode);
        add_line(name, 'x/1', 'controller/1', 'autorouting', 'on');
        for p = 1:numel(ctrlParams)
            blk = [name '/' ctrlParams{p}];
            add_block('simulink/Sources/Constant', blk, 'Value', ctrlParams{p}, ...
                      'Position', [20 100 + 40*p 70 120 + 40*p]);
            add_line(name, [ctrlParams{p} '/1'], sprintf('controller/%d', p + 1), 'autorouting', 'on');
        end
        add_line(name, 'controller/1', 'tau limit/1', 'autorouting', 'on');
        if strcmp(name, 'Acrobot_SwingUp')
            add_block('simulink/Sinks/To Workspace', [name '/mode_out'], 'VariableName', 'lqr_on', ...
                      'SaveFormat', 'Timeseries', 'Position', [300 230 370 260]);
            add_line(name, 'controller/2', 'mode_out/1', 'autorouting', 'on');
        end
    end
    save_system(name);
    close_system(name);
end

function set_script(blockPath, code)
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    chart.Script = code;
end
