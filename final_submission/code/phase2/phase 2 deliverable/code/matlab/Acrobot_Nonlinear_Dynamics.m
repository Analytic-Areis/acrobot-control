function dx = Acrobot_Nonlinear_Dynamics(t, x, tau)
%% ACROBOT_NONLINEAR_DYNAMICS
% Evaluates the exact first-principles nonlinear state derivative for the Acrobot.
%
% Syntax:
%   dx = Acrobot_Nonlinear_Dynamics(t, x, tau)
%
% Inputs:
%   t   - Current simulation time [s]
%   x   - State vector [q1; q2; dq1; dq2] (4x1) [rad, rad/s]
%   tau - Applied motor torque at Joint 2 (elbow) [N*m] (scalar or function handle)
%
% Outputs:
%   dx  - State time-derivative [dq1; dq2; ddq1; ddq2] (4x1)
%
% Note:
%   Solves M(q)*ddq = tau_g + B*tau - C*dq directly via Gaussian elimination
%   (backslash '\') rather than computing M^-1 explicitly, per Phase1_helper.pdf.

    % 1. System Parameters
    m1 = 1.0; m2 = 1.0;
    l1 = 1.0; l2 = 2.0;
    lc1 = 0.5; lc2 = 1.0;
    I1 = 0.083; I2 = 0.33;
    g = 9.81;
    tau_max = 20.0;

    % 2. Parse State
    q1  = x(1);
    q2  = x(2);
    dq1 = x(3);
    dq2 = x(4);
    dq  = [dq1; dq2];

    % 3. Evaluate Actuator Torque with Saturation
    if isa(tau, 'function_handle')
        tau_cmd = tau(t, x);
    else
        tau_cmd = tau;
    end
    tau_sat = max(min(tau_cmd, tau_max), -tau_max);
    B_tau = [0; tau_sat];

    % 4. Symmetric Positive-Definite Inertia Matrix M(q)
    c2 = cos(q2);
    s2 = sin(q2);
    M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2);
    M12 = I2 + m2*(lc2^2 + l1*lc2*c2);
    M21 = M12;
    M22 = I2 + m2*lc2^2;
    M = [M11, M12; M21, M22];

    % 5. Coriolis / Centrifugal Matrix C(q, dq)
    C11 = -2 * m2 * l1 * lc2 * s2 * dq2;
    C12 = -m2 * l1 * lc2 * s2 * dq2;
    C21 =  m2 * l1 * lc2 * s2 * dq1;
    C22 = 0;
    C = [C11, C12; C21, C22];

    % 6. Gravitational Torque Vector tau_g(q) = -grad(U)
    s1   = sin(q1);
    s12  = sin(q1 + q2);
    tau_g1 = -g * ((m1*lc1 + m2*l1)*s1 + m2*lc2*s12);
    tau_g2 = -g * (m2*lc2*s12);
    tau_g = [tau_g1; tau_g2];

    % 7. Solve for Acceleration ddq directly using backslash '\'
    %    M(q) * ddq = tau_g + B*tau - C(q, dq)*dq
    ddq = M \ (tau_g + B_tau - C*dq);

    % 8. Assemble State Derivative dx/dt
    dx = [dq; ddq];
end
