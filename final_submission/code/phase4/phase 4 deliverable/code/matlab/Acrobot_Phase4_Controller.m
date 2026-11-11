function [tau, isLQR] = Acrobot_Phase4_Controller(x, K, S, gains, c_switch)
%ACROBOT_PHASE4_CONTROLLER  Hybrid swing-up -> LQR (same law as the Python model).
%   x        state [q1; q2; dq1; dq2] (absolute angles, q1 from straight down)
%   K, S     LQR gain and Riccati matrix at the upright E3 (Phase 3: Q = I, R = 0.03)
%   gains    [k1 k2 k3] swing-up gains (chosen: [20 2 3])
%   c_switch catch threshold on x~' S x~ (chosen: 5)
%
%   Swing-up (energy pumping, collocated partial feedback linearization):
%     v   = -k1*q2~ - k2*dq2 + k3*dq1*(E - E_top),  E_top = 24.525 J
%     tau = (M22 - M21*M12/M11)*v + h2 - M21*h1/M11,  h = C*dq - tau_g
%   Catch: tau = -K*x~ when x~' S x~ < c_switch (x~ measured from E3).
%   The result is saturated to |tau| <= 20 N m.

    wrapA = @(a) mod(a + pi, 2*pi) - pi;
    q1 = x(1); q2 = x(2); d1 = x(3); d2 = x(4);
    e = [wrapA(q1 - pi); wrapA(q2); d1; d2];
    isLQR = (e' * S * e) < c_switch;
    if isLQR
        tau = -K * e;
    else
        c2 = cos(q2); s2 = sin(q2);
        m11 = 2.663 + 2*c2; m12 = 1.33 + c2; m22 = 1.33;
        E = 0.5*(m11*d1^2 + 2*m12*d1*d2 + m22*d2^2) - 14.715*cos(q1) - 9.81*cos(q1 + q2);
        h1 = -s2*(2*d1*d2 + d2^2) + 9.81*(1.5*sin(q1) + sin(q1 + q2));
        h2 = s2*d1^2 + 9.81*sin(q1 + q2);
        v = -gains(1)*wrapA(q2) - gains(2)*d2 + gains(3)*d1*(E - 24.525);
        tau = (m22 - m12^2/m11)*v + h2 - m12*h1/m11;
    end
    tau = max(min(tau, 20), -20);
end
