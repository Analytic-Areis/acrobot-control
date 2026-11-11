function [A, B, P, r] = Acrobot_Linearize(q1s, q2s)
%ACROBOT_LINEARIZE  Analytical linearization of the Acrobot at a zero-torque
%   equilibrium q* = (q1s, q2s), using
%       A = [0 I; P 0],  B = [0; r],  P = M(q*)^-1 K(q*),  r = M(q*)^-1 [0;1]
%   where K = d tau_g / dq evaluated at q*. (Coriolis terms vanish at dq* = 0.)
    c2  = cos(q2s);
    M   = [2.663 + 2*c2, 1.33 + c2; 1.33 + c2, 1.33];
    cp  = cos(q1s + q2s);
    K   = -9.81*[1.5*cos(q1s) + cp, cp; cp, cp];
    P   = M \ K;
    r   = M \ [0; 1];
    A   = [zeros(2), eye(2); P, zeros(2)];
    B   = [0; 0; r];
end
