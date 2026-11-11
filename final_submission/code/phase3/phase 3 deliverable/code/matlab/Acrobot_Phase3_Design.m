function D = Acrobot_Phase3_Design()
%ACROBOT_PHASE3_DESIGN  The Phase 3 controllers (same choices as the report).
%   D(k) has fields: name, eq (1..4), qs (equilibrium), A, B, K, poles,
%   and for LQR designs S (Riccati solution) and c (largest level set of
%   V = x'Sx on which |K x| <= tau_max, i.e. the linear loop never saturates).
%
%   E3 upright : LQR, Q = I, R = 0.03          (knee of the R sweep)
%   E3 upright : pole placement at -3,-4,-5,-6 (comparison)
%   E2, E4     : LQR, Q = I, R = 0.1
%   E1         : PD on the elbow angle only, Kp = -5, Kd = 0.4

    tau_max = 20;
    Q = eye(4);
    spec = { 'E3 LQR',            3, 'lqr',   0.03;
             'E3 pole placement', 3, 'place', [-3 -4 -5 -6];
             'E2 LQR',            2, 'lqr',   0.1;
             'E4 LQR',            4, 'lqr',   0.1;
             'E1 elbow PD',       1, 'pd',    [-5 0.4] };
    eqs = [0 0; 0 pi; pi 0; pi pi];

    D = struct([]);
    for k = 1:size(spec, 1)
        eq = spec{k, 2};
        qs = eqs(eq, :);
        [A, B] = Acrobot_Linearize(qs(1), qs(2));
        S = []; c = [];
        switch spec{k, 3}
            case 'lqr'
                Rw = spec{k, 4};
                [K, S] = lqr(A, B, Q, Rw);
                c = tau_max^2 / (K * (S \ K'));
            case 'place'
                K = place(A, B, spec{k, 4});
            case 'pd'
                g = spec{k, 4};
                K = [0, g(1), 0, g(2)];      % tau = -Kp*q2~ - Kd*dq2
        end
        D(k).name = spec{k, 1};
        D(k).eq = eq;
        D(k).qs = qs;
        D(k).A = A;
        D(k).B = B;
        D(k).K = K;
        D(k).S = S;
        D(k).c = c;
        D(k).poles = eig(A - B*K);
    end
end
