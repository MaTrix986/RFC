function [tau, dx, out] = redundant_impedance_control(robot, q, dq, ref, h, x, cfg)
    % 四元数内环阻抗 + RFC (2.78)、(2.83)-(2.91)。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    n = numel(q);
    assert(n > 6, '冗余接口要求多于六个关节。');

    [c, dx] = impedance_reference(x, ref, h, cfg);
    a = rfc_motion_acceleration(s, c, cfg);

    B = massMatrix(robot, q);
    C = coriolis_matrix(robot, q, dq);
    dB = C + C.';              % Christoffel C：Bdot-2C 反对称

    X = B \ s.J.';
    A = s.J*X;
    assert(rcond(A) > 1e-10, '动态一致伪逆接近奇异。');
    Jdag = X/A;

    dX = B \ (s.dJ.' - dB*X);
    dA = s.dJ*X + s.J*dX;
    dJdag = (dX - Jdag*dA)/A;  % 解析求导，不用时间差分
    N = eye(n) - Jdag*s.J;

    g = cfg.Wrest*(q - cfg.qrest);
    beta = cfg.kbeta*(B \ g);
    dbeta = B \ (cfg.kbeta*cfg.Wrest*dq - dB*beta);
    en = N*(beta - dq);
    an = N*(dbeta - dJdag*s.J*(beta - dq) ...
                  + B \ (cfg.Kn*en + C*en));

    alpha = Jdag*(a - s.dJ*dq) + an;
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;

    out.c = c;
    out.a = a;
    out.alpha = alpha;
    out.an = an;
    out.N = N;
    out.en = en;
    out.w = (q - cfg.qrest).'*g/2;

    dN = -dJdag*s.J - Jdag*s.dJ;
    den = dN*(beta - dq) + N*(dbeta - alpha);
    out.dV = en.'*B*den + en.'*dB*en/2;
end

function C = coriolis_matrix(robot, q, dq)
    % 工具箱给出 c(q,v)=C(q,v)v；极化二次型得到 Christoffel C。
    % C(q,dq)e_j=[c(q,dq+e_j)-c(q,dq-e_j)]/4，精确代数恒等式。
    n = numel(q);
    C = zeros(n);

    for j = 1:n
        ej = zeros(n, 1);
        ej(j) = 1;
        C(:,j) = (velocityProduct(robot, q, dq + ej) ...
                  - velocityProduct(robot, q, dq - ej))/4;
    end
end
