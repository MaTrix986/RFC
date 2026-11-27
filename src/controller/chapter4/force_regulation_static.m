function [tau, dx] = force_regulation_static(robot, q, dq, fd, mud, h, x, cfg)
    % 静态模型补偿：(4.1)-(4.6)。x=[int(df);int(dmu)]。
    s = rfc_state(robot, q, dq, cfg.tip);
    [phi, T] = rfc_xyz(s.R);
    df = fd - h(1:3);
    dmu = mud - h(4:6);

    pc = cfg.KPp \ (cfg.KFp*df + cfg.KIp*x(1:3));
    phic = cfg.KPo \ (cfg.KFo*dmu + cfg.KIo*x(4:6));
    gamma = [cfg.KPp*(pc - s.p) + fd;
             T.' \ (cfg.KPo*(phic - phi)) + mud];

    tau = s.J.'*gamma - cfg.Kq*dq + gravityTorque(robot, q);
    if norm(h(1:3)) > 1e-3
        dx = [df; dmu];
    else
        dx = zeros(6, 1);
    end
end
