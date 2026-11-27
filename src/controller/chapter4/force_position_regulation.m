function [tau, dx] = force_position_regulation(robot, q, dq, ref, fd, h, x, cfg)
    % 力/位置调节：(4.5)、(4.11)-(4.13)。x=int(df)。
    s = rfc_state(robot, q, dq, cfg.tip);
    [phi, T] = rfc_xyz(s.R);
    phid = rfc_xyz(ref.R);
    df = fd - h(1:3);
    pc = cfg.KPp \ (cfg.KFp*df + cfg.KIp*x);

    gamma = [cfg.KPp*(ref.p + pc - s.p) + fd;
             T.' \ (cfg.KPo*(phid - phi))];
    tau = s.J.'*gamma - cfg.Kq*dq + gravityTorque(robot, q);
    dx = df;
end
