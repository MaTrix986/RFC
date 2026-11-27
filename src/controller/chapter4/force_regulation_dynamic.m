function [tau, dx] = force_regulation_dynamic(robot, q, dq, fd, mud, h, x, cfg)
    % 动态模型补偿：(4.5)、(4.6)、(4.9)、(4.10)。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    [phi, T, dT] = rfc_xyz(s.R, s.v(4:6));
    df = fd - h(1:3);
    dmu = mud - h(4:6);

    pc = cfg.KPp \ (cfg.KFp*df + cfg.KIp*x(1:3));
    phic = cfg.KPo \ (cfg.KFo*dmu + cfg.KIo*x(4:6));
    dphi = T \ s.v(4:6);
    a = [-cfg.KDp*s.v(1:3) + cfg.KPp*(pc - s.p);
         T*(-cfg.KDo*dphi + cfg.KPo*(phic - phi)) + dT*dphi];

    alpha = s.J \ (a - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;
    dx = [df; dmu];
end
