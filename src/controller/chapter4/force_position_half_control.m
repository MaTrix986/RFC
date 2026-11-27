function [tau, dx] = force_position_half_control(robot, q, dq, ref, fd, h, x, cfg)
    % 半并行力/位置控制：(4.5)、(4.11)、(4.20)。x=int(df)。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    df = fd - h(1:3);
    pc = cfg.KPp \ (cfg.KFp*df + cfg.KIp*x);
    ap = ref.ddp + cfg.KDp*(ref.dp - s.v(1:3)) ...
         + cfg.KPp*(ref.p + pc - s.p);

    a = rfc_motion_acceleration(s, ref, cfg, ap);
    alpha = s.J \ (a - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;
    dx = df;
end
