function [tau, dx] = force_position_full_control(robot, q, dq, ref, fd, h, x, cfg)
    % 全并行力/位置控制：(4.29)-(4.32)。x=[pc;dpc]。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    pc = x(1:3);
    dpc = x(4:6);
    ddpc = cfg.KAp \ (fd - h(1:3) - cfg.KVp*dpc);
    ap = ref.ddp + ddpc + cfg.KDp*(ref.dp + dpc - s.v(1:3)) ...
         + cfg.KPp*(ref.p + pc - s.p);

    a = rfc_motion_acceleration(s, ref, cfg, ap);
    alpha = s.J \ (a - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;
    dx = [dpc; ddpc];
end
