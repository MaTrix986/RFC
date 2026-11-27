function [tau, dx] = force_tracking_adaptive(robot, q, dq, ref, fref, h, x, cfg)
    % 自适应刚度力跟踪：(4.52)-(4.59)。x=[pc;dpc;psi;epsilon]。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    pc = x(1:3);
    dpc = x(4:6);
    psi = x(7:9);
    epsilon = x(10);

    df = fref.f - h(1:3);
    fc = cfg.KAp*fref.ddf + cfg.KVp*fref.df + df;
    dpsi = fc - cfg.lambda*psi;
    depsilon = cfg.gamma*psi.'*df;
    phi = epsilon*fc + depsilon*psi;
    ddpc = cfg.KAp \ (phi - cfg.KVp*dpc);
    ap = ref.ddp + ddpc + cfg.KDp*(ref.dp + dpc - s.v(1:3)) ...
         + cfg.KPp*(ref.p + pc - s.p);

    a = rfc_motion_acceleration(s, ref, cfg, ap);
    alpha = s.J \ (a - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;
    dx = [dpc; ddpc; dpsi; depsilon];
end
