function [tau, out] = impedance_control(robot, q, dq, ref, h, cfg)
    % 直接阻抗：(3.17)-(3.21)，XYZ 欧拉角；非冗余六自由度。
    s = rfc_state(robot, q, dq, cfg.tip);
    [phi, T, dT] = rfc_xyz(s.R, s.v(4:6));
    [phid, Td, dTd] = rfc_xyz(ref.R, ref.omega);

    dphi = T \ s.v(4:6);
    dphid = Td \ ref.omega;
    ddphid = Td \ (ref.domega - dTd*dphid);

    ap = ref.ddp + cfg.Mp \ (cfg.Dp*(ref.dp - s.v(1:3)) ...
                             + cfg.Kp*(ref.p - s.p) - h(1:3));
    ddphi = ddphid + cfg.Mo \ (cfg.Do*(dphid - dphi) ...
                               + cfg.Ko*(phid - phi) - T.'*h(4:6));
    ao = T*ddphi + dT*dphi;

    alpha = s.J \ ([ap; ao] - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;

    out.a = [ap; ao];
    out.alpha = alpha;
end
