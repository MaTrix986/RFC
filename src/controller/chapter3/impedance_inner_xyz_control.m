function [tau, dx, out] = impedance_inner_xyz_control(robot, q, dq, ref, h, x, cfg)
    % 经典 XYZ 欧拉角阻抗：(3.25)、(3.32)、(3.45)。
    s = rfc_state(robot, q, dq, cfg.tip, true);
    pc = x(1:3);
    dpc = x(4:6);
    phic = x(7:9);
    dphic = x(10:12);

    [phid, Td, dTd] = rfc_xyz(ref.R, ref.omega);
    dphid = Td \ ref.omega;
    ddphid = Td \ (ref.domega - dTd*dphid);
    [Rc, Tc, dTc] = xyz_state(phic, dphic);

    ddpc = ref.ddp + cfg.Mp \ (cfg.Dp*(ref.dp - dpc) ...
                               + cfg.Kp*(ref.p - pc) - h(1:3));
    ddphic = ddphid + cfg.Mo \ (cfg.Do*(dphid - dphic) ...
                                 + cfg.Ko*(phid - phic) - Tc.'*h(4:6));

    [phie, Te, dTe] = rfc_xyz(s.R, s.v(4:6));
    dphie = Te \ s.v(4:6);
    ap = ddpc + cfg.KDp*(dpc - s.v(1:3)) + cfg.KPp*(pc - s.p);
    ao = Te*(ddphic + cfg.KDo*(dphic - dphie) ...
             + cfg.KPo*(phic - phie)) + dTe*dphie;

    alpha = s.J \ ([ap; ao] - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h;
    dx = [dpc; ddpc; dphic; ddphic];

    out.c.p = pc;
    out.c.dp = dpc;
    out.c.ddp = ddpc;
    out.c.R = Rc;
    out.c.omega = Tc*dphic;
    out.c.domega = Tc*ddphic + dTc*dphic;
    out.alpha = alpha;
end

function [R, T, dT] = xyz_state(phi, dphi)
    a = phi(1);
    b = phi(2);
    g = phi(3);
    ca = cos(a); sa = sin(a);
    cb = cos(b); sb = sin(b);
    cg = cos(g); sg = sin(g);

    R = [cb*cg, -cb*sg, sb;
         ca*sg + sa*sb*cg, ca*cg - sa*sb*sg, -sa*cb;
         sa*sg - ca*sb*cg, sa*cg + ca*sb*sg, ca*cb];
    T = [1, 0, sb;
         0, ca, -sa*cb;
         0, sa, ca*cb];
    dT = [zeros(3, 1), cross(T(:, 1)*dphi(1), T(:, 2)), ...
          cross(T(:, 1:2)*dphi(1:2), T(:, 3))];
end
