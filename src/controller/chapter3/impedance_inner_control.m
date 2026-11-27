function [tau, dx, out] = impedance_inner_control(robot, q, dq, ref, h, x, cfg)
    % 非冗余：四元数外环 (3.72)，运动内环 (3.26)、(3.81)。
    s = rfc_state(robot, q, dq, cfg.tip, true);

    [c, dx] = impedance_reference(x, ref, h, cfg);
    a = rfc_motion_acceleration(s, c, cfg);
    alpha = s.J \ (a - s.dJ*dq);
    tau = inverseDynamics(robot, q, dq, alpha) + s.J.'*h; % (3.17)

    out.c = c;
    out.a = a;
    out.alpha = alpha;
end
