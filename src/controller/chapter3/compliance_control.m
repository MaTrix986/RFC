function tau = compliance_control(robot, q, dq, goal, cfg)
    % 主动柔顺：(2.92)、(3.2)。固定目标，XYZ 欧拉角，F=0。
    % 不使用力传感器；外力只通过真实机器人动力学产生柔顺位移。
    s = rfc_state(robot, q, dq, cfg.tip);
    [phi, T] = rfc_xyz(s.R);
    phid = rfc_xyz(goal.R);

    gamma = [cfg.Kp*(goal.p - s.p); T.' \ (cfg.Ko*(phid - phi))];
    tau = s.J.'*gamma - cfg.Kq*dq + gravityTorque(robot, q);
end
