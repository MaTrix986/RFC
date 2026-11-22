function tau = tracking_control(robot, q, dq, ref)
    
    [p, phi, J, T] = task_state(robot, q);
    if rcond(J) < 1e-6
        error('跟踪轨迹接近机械臂运动学奇异点。');
    end
    v = J*dq;
    dphi = T \ v(4:6);
    kp = 36;
    kd = 12;
    ap = ref.ddp + kd*(ref.dp - v(1:3)) + kp*(ref.p - p); % (2.33)

    % (2.40)
    dT = [zeros(3,1), cross(T(:,1)*dphi(1), T(:,2)), ...
          cross(T(:,1:2)*dphi(1:2), T(:,3))];
    ao = T*(ref.ddphi + kd*(ref.dphi-dphi) + kp*(ref.phi-phi)) + dT*dphi;

    bias = jacobian_dot_velocity(robot,q,dq);
    alpha = J \ ([ap;ao] - bias);                         % (2.26)
    tau = inverseDynamics(robot, q, dq, alpha);             % (2.22)，F=0
end
