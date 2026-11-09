function [tau, V] = regulation_control(robot, q, dq, goal)
    % 恒定目标；书中 (2.92)、(2.96)、(2.98)。F=0，阻尼来自 KD。
    [p, phi, J, T] = task_state(robot, q);
    KPp = 900*eye(3);
    KPo = 30*eye(3);
    KD = diag([20 24 9 1 0.5 0.02]);
    ep = goal.p-p;
    eo = goal.phi-phi;
    gamma_p = KPp*ep;
    gamma_o = T.' \ (KPo*eo);
    tau = J.'*[gamma_p;gamma_o] - KD*dq + gravityTorque(robot,q);

    if nargout > 1
        V = (dq.'*massMatrix(robot,q)*dq + ep.'*KPp*ep + eo.'*KPo*eo)/2;
    end
end
