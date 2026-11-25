function dXdt = robot_ode(t, X, robot, controller)
    q = X(1:6); dq = X(7:12);
    tau = controller(t, q, dq);
    ddq = forwardDynamics(robot, q, dq, tau);
    dXdt = [dq; ddq];
end