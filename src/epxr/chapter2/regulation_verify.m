% 机器人从初始位置平移regulation goal

src = fileparts(mfilename('fullpath'));
root = fullfile(src, '..', '..');
addpath(genpath(root));

robot = load_irb1300();

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

q0 = deg2rad([15;-25;25;15;35;-60]);
X0 = [q0; zeros(6,1)];
[p0, phi0] = task_state(robot, q0);

q_goal = q0 + deg2rad([4;3;-5;4;3;2]);
[goal.p, goal.phi] = task_state(robot,q_goal);
reg = @(t,q,dq) regulation_control(robot, q, dq, goal);

tspan = 0:0.02:30;
[t, X] = ode15s(@(t,X) robot_ode(t,X, robot, reg), tspan, X0, opts);


function dXdt = robot_ode(t, X, robot, controller)
    q = X(1:6); dq = X(7:12);
    tau = controller(t, q, dq);
    ddq = forwardDynamics(robot, q, dq, tau);
    dXdt = [dq; ddq];
end