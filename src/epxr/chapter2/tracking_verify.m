% 机器人追踪八字形轨迹

src = fileparts(mfilename('fullpath'));
root = fullfile(src, '..', '..');
addpath(genpath(root));

robot = load_irb1300();

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

q0 = deg2rad([15;-25;25;15;35;-60]);
X0 = [q0; zeros(6,1)];
[p0, phi0] = task_state(robot, q0);

tf = 8;
get_ref = @(t) tracking_reference(t, tf, p0, phi0);
track = @(t,q,dq) tracking_control(robot, q, dq, get_ref(t));
[t, X] = ode45(@(t,X) robot_ode(t, X, robot, track), 0:0.02:tf, X0, opts);

visualize_robot(t, X)

function ref = tracking_reference(t,tf,p0,phi0)
    u = t/tf;
    s = 10*u^3-15*u^4+6*u^5;
    ds = (30*u^2-60*u^3+30*u^4)/tf;
    dds = (60*u-180*u^2+120*u^3)/tf^2;

    theta = 2*pi*s;
    dtheta = 2*pi*ds;
    ddtheta = 2*pi*dds;
    A = 0.18; B = 0.12;

    ref.p = p0 + [A*sin(theta); 0; B*sin(2*theta)];
    ref.dp = [A*cos(theta)*dtheta; 0; ...
              2*B*cos(2*theta)*dtheta];
    ref.ddp = [A*(cos(theta)*ddtheta-sin(theta)*dtheta^2); 0; ...
               2*B*cos(2*theta)*ddtheta ...
               - 4*B*sin(2*theta)*dtheta^2];

    ref.phi = phi0;
    ref.dphi = zeros(3,1);
    ref.ddphi = zeros(3,1);
end


function dXdt = robot_ode(t, X, robot, controller)
    q = X(1:6); dq = X(7:12);
    tau = controller(t, q, dq);
    ddq = forwardDynamics(robot, q, dq, tau);
    dXdt = [dq; ddq];
end