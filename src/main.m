

src = fileparts(mfilename('fullpath'));
addpath(...
    fullfile(src, 'model', 'scripts'), ...
    fullfile(src,'controller','chapter2'));
robot = load_irb1300();

% 让初始、目标及中间姿态处于同一 ZYZ 分支。
q0 = deg2rad([15;-25;25;15;35;-60]);
X0 = [q0; zeros(6,1)];
[p0, phi0] = task_state(robot, q0);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

%% 轨迹跟踪：4 s 五次插值，位置与姿态同时变化，初始位姿和速度与参考一致。
tf = 4;
get_ref = @(t) tracking_reference(t, tf, p0, phi0);
track = @(t,q,dq) tracking_control(robot, q, dq, get_ref(t));
[t1,X1] = ode45(@(t,X) robot_ode(t, X, robot, track), 0:0.02:tf, X0, opts);

%% 定点调节：由另一个关节构型生成可达的固定目标位姿，仅反馈末端误差。
q_goal = q0 + deg2rad([4;3;-5;4;3;2]);
[goal.p, goal.phi] = task_state(robot,q_goal);
reg = @(t,q,dq) regulation_control(robot, q, dq, goal);
% URDF 的末轴惯量很小，采用刚性求解器。
[t2,X2] = ode15s(@(t,X) robot_ode(t,X, robot, reg), 0:0.02:30, X0, opts);

%% 结果：位置误差单位 mm，姿态误差为实际相对旋转角（deg）。
err1 = zeros(numel(t1),2);
err2 = zeros(numel(t2),2);
V = zeros(size(t2));
p1 = zeros(numel(t1),3); pd1 = zeros(size(p1));
for i = 1:numel(t1)
    ref = get_ref(t1(i));
    [err1(i,:),p1(i,:)] = pose_error(robot,X1(i,1:6).',ref);
    pd1(i,:) = ref.p.';
end
for i = 1:numel(t2)
    q = X2(i,1:6).'; dq = X2(i,7:12).';
    err2(i,:) = pose_error(robot,q,goal);
    [~,V(i)] = regulation_control(robot, q, dq, goal);
end
fprintf('Tracking:   %.4g -> %.4g mm,  %.4g -> %.4g deg\n', ...
        err1(1,1),err1(end,1),err1(1,2),err1(end,2));
fprintf('Regulation: %.4g -> %.4g mm,  %.4g -> %.4g deg\n', ...
        err2(1,1),err2(end,1),err2(1,2),err2(end,2));
fprintf('Regulation: V(0)=%.6g J, V(end)=%.6g J, max(diff(V))=%.3g J\n', ...
        V(1),V(end),max(diff(V)));

figure('Name','RFC Chapter 2');
tiledlayout(2,2);
nexttile; plot3(pd1(:,1),pd1(:,2),pd1(:,3),'--',p1(:,1),p1(:,2),p1(:,3));
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Tracking'); legend('Desired','Actual');
nexttile; plot(t1,err1(:,1),t2,err2(:,1)); grid on;
ylabel('Position error (mm)'); legend('Tracking','Regulation');
nexttile; plot(t1,err1(:,2),t2,err2(:,2)); grid on;
ylabel('Orientation error (deg)');
nexttile; plot(t2,V); grid on; ylabel('Regulation V (J)'); xlabel('Time (s)');

%% 网格模型动画：跟踪按实际时间播放，调节按 4 倍速播放，同时导出视频。
assets = fullfile(fileparts(src),'assets');
if ~isfolder(assets), mkdir(assets); end
exportgraphics(gcf,fullfile(assets,'chapter2-results.png'),'Resolution',150);
animate_robot(robot,t1,X1(:,1:6),1,fullfile(assets,'chapter2-tracking.mp4'));
animate_robot(robot,t2,X2(:,1:6),4,fullfile(assets,'chapter2-regulation.mp4'));

function ref = tracking_reference(t,tf,p0,phi0)
    u = t/tf;
    s = 10*u^3-15*u^4+6*u^5;
    ds = (30*u^2-60*u^3+30*u^4)/tf;
    dds = (60*u-180*u^2+120*u^3)/tf^2;
    dp = [0.03;0.02;0.025];
    dphi = deg2rad([8;-5;6]);
    ref.p = p0+dp*s;
    ref.dp = dp*ds; ref.ddp = dp*dds;
    ref.phi = phi0+dphi*s;
    ref.dphi = dphi*ds; ref.ddphi = dphi*dds;
end

function dXdt = robot_ode(t,X,robot,controller)
    q = X(1:6); dq = X(7:12);
    tau = controller(t,q,dq);
    ddq = forwardDynamics(robot,q,dq,tau);
    dXdt = [dq;ddq];
end

function [e,p] = pose_error(robot,q,ref)
    H = getTransform(robot,q,'link7');
    p = H(1:3,4).';
    Rerr = H(1:3,1:3).'*eul2r(ref.phi.');
    % atan2 对接近零的旋转也能分辨，避免 acos(trace) 的精度损失。
    skew_part = [Rerr(3,2)-Rerr(2,3);Rerr(1,3)-Rerr(3,1);Rerr(2,1)-Rerr(1,2)];
    theta = atan2(norm(skew_part),trace(Rerr)-1);
    e = [1000*norm(ref.p-H(1:3,4)),rad2deg(theta)];
end
