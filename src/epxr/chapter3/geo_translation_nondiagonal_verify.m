here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

robot = load_irb1300();
cfg = chapter3_parameters(robot, 'link7');
U = [0.8047, -0.3106, 0.5059;
     0.5059, 0.8047, -0.3106;
     -0.3106, 0.5059, 0.8047];
cfg.Kp = U*diag([300, 500, 800])*U.';
q0 = deg2rad([15; -25; 25; 15; 35; -60]);
ref = fixed_reference(robot, q0, cfg.tip);
x0 = initialize_impedance_state(ref);

[t, X] = ode15s(@(t, X) rhs(t, X, robot, ref, cfg), [0, 8], ...
                [q0; zeros(6, 1); x0], ...
                odeset('RelTol', 1e-5, 'AbsTol', 1e-7));

dp = zeros(numel(t), 3);
f = dp;
for k = 1:numel(t)
    H = getTransform(robot, X(k, 1:6).', cfg.tip);
    dp(k, :) = 1000*(H(1:3, 4) - ref.p).';
    f(k, :) = load_force(t(k)).';
end

figure('Name', 'Non-diagonal translational stiffness', 'Color', 'w');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, dp, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('position displacement [mm]');
legend('x', 'y', 'z', 'Location', 'best');
nexttile;
plot(t, f, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('applied force [N]');
legend('x', 'y', 'z', 'Location', 'best');

function dX = rhs(t, X, robot, ref, cfg)
    q = X(1:6);
    dq = X(7:12);
    h = [load_force(t); zeros(3, 1)];
    [~, dx, out] = impedance_inner_control( ...
        robot, q, dq, ref, h, X(13:25), cfg);
    dX = [dq; out.alpha; dx];
end

function f = load_force(t)
    u = min(max(t - 2, 0), 1);
    f = [6*(10*u^3 - 15*u^4 + 6*u^5); 0; 0];
end

function r = fixed_reference(robot, q, tip)
    H = getTransform(robot, q, tip);
    r.p = H(1:3, 4);
    r.R = H(1:3, 1:3);
    r.dp = zeros(3, 1); r.ddp = zeros(3, 1);
    r.omega = zeros(3, 1); r.domega = zeros(3, 1);
end
