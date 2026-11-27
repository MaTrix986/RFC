here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

robot = load_irb1300();
cfg = chapter3_parameters(robot, 'link7');
cfg.Ko = diag([8, 12, 5]);

q0 = deg2rad([15; -25; 25; 15; 35; -60]);
H0 = getTransform(robot, q0, cfg.tip);
ref.p = H0(1:3, 4);
ref.R = H0(1:3, 1:3);
ref.dp = zeros(3, 1); ref.ddp = zeros(3, 1);
ref.omega = zeros(3, 1); ref.domega = zeros(3, 1);
x0 = [ref.p; ref.dp; rfc_xyz(ref.R); zeros(3, 1)];

[t, X] = ode15s(@(t, X) closed_loop(t, X, robot, ref, cfg), ...
                [0, 10], [q0; zeros(6, 1); x0], ...
                odeset('RelTol', 1e-5, 'AbsTol', 1e-7));

z_error = zeros(size(t));
mu = zeros(size(t));
for k = 1:numel(t)
    H = getTransform(robot, X(k, 1:6).', cfg.tip);
    z_error(k) = acos(max(-1, min(1, H(1:3, 3).'*ref.R(:, 3))));
    mu(k) = load_moment(t(k));
end

figure('Name', 'XYZ Euler geometric consistency', 'Color', 'w');
tiledlayout(1, 2);
nexttile;
plot(t, z_error, 'k', 'LineWidth', 1.5); grid on;
xlim([0, 10]); 
xlabel('t [s]'); ylabel('z-axis error [rad]');
title('orientation misalignment');
nexttile;
plot(t, [zeros(numel(t), 2), mu], 'LineWidth', 1.5); grid on;
xlim([0, 10]); ylim([-0.5, 3]);
xlabel('t [s]'); ylabel('\mu_d [N m]');
title('applied moment');
legend('x_d', 'y_d', 'z_d', 'Location', 'best');

function dX = closed_loop(t, X, robot, ref, cfg)
    q = X(1:6);
    dq = X(7:12);
    h = [zeros(3, 1); ref.R(:, 3)*load_moment(t)];
    [~, dx, out] = impedance_inner_xyz_control( ...
        robot, q, dq, ref, h, X(13:24), cfg);
    dX = [dq; out.alpha; dx];
end

function mu = load_moment(t)
    u = min(max(t - 2, 0), 1);
    mu = 2.5*(10*u^3 - 15*u^4 + 6*u^5);
end
