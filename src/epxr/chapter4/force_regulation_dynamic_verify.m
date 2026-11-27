here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

robot = load_irb1300();
cfg = chapter4_parameters(robot, 'link7');
q0 = deg2rad([15; -25; 25; 15; 35; -60]);
H0 = getTransform(robot, q0, cfg.tip);
p0 = H0(1:3, 4);
phi0 = rfc_xyz(H0(1:3, 1:3));
zp = p0(3) - 0.08;
ke = 2000;
pc0 = p0;
pc0(3) = zp;
x0 = [cfg.KIp \ (cfg.KPp*pc0); cfg.KIo \ (cfg.KPo*phi0)];

[t, X] = ode15s(@(t, X) rhs(t, X, robot, cfg, zp, ke), [0, 6], ...
                [q0; zeros(6, 1); x0], ...
                odeset('RelTol', 1e-5, 'AbsTol', 1e-7));

p = zeros(numel(t), 3);
fn = zeros(size(t));
fd = fn;
for k = 1:numel(t)
    H = getTransform(robot, X(k, 1:6).', cfg.tip);
    p(k, :) = H(1:3, 4).';
    fn(k) = ke*max(zp - p(k, 3), 0);
    f = force_reference(t(k));
    fd(k) = -f(3);
end

tiledlayout(1, 2);
nexttile;
plot(p(:, 1), p(:, 3), 'LineWidth', 1.5);
yline(zp);
grid on;
xlabel('x [m]'); ylabel('z [m]');
legend('actual', 'plane', 'Location', 'best');
nexttile;
plot(t, fd, '--', t, fn, 'LineWidth', 1.5);
grid on;
i = find(fn > 1e-3, 1);
if ~isempty(i)
    xline(t(i), '--', 'contact');
end
xlabel('t [s]'); ylabel('normal force [N]');
legend('desired', 'actual', 'Location', 'best');

function dX = rhs(t, X, robot, cfg, zp, ke)
    q = X(1:6);
    dq = X(7:12);
    H = getTransform(robot, q, cfg.tip);
    h = [0; 0; -ke*max(zp - H(3, 4), 0); zeros(3, 1)];
    [tau, dx] = force_regulation_dynamic( ...
        robot, q, dq, force_reference(t), zeros(3, 1), h, X(13:18), cfg);
    J = geometricJacobian(robot, q, cfg.tip);
    J = J([4:6 1:3], :);
    dX = [dq; forwardDynamics(robot, q, dq, tau - J.'*h); dx];
end

function f = force_reference(t)
    u = min(max(t - 0.5, 0), 1);
    f = [0; 0; -8*(10*u^3 - 15*u^4 + 6*u^5)];
end
