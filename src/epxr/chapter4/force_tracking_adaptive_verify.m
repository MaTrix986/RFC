here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

robot = load_irb1300();
cfg = chapter4_parameters(robot, 'link7', 'adaptive');
q0 = deg2rad([15; -25; 25; 15; 35; -60]);
H0 = getTransform(robot, q0, cfg.tip);
p0 = H0(1:3, 4);
R0 = H0(1:3, 1:3);
zp = p0(3) - 0.08;
ke = 2000;
x0 = [zeros(9, 1); 1.3/ke];
% cfg.gamma = 0;

[t, X] = ode15s(@(t, X) rhs(t, X, robot, cfg, p0, R0, zp, ke), ...
                [0, 8], [q0; zeros(6, 1); x0], ...
                odeset('RelTol', 1e-5, 'AbsTol', 1e-7));

[p, pd, fn, fd] = output(t, X, robot, cfg, p0, R0, zp, ke);
draw(t, p, pd, fn, fd, zp);

function dX = rhs(t, X, robot, cfg, p0, R0, zp, ke)
    q = X(1:6);
    dq = X(7:12);
    H = getTransform(robot, q, cfg.tip);
    h = [0; 0; -ke*max(zp - H(3, 4), 0); zeros(3, 1)];
    [tau, dx] = force_tracking_adaptive( ...
        robot, q, dq, reference2(t, p0, R0), force_reference(t), ...
        h, X(13:22), cfg);
    J = geometricJacobian(robot, q, cfg.tip);
    J = J([4:6 1:3], :);
    dX = [dq; forwardDynamics(robot, q, dq, tau - J.'*h); dx];
end

function r = reference(t, p0, R0)
    Tx = 4;
    Tz = 3;
    dx = 0.06;
    dz = -0.082;   % 平面在 -0.08 m，继续穿透 2 mm 后保持不变

    ux = min(max(t/Tx, 0), 1);
    uz = min(max(t/Tz, 0), 1);

    sx = 10*ux^3 - 15*ux^4 + 6*ux^5;
    dsx = (30*ux^2 - 60*ux^3 + 30*ux^4)/Tx;
    ddsx = (60*ux - 180*ux^2 + 120*ux^3)/Tx^2;

    sz = 10*uz^3 - 15*uz^4 + 6*uz^5;
    dsz = (30*uz^2 - 60*uz^3 + 30*uz^4)/Tz;
    ddsz = (60*uz - 180*uz^2 + 120*uz^3)/Tz^2;

    r.p = p0 + [dx*sx; 0; dz*sz];
    r.dp = [dx*dsx; 0; dz*dsz];
    r.ddp = [dx*ddsx; 0; dz*ddsz];

    r.R = R0;
    r.omega = zeros(3, 1);
    r.domega = zeros(3, 1);
end

function r = reference2(t, p0, R0)
    u = min(max(t/4, 0), 1);
    s = 10*u^3 - 15*u^4 + 6*u^5;
    ds = (30*u^2 - 60*u^3 + 30*u^4)/4;
    dds = (60*u - 180*u^2 + 120*u^3)/16;
    d = [0.06; 0; -0.10];
    r.p = p0 + d*s;
    r.dp = d*ds;
    r.ddp = d*dds;
    r.R = R0;
    r.omega = zeros(3, 1);
    r.domega = zeros(3, 1);
end


function r = force_reference(t)
    tau = t - 2.7;
    u = min(max(tau, 0), 1);
    s = 10*u^3 - 15*u^4 + 6*u^5;
    ds = 30*u^2 - 60*u^3 + 30*u^4;
    dds = 60*u - 180*u^2 + 120*u^3;
    w = pi;
    g = 8 + 2*cos(w*(tau - 1));
    dg = -2*w*sin(w*(tau - 1));
    ddg = -2*w^2*cos(w*(tau - 1));
    F = s*g;
    dF = ds*g + s*dg;
    ddF = dds*g + 2*ds*dg + s*ddg;
    r.f = [0; 0; -F];
    r.df = [0; 0; -dF];
    r.ddf = [0; 0; -ddF];
end

function [p, pd, fn, fd] = output(t, X, robot, cfg, p0, R0, zp, ke)
    p = zeros(numel(t), 3);
    pd = p;
    fn = zeros(size(t));
    fd = fn;
    for k = 1:numel(t)
        H = getTransform(robot, X(k, 1:6).', cfg.tip);
        p(k, :) = H(1:3, 4).';
        r = reference2(t(k), p0, R0);
        pd(k, :) = r.p.';
        f = force_reference(t(k));
        fn(k) = ke*max(zp - p(k, 3), 0);
        fd(k) = -f.f(3);
    end
end

function draw(t, p, pd, fn, fd, zp)
    tiledlayout(1, 2);
    nexttile;
    plot(pd(:, 1), pd(:, 3), '--', p(:, 1), p(:, 3), 'LineWidth', 1.5);
    yline(zp); grid on;
    xlabel('x [m]'); ylabel('z [m]');
    legend('desired', 'actual', 'plane', 'Location', 'best');
    nexttile;
    plot(t, fd, '--', t, fn, 'LineWidth', 1.5); grid on;
    i = find(fn > 1e-3, 1);
    if ~isempty(i)
        xline(t(i), '--', 'contact');
    end
    xline(4, '--', 'trajectory end');
    xlabel('t [s]'); ylabel('normal force [N]');
    legend('desired', 'actual', 'Location', 'best');
end
