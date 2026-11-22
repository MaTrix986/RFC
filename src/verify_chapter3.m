function results = verify_chapter3()
    % 算法验证入口：不生成实验图，不修改 main.m。
    root = fileparts(mfilename('fullpath'));
    addpath(fullfile(root, 'model', 'scripts'), ...
            fullfile(root, 'controller', 'chapter3'));
    rng(7);

    robots = {load_irb1300(), load_iiwa14()};
    tips = {'link7', 'iiwa_link_7'};
    qs = {deg2rad([15; -25; 25; 15; 35; -60]), ...
          deg2rad([10; 30; 15; -55; 20; 40; 10])};

    % 检查质量矩阵及解析 Jdot。
    for k = 1:2
        robot = robots{k};
        q = qs{k};
        dq = 0.1*randn(size(q));
        tip = tips{k};

        B = massMatrix(robot, q);
        assert(min(eig(B)) > 0);
        [~, dJ] = serial_jacobian_dot(robot, q, dq, tip);

        dt = 1e-6;
        Jp = jac(robot, q + dt*dq, tip);
        Jm = jac(robot, q - dt*dq, tip);
        residual = norm(dJ - (Jp - Jm)/(2*dt), 'fro');
        assert(residual < 1e-7, '解析 Jdot 验证失败。');

        fprintf('Model %d: n=%d, min eig(B)=%.3g, Jdot residual=%.3g\n', ...
            k, numel(q), min(eig(B)), residual);
    end

    robot = robots{1};
    q = qs{1};
    dq = zeros(6, 1);
    cfg = chapter3_parameters(robot, tips{1});
    ref = reference(robot, q, cfg.tip);
    J = jac(robot, q, cfg.tip);
    h = [2; -1; 3; 0.1; -0.05; 0.08];

    tau = compliance_control(robot, q, dq, ref, cfg);
    assert(norm(tau - gravityTorque(robot, q)) < 1e-9);
    ref.p = ref.p + [0.003; -0.002; 0.002];

    % 主动柔顺在 h=gamma 时应静力平衡（含正确的接触力符号）。
    tau = compliance_control(robot, q, dq, ref, cfg);
    hc = [cfg.Kp*(ref.p - get_position(robot, q, cfg.tip)); zeros(3, 1)];
    assert(norm(forwardDynamics(robot, q, dq, tau - J.'*hc)) < 1e-7);

    [tau, out] = impedance_control(robot, q, dq, ref, h, cfg);
    ddq = forwardDynamics(robot, q, dq, tau - J.'*h);
    assert(norm(J*ddq - out.a) < 1e-7, '直接阻抗动力学补偿失败。');
    assert(norm(cfg.Mp*(ref.ddp - out.a(1:3)) ...
                + cfg.Kp*(ref.p - get_position(robot, q, cfg.tip)) ...
                - h(1:3)) < 1e-8);

    x = initialize_impedance_state(reference(robot, q, cfg.tip));
    [tau, dx, out] = impedance_inner_control(robot, q, dq, ref, h, x, cfg);
    ddq = forwardDynamics(robot, q, dq, tau - J.'*h);
    assert(norm(J*ddq - out.a) < 1e-7, '内环动力学补偿失败。');
    assert(numel(dx) == 13 && all(isfinite(dx)));

    % 移动参考、非零相对旋转：独立检查 Rc、omega_c、dot(omega_c) 一致性。
    moving = ref;
    moving.omega = [0.2; -0.1; 0.15];
    moving.domega = [0.04; 0.01; -0.02];
    x(7:10) = axang2quat([1 2 3 0.2]./[norm([1 2 3])*ones(1, 3) 1]).';
    x(11:13) = [0.03; -0.05; 0.02];
    [c, dx] = impedance_reference(x, moving, h, cfg);

    dt = 1e-6;
    rp = moving;
    rm = moving;
    rp.R = expm(skew(moving.omega)*dt)*moving.R;
    rm.R = expm(-skew(moving.omega)*dt)*moving.R;
    rp.omega = moving.omega + dt*moving.domega;
    rm.omega = moving.omega - dt*moving.domega;
    cp = impedance_reference(x + dt*dx, rp, h, cfg);
    cm = impedance_reference(x - dt*dx, rm, h, cfg);

    assert(norm((cp.R - cm.R)/(2*dt) - skew(c.omega)*c.R, 'fro') < 1e-7);
    assert(norm((cp.omega - cm.omega)/(2*dt) - c.domega) < 1e-7);
    fprintf('Compliance equilibrium, direct/inner dynamics and reference derivatives: PASS\n');

    % 验证冗余公式，包含非零关节速度与外力。
    robot = robots{2};
    q = qs{2};
    dq = 0.03*randn(7, 1);
    cfg = chapter3_parameters(robot, tips{2});
    ref = reference(robot, q, cfg.tip);
    cfg.qrest = q + [0.1; -0.03; 0.05; 0.02; 0; -0.03; 0.04];
    x = initialize_impedance_state(ref);
    J = jac(robot, q, cfg.tip);
    B = massMatrix(robot, q);

    [tau, ~, out] = redundant_impedance_control(robot, q, dq, ref, h, x, cfg);
    [bias, ~] = serial_jacobian_dot(robot, q, dq, cfg.tip);
    ddq = forwardDynamics(robot, q, dq, tau - J.'*h);

    assert(norm(J*out.an) < 1e-9);
    assert(norm(out.N*(B \ J.'), 'fro') < 1e-8);
    assert(norm(J*ddq + bias - out.a) < 1e-7);
    assert(abs(out.dV + out.en.'*cfg.Kn*out.en) < 1e-8);
    fprintf('Redundancy: J*an=%.3g, N*B^-1*J''=%.3g, Lyapunov residual=%.3g\n', ...
        norm(J*out.an), norm(out.N*(B \ J.'), 'fro'), ...
        abs(out.dV + out.en.'*cfg.Kn*out.en));

    % 四个控制器的短时闭环积分，六轴和七轴均含非零接触力。
    results = struct();
    for mode = 1:4
        k = 1 + (mode == 4);
        robot = robots{k};
        q = qs{k};
        n = numel(q);
        cfg = chapter3_parameters(robot, tips{k});
        ref = reference(robot, q, cfg.tip);
        x = initialize_impedance_state(ref);
        ref.p = ref.p + [0.003; 0; 0];
        cfg.qrest = q + [0.1; zeros(n - 1, 1)];

        X0 = [q; zeros(n, 1); x];
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
        [t, X] = ode15s(@(t, X) rhs(t, X, robot, ref, cfg, mode), ...
                       [0 0.3], X0, options);
        assert(all(isfinite(X), 'all') ...
               && max(abs(X(:,1:n) - q.'), [], 'all') < 0.5);

        results(mode).mode = mode;
        results(mode).steps = numel(t);
        results(mode).final_position_error = ...
            norm(get_position(robot, X(end,1:n).', cfg.tip) - ref.p);

        if mode >= 3
            xf = X(end,2*n+1:end).';
            c = impedance_reference(xf, ref, [0.5; zeros(5, 1)], cfg);
            H = getTransform(robot, X(end,1:n).', cfg.tip);
            assert(norm(H(1:3,4) - c.p) < 1e-5, '内环没有跟随柔顺参考。');
            assert(norm(H(1:3,1:3) - c.R, 'fro') < 1e-5);
            assert(abs(norm(xf(7:10)) - 1) < 1e-6);
        end

        if mode == 4
            w0 = (q - cfg.qrest).'*cfg.Wrest*(q - cfg.qrest)/2;
            e = X(end,1:n).' - cfg.qrest;
            wf = e.'*cfg.Wrest*e/2;
            assert(wf < w0, '该短时算例的次任务没有改善。');
            fprintf('Null-space objective: %.6g -> %.6g\n', w0, wf);
        end

        fprintf('Controller %d: 0.3 s closed-loop PASS, steps=%d, final |p-pd|=%.3g m\n', ...
            mode, numel(t), results(mode).final_position_error);
    end
    fprintf('ALL CHAPTER 3 CHECKS PASSED\n');
end

function dX = rhs(t, X, robot, ref, cfg, mode)
    n = numel(homeConfiguration(robot));
    q = X(1:n);
    dq = X(n+1:2*n);
    x = X(2*n+1:end);
    h = [0.5; 0; 0; 0; 0; 0]*min(t/0.05, 1);
    dx = zeros(13, 1);

    switch mode
        case 1
            tau = compliance_control(robot, q, dq, ref, cfg);
        case 2
            tau = impedance_control(robot, q, dq, ref, h, cfg);
        case 3
            [tau, dx] = impedance_inner_control(robot, q, dq, ref, h, x, cfg);
        case 4
            [tau, dx] = redundant_impedance_control(robot, q, dq, ref, h, x, cfg);
    end

    J = jac(robot, q, cfg.tip);
    ddq = forwardDynamics(robot, q, dq, tau - J.'*h);
    dX = [dq; ddq; dx];
end

function r = reference(robot, q, tip)
    H = getTransform(robot, q, tip);
    r.p = H(1:3,4);
    r.R = H(1:3,1:3);
    r.dp = zeros(3, 1);
    r.ddp = zeros(3, 1);
    r.omega = zeros(3, 1);
    r.domega = zeros(3, 1);
end

function J = jac(robot, q, tip)
    J = geometricJacobian(robot, q, tip);
    J = J([4:6 1:3],:);
end

function p = get_position(robot, q, tip)
    H = getTransform(robot, q, tip);
    p = H(1:3,4);
end

function S = skew(v)
    S = [0     -v(3)  v(2);
         v(3)   0    -v(1);
        -v(2)   v(1)  0];
end
