function s = rfc_state(robot, q, dq, tip, dynamic)
    H = getTransform(robot, q, tip);
    s.p = H(1:3, 4);
    s.R = H(1:3, 1:3);

    J = geometricJacobian(robot, q, tip);
    s.J = J([4:6 1:3], :);
    assert(rank(s.J, 1e-8) == 6, '任务雅可比不满行秩。');
    s.v = s.J*dq;

    if nargin == 5 && dynamic
        [~, s.dJ] = serial_jacobian_dot(robot, q, dq, tip);
    end
end
