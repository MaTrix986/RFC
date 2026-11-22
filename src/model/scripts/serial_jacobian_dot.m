function [bias, dJ] = serial_jacobian_dot(robot, q, dq, tip)
    % 固定基座、单串联 URDF 转动关节链；允许链中存在固定关节。
    % 使用工具箱位姿/雅可比求解析 Jdot，不手动复现机器人运动学。
    H = getTransform(robot, q, tip);
    pe = H(1:3,4);
    Jw = geometricJacobian(robot, q, tip);
    ve = Jw(4:6,:)*dq;
    dJ = zeros(6, numel(q));
    k = 0;

    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        if strcmp(body.Joint.Type, 'fixed')
            continue;
        end
        assert(strcmp(body.Joint.Type, 'revolute'), '仅支持转动关节串联链。');

        k = k + 1;
        Hi = getTransform(robot, q, body.Name);
        Ji = geometricJacobian(robot, q, body.Name);
        vi = Ji*dq;
        z = Jw(1:3,k);
        dz = cross(vi(1:3), z);
        dJ(:,k) = [cross(dz, pe - Hi(1:3,4)) + cross(z, ve - vi(4:6)); dz];
    end

    assert(k == numel(q), '关节数与配置向量不一致。');
    bias = dJ*dq;
end
