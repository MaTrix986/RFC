function [bias, dJ] = serial_jacobian_dot(robot, q, dq, tip)
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

        k = k + 1;
        Hi = getTransform(robot, q, body.Name);
        Ji = geometricJacobian(robot, q, body.Name);
        vi = Ji*dq;
        z = Jw(1:3,k);
        dz = cross(vi(1:3), z);
        dJ(:,k) = [cross(dz, pe - Hi(1:3,4)) + cross(z, ve - vi(4:6)); dz];
    end

    bias = dJ*dq;
end
