function [bias,Jdot] = jacobian_dot_velocity(robot,q,dq)
    
    H = getTransform(robot, q, 'link7');
    pe = H(1:3, 4);
    Jw = geometricJacobian(robot, q, 'link7'); % 工具箱顺序 [角;线]
    ve = Jw(4:6, :) * dq;
    Jdot = zeros(6,6);
    for i = 1:6
        body = robot.BodyNames{i};
        Hi = getTransform(robot,q,body);
        Ji = geometricJacobian(robot,q,body);
        vi = Ji*dq;
        zi = Jw(1:3,i);                  % 当前关节轴，基坐标系
        dzi = cross(vi(1:3),zi);         % 轴向量随子连杆旋转的导数
        % d/dt [zi x (pe-pi)]，以及 d/dt zi
        Jdot(:,i) = [cross(dzi,pe-Hi(1:3,4)) + cross(zi,ve-vi(4:6)); dzi];
    end
    bias = Jdot*dq;
end
