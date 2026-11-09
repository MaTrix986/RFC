function [p, phi, J, T] = task_state(robot, q)
    % RFC 记号：基坐标系，J*dq = [dp; omega]，omega = T(phi)*dphi。
    H = getTransform(robot, q, 'link7');
    p = H(1:3,4);
    phi = tr2eul(H(1:3,1:3)).';  % Corke Spatial Math，ZYZ 欧拉角
    if nargout > 2
        Jw = geometricJacobian(robot, q, 'link7'); % 工具箱顺序 [omega; dp]
        J = Jw([4:6 1:3],:);
        T = eul2jac(phi);
        if rcond(T) < 1e-6
            error('ZYZ 欧拉角接近表示奇异点。');
        end
    end
end
