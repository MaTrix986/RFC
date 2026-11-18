function [phi,T,dT] = rfc_xyz(R,omega)
    % RFC 附录 A：R=Rx(alpha)*Ry(beta)*Rz(gamma)。
    b = asin(max(-1,min(1,R(1,3))));
    assert(abs(cos(b))>1e-6,'XYZ 欧拉角接近表示奇异。');
    a = atan2(-R(2,3),R(3,3)); g = atan2(-R(1,2),R(1,1));
    phi = [a;b;g];
    T = [1 0 sin(b);0 cos(a) -sin(a)*cos(b);0 sin(a) cos(a)*cos(b)];
    if nargin>1
        dphi = T\omega;
        dT = [zeros(3,1),cross(T(:,1)*dphi(1),T(:,2)), ...
              cross(T(:,1:2)*dphi(1:2),T(:,3))];
    end
end
