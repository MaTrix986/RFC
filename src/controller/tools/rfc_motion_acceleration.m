function a = rfc_motion_acceleration(s, ref, cfg, ap)
    if nargin < 4
        ap = ref.ddp + cfg.KDp*(ref.dp - s.v(1:3)) ...
             + cfg.KPp*(ref.p - s.p);
    end

    Q = rotm2quat(s.R.'*ref.R).';
    if Q(1) < 0
        Q = -Q;
    end
    ao = ref.domega + cfg.KDo*(ref.omega - s.v(4:6)) ...
         + cfg.KPo*s.R*Q(2:4);
    a = [ap; ao];
end
