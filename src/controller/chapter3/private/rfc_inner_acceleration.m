function a = rfc_inner_acceleration(s, c, cfg)
    Qce = rotm2quat(s.R.'*c.R).';
    if Qce(1) < 0
        Qce = -Qce;            % 选最短相对旋转分支，避开 180 度
    end

    ap = c.ddp + cfg.KDp*(c.dp - s.v(1:3)) + cfg.KPp*(c.p - s.p);
    ao = c.domega + cfg.KDo*(c.omega - s.v(4:6)) + cfg.KPo*s.R*Qce(2:4);
    a = [ap; ao];
end
