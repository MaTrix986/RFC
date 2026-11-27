function cfg = chapter4_parameters(robot, tip, mode)
    cfg.tip = tip;
    cfg.KPp = 2500*eye(3);
    cfg.KDp = 90*eye(3);
    cfg.KPo = 4500*eye(3);
    cfg.KDo = 65*eye(3);
    cfg.KFp = 0.125*eye(3);
    cfg.KIp = 37.5*eye(3);
    cfg.KFo = cfg.KFp;
    cfg.KIo = cfg.KIp;
    cfg.Kq = 20*eye(numel(homeConfiguration(robot)));

    if nargin == 3 && strcmp(mode, 'adaptive')
        cfg.KAp = 0.005*eye(3);
        cfg.KVp = 0.15*eye(3);
    else
        cfg.KAp = 8*eye(3);
        cfg.KVp = 480*eye(3);
    end
    cfg.lambda = 20;
    cfg.gamma = 1e-4;
end
