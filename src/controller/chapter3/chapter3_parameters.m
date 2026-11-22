function cfg = chapter3_parameters(robot, tip)
    % RFC 第三章；SI 单位。h=[f;mu] 为机器人施加给环境的力/力矩。
    cfg.tip = tip;

    cfg.Mp = 10*eye(3);
    cfg.Dp = 255*eye(3);
    cfg.Kp = 500*eye(3);

    cfg.Mo = 0.25*eye(3);
    cfg.Do = 3.5*eye(3);
    cfg.Ko = 2.5*eye(3);

    cfg.KDp = 65*eye(3);
    cfg.KPp = 2025*eye(3);
    cfg.KDo = 65*eye(3);
    cfg.KPo = 4500*eye(3);

    n = numel(homeConfiguration(robot));
    cfg.Kq = 5*eye(n);       % 主动柔顺的关节阻尼
    cfg.Kn = 20*eye(n);      % (2.89) 内部速度误差增益
    cfg.kbeta = -1;         % (2.91) 负值表示最小化 w
    cfg.qrest = homeConfiguration(robot);
    cfg.Wrest = eye(n);     % w = (q-qrest)'*Wrest*(q-qrest)/2
end
