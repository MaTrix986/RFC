function [c, dx] = impedance_reference(x, ref, h, cfg)
    % (3.25)、(3.72)-(3.73)：外环状态显式交给 ODE 求解器积分。
    % 无 persistent 状态，ODE 的试探/回退调用不会改变控制器。
    pc = x(1:3);
    dpc = x(4:6);
    Q = x(7:10);
    Q = Q/norm(Q);
    eta = Q(1);
    eps = Q(2:4);
    vo = x(11:13);             % Delta^c omega_dc

    Rdc = quat2rotm(Q.');
    Rc = ref.R*Rdc.';
    E = eta*eye(3) - rfc_skew(eps);
    muE = 2*E.'*cfg.Ko*eps;
    dvo = cfg.Mo \ (Rc.'*h(4:6) - cfg.Do*vo - muE);

    ddpc = ref.ddp + cfg.Mp \ (cfg.Dp*(ref.dp - dpc) ...
                               + cfg.Kp*(ref.p - pc) - h(1:3));
    wc = ref.omega - Rc*vo;
    dwc = ref.domega - cross(wc, Rc*vo) - Rc*dvo;

    dQ = 0.5*[-eps.'; E]*vo;
    dx = [dpc; ddpc; dQ; dvo];

    c.p = pc;
    c.dp = dpc;
    c.ddp = ddpc;
    c.R = Rc;
    c.omega = wc;
    c.domega = dwc;
end
