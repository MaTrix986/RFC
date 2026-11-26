here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here,'..','..')));

robot = load_irb1300();

cfg = chapter3_parameters(robot,'link7');

q0 = deg2rad([15;-25;25;15;35;-60]);
H0 = getTransform(robot,q0,cfg.tip);
p0 = H0(1:3,4); 
R0 = H0(1:3,1:3);
zp = p0(3)-0.08; 
ke = 2000;
rng(1);
A = randn(6);

[t,X] = ode15s(@(t,X) rhs(t,X,robot,cfg,p0,R0,zp,ke,A), [0 6], ...
    [q0; zeros(6,1)]);

p = zeros(numel(t),3); 
pd = p; 
fn = zeros(size(t));
for k = 1:numel(t)
    H = getTransform(robot,X(k,1:6).',cfg.tip);
    p(k,:) = H(1:3,4).';
    r = reference(t(k),p0,R0); pd(k,:) = r.p.';
    fn(k) = ke*max(zp-p(k,3),0);
end
tc = t(find(fn>0,1));

tiledlayout(1,2);
nexttile;
plot(t,1000*vecnorm(pd-p,2,2),'LineWidth',1.5); grid on;
xlim([0 tc]);
xlabel('t [s]'); ylabel('position error [mm]');
nexttile; 
plot(t,fn,'LineWidth',1.5); grid on;
xline(tc,'--','contact');
xline(4,'--','trajectory end');
xlabel('t [s]'); ylabel('normal force [N]');

function dX = rhs(t,X,robot,cfg,p0,R0,zp,ke,A)
    q = X(1:6); dq = X(7:12);
    H = getTransform(robot,q,cfg.tip);
    h = [0; 0; -ke*max(zp-H(3,4),0); zeros(3,1)];
    tau = impedance_control(robot, q, dq, reference(t, p0, R0), h, cfg);
    J = geometricJacobian(robot,q,cfg.tip); 
    J = J([4:6 1:3],:);
    w = [6;10;14];
    td = 0.1 * [0.4;0.3;0.2;0;0;0] .* (A*[sin(w*t);cos(w*t)])/sqrt(3);
    dX = [dq; forwardDynamics(robot, q, dq, tau+td-J.'*h)];
end

function r = reference(t,p0,R0)
    u = min(max(t/4,0),1);
    s = 10*u^3-15*u^4+6*u^5;
    ds = (30*u^2-60*u^3+30*u^4)/4;
    dds = (60*u-180*u^2+120*u^3)/16;
    d = [0.06; 0; -0.10];
    r.p = p0+d*s; 
    r.dp = d*ds; 
    r.ddp = d*dds;
    r.R = R0; r.omega = zeros(3,1); r.domega = zeros(3,1);
end
