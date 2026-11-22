# RFC 第三章控制器

沿用现有函数式控制器和 MATLAB Robotics System Toolbox `rigidBodyTree`。
不改动第二章 `main.m`。以下四个入口覆盖本轮所需结构；冗余是内环的扩展，
不是独立的力控制原理。本轮不含 §3.5 平移—旋转耦合刚度或实验场景。

| 函数                            | 对应原书                        | 姿态处理                     |
| ------------------------------- | ------------------------------- | ---------------------------- |
| `compliance_control`          | (2.92)、(3.2) 主动柔顺          | XYZ，固定目标                |
| `impedance_control`           | (3.17)–(3.21) 直接阻抗         | XYZ，六轴                    |
| `impedance_inner_control`     | (3.25)、(3.26)、(3.72)、(3.81)  | 四元数外环、四元数内环，六轴 |
| `redundant_impedance_control` | 同上，加 (2.78)、(2.83)–(2.91) | 四元数，七轴                 |

## 统一约定

- `q,dq` 为 n×1 列向量；位置 m、角度 rad、力 N、力矩 Nm。
- `ref` 含 `p,dp,ddp,R,omega,domega`；除 R 为 3×3 外均为 3×1。
  角速度/角加速度都在基坐标系中表达，`dot(R)=skew(omega)*R`。
  compliance 的 `goal` 只需固定的 `p,R`。
- `h=[f;mu]` 是**机器人施加给环境**、在末端原点计矩且用基坐标系表达的力/力矩。
  若传感器输出环境作用于机器人的力，必须先反号；若参考点不同，还须转换力矩。
- 机器人动力学：`B*ddq+C*dq+g=tau-J'*h`，`J*dq=[dp;omega]`。
  MATLAB `geometricJacobian` 的行顺序是 `[角;线]`，代码已统一转换。
- 三种动态阻抗控制都有 `+J'*h` 补偿，同时在阻抗方程中保留力输入。
  仿真中可使用 `forwardDynamics(robot,q,dq,tau-J'*h)` 施加接触载荷。
- 采用精确模型、理想力矩执行、F=0；不含摩擦、饱和、时延、传感器滤波或关节限位保护。
  七轴模型的 link inertia 保留自源文件；不含 Drake 专用电机转子惯量及阻尼扩展。

## 调用示例

```matlab
src = 'D:/MINE/projects/Projects/2026-2/RFC/src';
addpath(fullfile(src,'model','scripts'),fullfile(src,'controller','chapter3'));
robot = load_irb1300();
cfg = chapter3_parameters(robot,'link7');
q = deg2rad([15;-25;25;15;35;-60]); dq = zeros(6,1);
H = getTransform(robot,q,cfg.tip);
ref.p=H(1:3,4); ref.R=H(1:3,1:3);
ref.dp=zeros(3,1); ref.ddp=zeros(3,1);
ref.omega=zeros(3,1); ref.domega=zeros(3,1);
h=zeros(6,1);
tau1 = compliance_control(robot,q,dq,ref,cfg);
tau2 = impedance_control(robot,q,dq,ref,h,cfg);
x = initialize_impedance_state(ref);
[tau3,dx,out] = impedance_inner_control(robot,q,dq,ref,h,x,cfg);
```

外环状态为 `x=[pc(3);dpc(3);eta_dc;epsilon_dc(3);vo(3)]`，共 13 维。
相对四元数表示 `Rc'*Rd`，`vo=Rc'*(omega_d-omega_c)`。
控制器返回 `dx`；调用者须把 x 和机器人状态一起积分，不能每周期重新初始化。
`out.c` 包含 `p,dp,ddp,R,omega,domega`，`out.a` 为内环任务加速度，
`out.alpha` 为关节加速度。状态初始化为 c=d；若初始实际位姿不匹配，须配置合理的过渡。

七轴使用 `load_iiwa14()`、`cfg.tip='iiwa_link_7'`，调用：

```matlab
[tau,dx,out] = redundant_impedance_control(robot,q,dq,ref,h,x,cfg);
```

`cfg.qrest` 为恒定关节目标，`cfg.Wrest` 为对称正定权重，
`w=(q-qrest)'*Wrest*(q-qrest)/2`。默认 `kbeta=-1`，使 beta 沿质量加权负梯度。
`out.w, out.en, out.N, out.an, out.dV` 用于检查次任务及零空间动力学。
这里并不保证任意动态主任务下 w 每时刻单调减小；书中的严格耗散结论针对内部速度误差能量。

## 导数与适用范围

- `serial_jacobian_dot` 使用工具箱几何量解析构造 Jdot；支持当前两种固定基座、
  全转动关节的单串联 URDF 链，允许固定关节；不适用于分支树和移动关节模型。
- 冗余控制由 `velocityProduct(q,v)` 的二次型极化得到 Christoffel C，
  用 `Bdot=C+C'` 及矩阵求导计算加权伪逆导数，不使用实时数值差分。
  该方案优先保证公式清晰，尚未针对硬实时运行优化。
- XYZ 的绝对姿态表示须远离 beta=±pi/2，两个角度解须保持同一连续分支。
  主动柔顺/直接阻抗没有自动分支展开。第二章仍使用原来的 ZYZ，实现互不影响。
- 四元数内环选最短旋转，须避开 180 度分支；外环计算时归一化四元数，
  调用者应选足够严格的积分精度，固定步长实现可在每个接受步后归一化。
- 不满秩或接近奇异时直接报错，没有擅自加入会改变原书方程的阻尼伪逆。

## 验证

在 src 加入 MATLAB 路径后运行 `results=verify_chapter3()`。
验证包括六/七轴正定质量矩阵、Jdot 独立差分校验、静力平衡、非零外力下动力学补偿、
移动姿态参考的导数一致性、零空间不干扰主任务、动态一致性、Lyapunov 导数恒等式，
以及四个控制器的短时闭环运行和内环参考跟踪。
这属于实现正确性和可运行性验证，不是完整接触实验或真机部署验证。
