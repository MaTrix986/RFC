# KUKA LBR iiwa 14 模型来源

- 仓库：https://github.com/RobotLocomotion/models
- 固定版本：a7ba22fa4f5488b494727e7a015acefd84befa9f
- 源文件：iiwa_description/urdf/iiwa14_no_collision.urdf
- 下载日期：2026-11-18
- 七个转动关节；工具坐标系 iiwa_link_7。

`upstream.urdf` 保留原始描述。`iiwa14.urdf` 保留 link、joint、mass、COM、inertia 和
关节限位；将 glTF 外观网格转换成 STL，并修改为相对路径。转换包含 glTF 场景节点变换，
不修改网格的米制尺寸；STL 不携带材质。网格转换采用 trimesh。
去除 Gazebo/transmission/Drake 扩展及 joint damping，采用 F=0 的刚性连杆模型；
不模拟 Drake 的电机转子惯量。模型没有碰撞几何，本轮只用于动力学验证，
后续接触仿真需要单独定义接触模型。

这是一套公开仿真参数，不是已辨识的本机真实机器人参数。
许可证保留在 LICENSE.TXT、iiwa_stack.LICENSE.txt、drake.LICENSE.TXT。
