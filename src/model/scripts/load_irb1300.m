function robot = load_irb1300()
    % 课程项目 irb1300_redef.urdf，STL 保存在同目录 meshes 下。
    % 直接使用 rigidBodyTree；六个关节均为转动关节，末端为 link7。
    model_dir = fileparts(mfilename('fullpath'));
    robot = importrobot(fullfile(model_dir, '..', 'urdf', 'irb1300_redef.urdf'));
    robot.DataFormat = 'column';
    robot.Gravity = [0 0 -9.81];
end
