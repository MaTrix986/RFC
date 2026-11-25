function robot = load_irb1300()
    model_dir = fileparts(mfilename('fullpath'));
    robot = importrobot(fullfile(model_dir, '..', 'urdf', 'irb1300_redef.urdf'));
    robot.DataFormat = 'column';
    robot.Gravity = [0 0 -9.81];
end
