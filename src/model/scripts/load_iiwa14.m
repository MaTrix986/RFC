function robot = load_iiwa14()
    root = fileparts(fileparts(mfilename('fullpath')));
    robot = importrobot(fullfile(root, 'iiwa14', 'iiwa14.urdf'));
    robot.DataFormat = 'column';
    robot.Gravity = [0 0 -9.81];
end
