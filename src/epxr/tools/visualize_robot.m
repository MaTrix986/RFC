function visualize_robot(t, X)
    robot = load_irb1300();
    root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    out = fullfile(root, 'res');
    if ~isfolder(out), mkdir(out); end

    fps = 20;
    ti = linspace(t(1), t(end), max(2, round((t(end)-t(1))*fps)+1));
    Q = interp1(t, X(:,1:6), ti);
    video = VideoWriter(fullfile(out, 'robot_animation.mp4'), 'MPEG-4');
    video.FrameRate = fps; open(video);
    cleanup = onCleanup(@() close(video));

    figure; ax = axes;
    show(robot, Q(1,:).', 'Parent', ax, 'Frames', 'off');
    hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');
    view(ax, 135, 20);
    xlim(ax, [-0.8 0.8]); ylim(ax, [-0.8 0.8]); zlim(ax, [0 1.6]);
    camtarget(ax, [0 0 0.8]);
    axis(ax, 'manual');
    ax.CameraPositionMode = 'manual';
    ax.CameraTargetMode = 'manual';
    ax.CameraUpVectorMode = 'manual';
    ax.CameraViewAngleMode = 'manual';
    path = animatedline(ax, 'Color', 'r', 'LineWidth', 2);
    for k = 1:numel(ti)
        q = Q(k,:).';
        show(robot, q, 'Parent', ax, 'Frames', 'off', ...
            'PreservePlot', false, 'FastUpdate', true);
        H = getTransform(robot, q, 'link7');
        addpoints(path, H(1,4), H(2,4), H(3,4));
        drawnow; writeVideo(video, getframe(gcf));
    end
end
