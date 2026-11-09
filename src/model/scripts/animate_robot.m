function animate_robot(robot,t,Q,speed,video_file)
    % Q 每行为一个采样构型。show 更新真实网格，VideoWriter 导出动画。
    fps = 20;
    frame_count = ceil((t(end)-t(1))/speed*fps)+1;
    frame_times = linspace(t(1),t(end),frame_count);
    frames = interp1(t,Q,frame_times,'linear');
    fig = figure('Name','IRB1300 motion','Position',[100 100 900 720]);
    ax = axes(fig);
    show(robot,frames(1,:).','Parent',ax,'Frames','off','Visuals','on', ...
         'PreservePlot',false,'FastUpdate',true);
    view(ax,135,20);
    axis(ax,'equal');
    xlim(ax,[-0.65 0.65]); ylim(ax,[-0.65 0.65]); zlim(ax,[0 1.45]);
    grid(ax,'on'); xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
    camlight(ax,'headlight'); lighting(ax,'gouraud');
    material(ax,'dull');
    ax.Toolbar.Visible = 'off';
    hold(ax,'on');
    path = animatedline(ax,'Color',[0.85 0.2 0.1],'LineWidth',2);
    video = VideoWriter(video_file,'MPEG-4');
    video.FrameRate = fps;
    open(video);
    cleanup = onCleanup(@() close(video));
    timer = tic;
    for i = 1:frame_count
        if ~isgraphics(fig), break; end % 关闭窗口即可停止当前动画
        q = frames(i,:).';
        show(robot,q,'Parent',ax,'Frames','off','Visuals','on', ...
             'PreservePlot',false,'FastUpdate',true);
        ax.Position = [0.12 0.10 0.76 0.76];
        H = getTransform(robot,q,'link7');
        addpoints(path,H(1,4),H(2,4),H(3,4));
        title(ax,sprintf('IRB1300 | t = %.2f s | playback %.0fx',frame_times(i),speed));
        drawnow;
        writeVideo(video,getframe(fig));
        pause(max(0,(i-1)/fps-toc(timer)));
    end
end
