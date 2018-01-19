function animate_3d_trajectory_of_frames(T, Pos, Quat)
    

    Axang = quat2axang(Quat')';
    
    figure('Name','animated frames plot','NumberTitle','off');
    pos_scale = 20;
    ax_max = pos_scale * ( max(abs(Pos(1:3,1))) + 0.0);
    ax_min = -ax_max;

    ax = axes('XLim',[ax_min ax_max],'YLim',[ax_min ax_max],'ZLim',[ax_min ax_max]);
    view([-75.5, 40]);
    %view(3);
    grid on

    frame_scale = 20;
    endEffector_frame = get_frame_handle(frame_scale,0.95);
    set(endEffector_frame,'Parent',ax);
    endEffector_trace = animatedline('Color','blue', 'LineStyle','--', 'LineWidth',0.75, 'Marker','o', 'MarkerSize',5);
    set(endEffector_trace,'Parent',ax);


    target_frame = get_frame_handle(frame_scale,0.7);
    set(target_frame,'Parent',ax);

    legend('human','SEDS');
    drawnow

    % use this matrix to implicitly change the viewpoint by changing the positions of the axes
    T_yzx0 = [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
    set(target_frame, 'Matrix',T_yzx0);
    for i=2:size(Pos,2)
        T_target_endEffector = T_yzx0 * makehgtform('translate',pos_scale*Pos(:,i)') * makehgtform('axisrotate',Axang(1:3,i)',Axang(4,i));
        set(endEffector_frame, 'Matrix',T_target_endEffector);
        point_trace = T_yzx0(1:3,1:3) * pos_scale*Pos(:,i);
        addpoints(endEffector_trace,point_trace(1),point_trace(2),point_trace(3));
        drawnow
        pause(T(i)-T(i-1))
    end

end
