function plot_3d_trajectory(Pos, Quat)
    

    Axang = quat2axang(Quat')';
    
    Z = Pos(1,:);   X = Pos(2,:);    Y = Pos(3,:);
    W = Axang(1,:); U = Axang(2,:);  V = Axang(3,:);

    scale = 0.28;

    figH = figure('Name','3d trajectory plot','NumberTitle','off');
    set(groot,'CurrentFigure',figH); % set this figure as the current figure
    ax = gca; % get the axis of the current figure
    %view(3);
    view([-75.5, 40]);

    % plot on the axis of the handle 'ax'
    hold on;
    quiver3(ax,X,Y,Z,U,V,W,scale, ...
            'Color','blue', 'LineStyle','-', 'LineWidth',0.8, ...
            'Marker','o', 'MarkerSize',5, 'MarkerEdgeColor','cyan');
        
    plot3(ax, X,Y,Z,'b','Linestyle','-','MarkerSize',4,'LineWidth',1);

    legend('human','SEDS');
    
    plot3(ax, X(1),Y(1),Z(1),'go','MarkerSize',12,'LineWidth',3);
    text(X(1),Y(1),Z(1),'start');
    plot3(ax,X(end),Y(end),Z(end),'gx','MarkerSize',12,'LineWidth',3);
    text(X(end),Y(end),Z(end),'end');

    title('3d oriented trajectory of endEffector');

    hold off

end
