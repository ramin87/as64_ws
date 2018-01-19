function plot_animated_3Dpath_with_orientFrames(Time, Pos, Quat, varargin)
%% Plots 3D path with frames denoting the orientation at each point.
%  @param[in] Time: 1xN matrix with timestamps.
%  @param[in] Pos: 3xN matrix with position (x,y,z) at each column.
%  @param[in] Quat: 4xN matrix with orientation as a unit quaternion at each column.
%  Variable argument list:
%  Optional arguments:
%  @param[in] axes: The axis object where to plot the data (optional, default = gca, i.e. the current axes).
%  Optional Name-Value pairs:
%  @param[in] numberOfFrames: The number of orientation frames to plot. They will be equally spaced along the path (optional, default = 6).
%  @param[in] frameScale: The scaling of the orientation frames size (optional, default = 1.0).
%  @param[in] frameLineWidth: The linewidth of the orientation frame axes (optional, default = 1.0).
%  @param[in] frameXAxisColor: The color of the x-axis of the orientation frame (optional, default = [1 0 0]).
%  @param[in] frameYAxisColor: The color of the y-axis of the orientation frame (optional, default = [0 1 0]).
%  @param[in] frameZAxisColor: The color of the z-axis of the orientation frame (optional, default = [0 0 1]).
%  @param[in] title: The title of the axis (optional, default = '').
%  @param[in] xlabel: The x-label of the axis (optional, default = '').
%  @param[in] ylabel: The y-label of the axis (optional, default = '').
%  @param[in] zlabel: The z-label of the axis (optional, default = '').
%  @param[in] LineWidth: The linewidth of the path line (optional, default = 1.0).
%  @param[in] LineColor: The color of the path line (optional, default = [0.45 0.26 0.26]).
%  @param[in] Interpreter: The text interpreter (optional, default = 'latex').
%  @param[in] fontSize: The text fontsize (optional, default = 12).
%     
    
    %% Parse the input arguments
    inArgs = parseInputArguments(varargin{:});
    ax = inArgs.axes;
    axis_colors = cell({inArgs.frameXAxisColor, inArgs.frameYAxisColor, inArgs.frameZAxisColor});

    
    %% Enable 3D view on the 'ax' object
    view(ax, 3);


    %% Extract position and orientation coordinates
    Axang = quat2axang(Quat')';
    X = Pos(1,:);   Y = Pos(2,:);    Z = Pos(3,:);
    U = Axang(1,:); V = Axang(2,:);  W = Axang(3,:);

    
    %% Set axes limits
    w = max(X) - min(X);
    a = w*0.1;
    ax.XLim = [min(X)-a max(X)+a];
    w = max(Y) - min(Y);
    a = w*0.1;
    ax.YLim = [min(Y)-a max(Y)+a];
    w = max(Z) - min(Z);
    a = w*0.1;
    ax.ZLim = [min(Z)-a max(Z)+a];
    
    
    %% Find where to place the orientation frames so that they equally spaced
    m = inArgs.numberOfFrames; % how many frames to put between the start and end frame
    dist = zeros(length(X),1);
    for i=2:length(dist)
        dist(i) = dist(i-1) + norm([X(i) Y(i) Z(i)]-[X(i-1) Y(i-1) Z(i-1)]);
    end
    d = dist(end)/(m+1);
    frames_ind = [1];
    
    j = 1;
    for i=1:length(dist)
        if (dist(i) >= d*j)
            frames_ind = [frames_ind i];
            j = j + 1;
        end
    end
    frames_ind = [frames_ind length(dist)];
    frames_ind = unique(frames_ind);
    
    
    %% Enable 'hold on' on the ax object
    NextPlot0 = ax.NextPlot;
    ax.NextPlot = 'add';
    
    
    %% set the text properties
    ax.XLabel.String = inArgs.xlabel;
    ax.XLabel.Interpreter = inArgs.Interpreter;
    ax.XLabel.FontSize = inArgs.fontSize;

    ax.YLabel.String = inArgs.ylabel;
    ax.YLabel.Interpreter = inArgs.Interpreter;
    ax.YLabel.FontSize = inArgs.fontSize;

    ax.ZLabel.String = inArgs.zlabel;
    ax.ZLabel.Interpreter = inArgs.Interpreter;
    ax.ZLabel.FontSize = inArgs.fontSize;
    
    ax.Title.String = inArgs.title;
    ax.Title.Interpreter = inArgs.Interpreter;
    ax.Title.FontSize = inArgs.fontSize;

    
    %% Initialize 3 quivers, one for each axis of the orientation frame
    quiv = cell(3,1);
    for j=1:3
        quiv{j} = quiver3(ax, 0,0,0,0,0,0, inArgs.frameScale);
        quiv{j}.Color = axis_colors{j};
        quiv{j}.LineStyle = '-';
        quiv{j}.LineWidth = inArgs.frameLineWidth;    
        quiv{j}.AutoScale = 'off';
    end


    %% Initialize line object
    lineH = line(ax);
    lineH.XData = [];
    lineH.YData = [];
    lineH.ZData = [];
    lineH.Color = inArgs.LineColor;
    lineH.LineWidth = inArgs.LineWidth;
    
    
    %% Plot animated path with frames
    k = 1;
    Time = [Time Time(end)];
    for i=1:(length(Time)-1)
        lineH.XData = [lineH.XData X(i)];
        lineH.YData = [lineH.YData Y(i)];
        lineH.ZData = [lineH.ZData Z(i)];
        
        T = makehgtform('translate',[X(i) Y(i) Z(i)]) * makehgtform('axisrotate',[U(i) V(i) W(i)],Axang(4,i));
        for j=1:3
            quiv{j}.XData = [quiv{j}.XData X(i)];
            quiv{j}.YData = [quiv{j}.YData Y(i)];
            quiv{j}.ZData = [quiv{j}.ZData Z(i)];
            
            quiv{j}.UData = [quiv{j}.UData T(1,j)];
            quiv{j}.VData = [quiv{j}.VData T(2,j)];
            quiv{j}.WData = [quiv{j}.WData T(3,j)];
        end
        
        drawnow
        pause(Time(i+1)-Time(i))
        
        if (i == frames_ind(k))
            k = k+1;
        else
            for j=1:3
                quiv{j}.XData = quiv{j}.XData(1:end-1);
                quiv{j}.YData = quiv{j}.YData(1:end-1);
                quiv{j}.ZData = quiv{j}.ZData(1:end-1);
                
                quiv{j}.UData = quiv{j}.UData(1:end-1);
                quiv{j}.VData = quiv{j}.VData(1:end-1);
                quiv{j}.WData = quiv{j}.WData(1:end-1);
            end
        
        end

    end
    
    for j=1:3  
        quiv{j}.AutoScale = 'on';
    end

    
    %% restore it to its previous state
    ax.NextPlot = NextPlot0; 

end


function inArgs = parseInputArguments(varargin)

    % function for validating input arguments
    is_numeric_scalar_positive = @(x) assert(isnumeric(x) && isscalar(x) && (x > 0), 'Value must be positive, scalar, and numeric.');
    is_numeric_scalar_nonnegative = @(x) assert(isnumeric(x) && isscalar(x) && (x >= 0), 'Value must be non-negative, scalar, and numeric.');
    is_color = @(x) assert(isnumeric(x) && length(x)==3, 'Input must be a vector of 3 elements.');
    is_axes = @(x) assert(strcmp(get(x, 'type'), 'axes'), 'Input must be of type axes.');
    is_string = @(x) assert( ischar(x), 'Value must be a string.');

    % initialize parser with the names and default values of the input arguments
    inPars = inputParser;
    inPars.addOptional('axes', gca, is_axes);
    inPars.addParameter('numberOfFrames', 6, is_numeric_scalar_positive);
    inPars.addParameter('frameScale', 1.0, is_numeric_scalar_nonnegative);
    inPars.addParameter('frameLineWidth', 1.0, is_numeric_scalar_positive);
    inPars.addParameter('frameXAxisColor', [1.0 0.0 0.0], is_color);
    inPars.addParameter('frameYAxisColor', [0.0 1.0 0.0], is_color);
    inPars.addParameter('frameZAxisColor', [1.0 0.0 1.0], is_color);
    
    inPars.addParameter('title', '', is_string);
    inPars.addParameter('xlabel', '', is_string);
    inPars.addParameter('ylabel', '', is_string);
    inPars.addParameter('zlabel', '', is_string);
    
    inPars.addParameter('LineWidth', 1.0, is_numeric_scalar_positive);
    inPars.addParameter('LineColor', [0.45 0.26 0.26], is_color);
    
    inPars.addParameter('Interpreter', 'latex', is_string);
    inPars.addParameter('fontSize', 12, is_numeric_scalar_positive);
    
    % Parse input arguments
    inPars.parse(varargin{:});
    
    inArgs = inPars.Results;


end
