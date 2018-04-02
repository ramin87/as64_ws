function [Time, y_data, dy_data, ddy_data] = DMP_sim(dmp, dt, tend, y0, g0)


%set_matlab_utils_path();

canClockPtr = dmp.canClockPtr;

tau = canClockPtr.getTau();

%% ========================================================
%% DMP simulation

g = g0; 
x = 0.0;
dx = 0.0;
t = 0.0;

y = y0;
dy = 0.0;
ddy = 0.0;
z = 0;
dz = 0;

Time = [];
x_data = [];
y_data = [];
dy_data = [];
ddy_data = [];

iters = 0;

while (true)
     
    %% data logging

    Time = [Time t];
    
    x_data = [x_data x];
    
    y_data = [y_data y];
    dy_data = [dy_data dy];   
    ddy_data = [ddy_data ddy];

    %% DMP model simulation
    Y_c = 0.0;
    Z_c = 0.0;
    [dy, dz] = dmp.getStatesDot(x, y, z, y0, g, Y_c, Z_c);
    dx = canClockPtr.getPhaseDot(x);
    ddy = dz/dmp.get_v_scale();
    
    iters = iters + 1;
    if (t>=tend)
        break;
    end

    %% Numerical integration
    t = t + dt;
    
    x = x + dx*dt;
    
    y = y + dy*dt;
    z = z + dz*dt;
end
