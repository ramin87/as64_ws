%% OL_1D_DMP_rup class
%  OL: Object load.
%  1D: 1 dimension (gravity direction)
%  DMP_rup: DMP with recursive update method
%

classdef OL_1D_DMP_rup < handle
    properties
        %% DMP parameters
        dmp % dmp model
        can_clock_ptr % canonical clock pointer
        shape_attrGating_ptr % shape attractor gating function pointer
        goal_attrGating_ptr % goal attractor gating function pointer
        D_z % dmp damping
        K_z % dmp stiffness
        N_kernels % number of kernels used in the DMP
        kernel_std_scaling % scaling factor for the kernels std
        train_method % 'LWR', 'LS', 'RLS' , 'RLWR'
        dmp_tau % dmp duration
        train_dmp_offline % if true trains offline the DMP (before online updata/execution)
        
        %% recursive estimation parameters
        est_method % {'KF', 'RLS', 'RLWR'}
        sigma_dmp_w % initial value of covariance matrix for dmp weights
        sigma_noise % variance of noise in force error measurement
        lambda % forgetting factor for recursive training methods
        k_Ferr % gain applied to force error used to updata the dmp weights

        %% Reference model
        y0_ref % initial position for reference 
        g_ref % goal position for reference 
        a6 % reference model param
        a7 % reference model param
        tau_ref % reference model trajectory duration

        %% Human model
        M_h % inertia
        D_h % damping
        K_h % stiffness   

        %% Robot model
        M_r % inertia
        D_r % damping
        K_r % stiffness       

        %% Object Model
        M_o % object's mass
        od_flag % true of we consider the object dynamics
        
        %% Object lifting params
        M_o_r % object load carried by the robot
        M_o_h % object load carried by the human
        load_h_p_var % variance in the load carried by the human


        %% Simulation params
        sim_timestep %simulation time_step;
        pos_tol_stop % position error tolerance to stop the simulation
        vel_tol_stop % minimum velocity to stop the simulation
        max_sim_time_p % maximum simulation time as percentage of the reference trajectory time
        
        %% Training data
        Ts % sampling time
        Time_data % train data timestamps
        Y_data % train data position
        dY_data % train data velocity
        ddY_data % train data acceleration
        
        %% data logging
        log_data % struct for logging training and simulation data and results

        %% other parameters
        grav % the gravity acceleration
        ref_model % human's reference model

    end

    methods
        %% constructor
        function this = OL_1D_DMP_rup()
            
            setMatlabPath();
            this.init();
            
            disp(['Object lift - 1D - DMP with ' this.est_method ' update - with object dynamics']);
            
        end

        %% Initialization
        function init(this)

            this.initProgramArguments();
            this.initProgramVariables();

        end

        function initProgramArguments(this)
            
            zero_tol = 1e-50;
            
            %% DMP parameters
            this.D_z = 50.0;
            this.K_z = 0;
            this.dmp_tau = 10; % 10 sec
            this.N_kernels = 50 * this.dmp_tau;
            this.kernel_std_scaling = 1.0;
            this.train_method = 'LWR';
            this.train_dmp_offline = false;
            
            %% recursive estimation parameters
            this.est_method = 'KF'; % {'KF', 'RLS', 'RLWR'}
            this.sigma_dmp_w = 1e2;
            this.sigma_noise = 8e-1;
            this.lambda = 0.99;
            % k_Ferr, defined at the end
            
            
            %% Reference model
            this.y0_ref = 0.0;
            this.g_ref = 0.6;
            this.a6 = -5.0;
            this.a7 = 0.0;
            this.tau_ref = 2.5;


            %% Human model
            this.M_h = 15.0 + zero_tol;
            this.K_h = 500.0;
            this.D_h = 2*sqrt(this.M_h*this.K_h);

            
            %% Robot model
            this.M_r = 20.0 + zero_tol;
            this.K_r = 750.0;
            this.D_r = 2*sqrt(this.M_r*this.K_r);

            
            %% Object model
            this.M_o = 10.0 + zero_tol;
            this.od_flag = true;
            this.M_o_r = 7.0;
            this.M_o_h = 3.0;
            this.load_h_p_var = 0.0;

            this.k_Ferr = 1/this.M_h;
            
            %% Simulation params
            this.sim_timestep = 0.002;
            this.pos_tol_stop = 0.02;
            this.vel_tol_stop = 0.005;
            this.max_sim_time_p = 1.3;
            
        end
        
        function initProgramVariables(this)
            
            %% global params
            this.grav = 9.81;
            
            
            %% Set up Reference model
            this.ref_model = RefModel(this.y0_ref, this.g_ref, this.tau_ref, this.a6, this.a7);
            
            
            %% Set up DMP model
            this.can_clock_ptr = LinCanonicalClock(this.dmp_tau);
            this.shape_attrGating_ptr = ConstGatingFunction(1.0, 1.0);
            this.goal_attrGating_ptr = ConstGatingFunction(1.0, 1.0);
            this.dmp = DMP_VT(this.N_kernels, this.D_z, this.K_z/this.D_z, this.can_clock_ptr, this.shape_attrGating_ptr, this.goal_attrGating_ptr, this.kernel_std_scaling);
            
            %% Init log data struct
            this.log_data = struct('Time',[], ...,
                                  'y_dmp_data',[],'dy_dmp_data',[],'ddy_dmp_data',[],'x_data',[], 'w_dmp_data',[], ...
                                  'P_w_data',[], ...
                                  'y_r_data',[],'dy_r_data',[],'ddy_r_data',[], 'u_r_data',[], 'F_c_r_data',[], ...
                                  'y_ref_data',[],'dy_ref_data',[],'ddy_ref_data',[], ...
                                  'y_h_data',[],'dy_h_data',[],'ddy_h_data',[], 'u_h_data',[], 'F_c_h_data',[],  ...
                                  'y_o_data',[],'dy_o_data',[],'ddy_o_data',[], 'F_c_o_data',[],  ...
                                  'F_err_data',[], 'F_c_r_d_data',[], 'F_c_h_d_data',[]);
            
            %% load train data
            load data/data.mat Ts Time_data Y_data dY_data ddY_data
            this.Ts = Ts;
            this.Time_data = Time_data;
            this.Y_data = Y_data;
            this.dY_data = dY_data;
            this.ddY_data = ddY_data;
            
        end
        
        %% Train the dmp model
        function trainModel(this)
            
            if (this.train_dmp_offline)
                trainParamsName = {'lambda', 'P_cov'};
                trainParamsValue = {this.lambda, this.sigma_dmp_w};
                disp('DMP training...')
                tic
                this.dmp.setTrainingParams(this.train_method, trainParamsName, trainParamsValue);
                this.dmp.trainMulti(this.Time_data, this.Y_data, this.dY_data, this.ddY_data);      
                toc
            else
                disp('No training. Initializing DMP with zero weights...')
                this.dmp.w = zeros(size(this.dmp.w));
            end

            this.can_clock_ptr.setTau(this.dmp_tau);
            
        end
        
        %% Run simulation
        function simulation(this)
            
            %% copy some pointers for ease of use
            M_r = this.M_r;  D_r = this.D_r;  K_r = this.K_r;
            M_h = this.M_h;  D_h = this.D_h;  K_h = this.K_h;
            M_o = this.M_o;
            M_o_r = this. M_o_r;
            M_o_h = this. M_o_h;
            log_data = this.log_data;
            grav = this.grav;

            
            %% set initial values
            y0 = this.y0_ref;
            g0 = this.g_ref; 
            g = y0; 
            x = 0.0;
            dx = 0.0;
            t = 0.0;

            y_dmp = y0;
            dy_dmp = 0.0;
            ddy_dmp = 0.0;
            z = 0;
            dz = 0;

            w_o = this.M_o*this.grav;
            F_c_h_d = M_o_r;
            F_c_r_d = M_o_h;

            y_r = y0;
            dy_r = 0.0;
            ddy_r = 0.0;
            u_r = 0.0;
            F_c_r = 0;

            y_h = y0;
            dy_h = 0.0;
            ddy_h = 0.0;
            u_h = 0.0;
            F_c_h = 0;
            
            y_o = y0;
            dy_o = 0.0;
            ddy_o = 0.0;
            F_c_o = F_c_r+F_c_h;

            F_err_prev = 0.0;
            F_err = 0.0;
            F_err_mean = 0.0;

            F_c1 = 0.0;
            F_c2 = 0.0;

            P_w = this.sigma_dmp_w*ones(this.dmp.N_kernels,1);
            Sigma_w = diag(P_w);

            tau = this.tau_ref;
            dt = this.sim_timestep;

            iters = 0;

            %% Simulation
            disp('Simulation...')
            tic
            while (true)

                %% ===========  Get refernce trajectories  =================

                %% get the human's model reference
                [y_ref, dy_ref, ddy_ref] = this.ref_model.getRef(t);

                %% get the DMP's model reference
                Y_c = 0.0;
                Z_c = 0.0; % 0.97*F_err;
                [dy_dmp, dz] = this.dmp.getStatesDot(x, y_dmp, z, y0, g, Y_c, Z_c);
                dx = this.can_clock_ptr.getPhaseDot(x);
                ddy_dmp = dz/this.dmp.get_v_scale();


                %% ===========  data logging ===========  

                log_data.Time = [log_data.Time t];

                log_data.x_data = [log_data.x_data x];

                log_data.y_dmp_data = [log_data.y_dmp_data y_dmp];
                log_data.dy_dmp_data = [log_data.dy_dmp_data dy_dmp];   
                log_data.ddy_dmp_data = [log_data.ddy_dmp_data ddy_dmp];

                log_data.y_r_data = [log_data.y_r_data y_r];
                log_data.dy_r_data = [log_data.dy_r_data dy_r];   
                log_data.ddy_r_data = [log_data.ddy_r_data ddy_r];
                log_data.u_r_data = [log_data.u_r_data u_r];
                log_data.F_c_r_data = [log_data.F_c_r_data F_c_r];

                log_data.y_h_data = [log_data.y_h_data y_h];
                log_data.dy_h_data = [log_data.dy_h_data dy_h];   
                log_data.ddy_h_data = [log_data.ddy_h_data ddy_h];
                log_data.u_h_data = [log_data.u_h_data u_h];
                log_data.F_c_h_data = [log_data.F_c_h_data F_c_h];
                
                log_data.y_o_data = [log_data.y_o_data y_o];
                log_data.dy_o_data = [log_data.dy_o_data dy_o];   
                log_data.ddy_o_data = [log_data.ddy_o_data ddy_o];
                log_data.F_c_o_data = [log_data.F_c_o_data F_c_o];

                log_data.y_ref_data = [log_data.y_ref_data y_ref];
                log_data.dy_ref_data = [log_data.dy_ref_data dy_ref];   
                log_data.ddy_ref_data = [log_data.ddy_ref_data ddy_ref];

                
                log_data.F_err_data = [log_data.F_err_data F_err];

                log_data.w_dmp_data = [log_data.w_dmp_data this.dmp.w];

                log_data.P_w_data = [log_data.P_w_data P_w];

                log_data.F_c_r_d_data = [log_data.F_c_r_d_data F_c_r_d];
                log_data.F_c_h_d_data = [log_data.F_c_h_d_data F_c_h_d];


                %% ===========  Robot and Human model simulation  ===========  

                % Control applied by the robot to track its reference
                v_r = K_r*(y_dmp-y_r) + D_r*dy_dmp + M_r*ddy_dmp;

                % Control applied by the human to track its reference
                v_h = K_h*(y_ref-y_h) + D_h*dy_ref + M_h*ddy_ref;
%                 a_g = this.sim_timestep*this.tau_ref/2; 
%                 g = (1-a_g)*g + a_g*g0;
%                 v_h = K_h*(g-y_h);
                
                % Nominal coupling force exerted on the human
                F_c_h_d = this.calc_Fc_d(dy_h, M_h, D_h, v_h, M_o_h, true);
                
                % Nominal coupling force exerted on the robot
                F_c_r_d = this.calc_Fc_d(dy_r, M_r, D_r, v_r, M_o_r, true);
                % F_c_r_d = -M_o_r*grav;

                % Control applied by the human to track its reference and compensate for coupling forces
                u_h = v_h + F_c_h_d;
                 
                % Actual coupling forces exerted on the robot and the human
                [F_c_r, F_c_h, F_c_o] = this.calc_Fc(dy_r, M_r, D_r, v_r, dy_h, M_h, D_h, u_h, M_o, true);
                
                % Control applied by the robot to track its reference and compensate for coupling forces
                u_r = v_r + F_c_r;

                % Robot model dynamics
                ddy_r = inv(M_r) * ( - D_r*dy_r + u_r - F_c_r);

                % Human model dynamics
                ddy_h = inv(M_h) * ( - D_h*dy_h + u_h - F_c_h); 
                
                % Object model dynamics
                ddy_o = inv(M_o) * (F_c_o - w_o);


                %% Force error
                F_err_prev = F_err;
                F_err = this.k_Ferr*(-F_c_r + F_c_r_d);
%                 this.sigma_noise = 0.1*abs(F_err-F_err_prev).^2;


                %% ===========  DMP model online adaption  =========== 
                if (strcmpi(this.est_method,'KF'))
                    Sigma_w = this.dmp.update_weights_with_KF(x, F_err, 0.0, 0.0, Sigma_w, this.sigma_noise);
                    sn_a = 0.07;
                    this.sigma_noise = (1-sn_a)*this.sigma_noise + sn_a*1e-5;
                elseif (strcmpi(this.est_method,'RLS'))
                    Sigma_w = this.dmp.update_weights_with_RLS(x, F_err, 0.0, 0.0, Sigma_w, 0.98);
                elseif (strcmpi(this.est_method,'RLWR'))
                    Sigma_w = this.dmp.update_weights_with_RLWR(x, F_err, 0.0, 0.0, Sigma_w, 0.995);
                else
                    error('Unsopported estimation method: %s\n', this.est_method);
                end
                P_w = diag(Sigma_w);

                
                %%  ===========  Stopping criteria  ===========  
                err_p = max(abs(g0-y_r));
                if (err_p <= this.pos_tol_stop ...
                    && t>=tau && abs(dy_r)<this.vel_tol_stop && abs(dy_h)<this.vel_tol_stop)
                    break; 
                end

                iters = iters + 1;
                if (t>=this.max_sim_time_p*tau)
                    warning('Iteration limit reached. Stopping simulation...\n');
                    break;
                end

                %%  ===========  Numerical integration  ===========  
                t = t + dt;

                x = x + dx*dt;

                y_dmp = y_dmp + dy_dmp*dt;
                z = z + dz*dt;

                y_r = y_r + dy_r*dt;
                dy_r = dy_r + ddy_r*dt;

                y_h = y_h + dy_h*dt;
                dy_h = dy_h + ddy_h*dt;
                
                y_o = y_o + dy_o*dt;
                dy_o = dy_o + ddy_o*dt;

            end
            toc
            
            this.log_data = log_data;
        end
        
        %% Plot results
        function plotResults(this)
            
            log_data = this.log_data;
            dmp = this.dmp;
            N_kernels = dmp.getNumKernels();
            dt = this.sim_timestep;
            y0 = this.y0_ref;
            g0 = this.g_ref;
            
            Psi_data = zeros(length(this.dmp.w), length(log_data.x_data));
            for j=1:size(Psi_data,2)
                Psi_data(:,j) = dmp.kernelFunction(log_data.x_data(j));
            end

            fontsize = 14;
            lineWidth = 1.4;

            figure;
            subplot(3,1,1);
            plot(log_data.Time, log_data.y_dmp_data, 'm-.', log_data.Time, log_data.y_r_data, 'b-',log_data.Time, log_data.y_h_data, 'g-', log_data.Time, log_data.y_o_data, 'y-.', log_data.Time, log_data.y_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Position', 'FontSize',fontsize, 'Interpreter','latex');
            legend({'DMP','Robot','Human','Object','Ref'}, 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;
            subplot(3,1,2);
            plot(log_data.Time, log_data.dy_dmp_data, 'm-.', log_data.Time, log_data.dy_r_data, 'b-',log_data.Time, log_data.dy_h_data, 'g-', log_data.Time, log_data.dy_o_data, 'y-.', log_data.Time, log_data.dy_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Velocity', 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;
            subplot(3,1,3);
            plot(log_data.Time, log_data.ddy_dmp_data, 'm-.', log_data.Time, log_data.ddy_r_data, 'b-',log_data.Time, log_data.ddy_h_data, 'g-', log_data.Time, log_data.ddy_o_data, 'y-.', log_data.Time, log_data.ddy_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Acceleration', 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m/s^2$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;

            figure;
            t0 = log_data.Time(1);
            tend = log_data.Time(end);
            plot(log_data.Time,log_data.F_c_r_data,'b-' ,log_data.Time,log_data.F_c_h_data,'g-', log_data.Time, log_data.F_c_r_d_data,'r--', log_data.Time, log_data.F_c_h_d_data,'m--','LineWidth',lineWidth);
            title('Forces exterted on robot and human', 'FontSize',14, 'Interpreter','latex');
            legend({'$F_{c,r}$','$F_{c,h}$','$Load_{robot}$','$Load_{human}$'}, 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$N$]', 'FontSize',fontsize, 'Interpreter','latex');
            xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;


            tend = log_data.Time(end);
            n = size(log_data.w_dmp_data,2);
            step = floor(n/7);
            w_ind = 1:step:n;
            if (w_ind(end) ~= n)
                w_ind = [w_ind n];
            end
            n = length(w_ind);
            legend_str = cell(n,1);
            min_dy = 1e50;
            max_dy = -min_dy;
            t_data = log_data.Time(w_ind);
            w_data = log_data.w_dmp_data(:,w_ind);

            Colors = [153 153   0;
                        0 255   0;
                        0 204 102;
                        0 153 153;
                        0  51 255;
                        0   0 153;
                      102   0 102;
                      200   0 200;
                      255   0 255;
                      255  45 255;
                      255 145 255;
                     ]/255;

            figure;
            hold on;
            for i=1:n
                t = t_data(i);
                dmp.w = w_data(:,i);

                legend_str{i} = ['t = ' num2str(t) ' s'];

                [Time, y, dy, ddy] = DMP_sim(dmp, dt, tend, y0, g0);

                dy_low = min(dy);
                dy_up = max(dy);
                if (min_dy > dy_low), min_dy=dy_low; end
                if (max_dy < dy_up), max_dy=dy_up; end

                plot(Time, dy, 'Color',Colors(i,:), 'LineWidth',1.0+i*0.15);
            end
            plot(log_data.Time, log_data.dy_ref_data, 'Color','red', 'LineStyle',':', 'LineWidth',2.0);
            legend_str = [legend_str; 'ref'];
            title('DMP velocity profile adaption', 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
            xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
            legend(legend_str, 'FontSize',fontsize, 'Interpreter','latex');

            for i=1:n
                t = t_data(i);
            %     plot([t t], [min_dy-0.05 max_dy+0.05], 'Color',Colors(i,:), 'LineStyle','--', 'LineWidth',1.0);
                plot([t], [min_dy], 'Color',Colors(i,:), 'MarkerSize',14, 'Marker','*', 'LineStyle','--', 'LineWidth',3.0);
            end
            axis tight;
            hold off;

            ind = 1:5:N_kernels;
            figure;
            subplot(3,1,1);
            plot(log_data.Time, log_data.P_w_data(ind,:), 'LineWidth', 1.5);
            xlim([log_data.Time(1) log_data.Time(end)]);
            title('Increamental LWR - covariance evolution', 'FontSize',fontsize, 'Interpreter','latex');
            subplot(3,1,2);
            plot(log_data.Time, Psi_data(ind,:), 'LineWidth', 1.5);
            xlim([log_data.Time(1) log_data.Time(end)]);
            title('Kenrel activations', 'FontSize',fontsize, 'Interpreter','latex');
            subplot(3,1,3);
            stem(dmp.c * dmp.getTau(), dmp.w);
            xlim([log_data.Time(1) log_data.Time(end)]);
            title('DMP weights', 'FontSize',fontsize, 'Interpreter','latex');
            xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');

        end

        function Fc_d = calc_Fc_d(this, dy, M, D, u, M_o, is_stiff)

            A = [M (~is_stiff); M_o -1];
            b = [-D*dy+u; -M_o*this.grav];
            X = A\b;
            Fc_d = X(2);
                
        end
        
        function [F_c_r, F_c_h, F_c_o] = calc_Fc(this, dy_r, M_r, D_r, v_r, dy_h, M_h, D_h, u_h, M_o, is_stiff)
            
            A = [M_r (~is_stiff) 0; M_h 0 1; M_o -1 -1];
            b = [-D_r*dy_r+v_r; -D_h*dy_h+u_h; -M_o*this.grav];
            X = A\b;
            % ddy = X(1);
            F_c_r = X(2);
            F_c_h = X(3);
            F_c_o = (F_c_r + F_c_h);
            
        end
        
    end
end
