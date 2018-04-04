%% OL_1D_DMP_KF_nod class
%  OL: Object load.
%  1D: 1 dimension (gravity direction)
%  DMP_KF: DMP with Kalman-filter update for the weights
%  nod: No object dynamics are included.
%

classdef OL_1D_DMP_KF_nod < handle
    properties
        %% DMP parameters
        dmp % dmp model
        canClockPtr % canonical clock pointer
        shapeAttrGatingPtr % shape attractor gating function pointer
        goalAttrGatingPtr % goal attractor gating function pointer
        D_z % dmp damping
        K_z % dmp stiffness
        N_kernels % number of kernels used in the DMP
        kernelStdScaling % scaling factor for the kernels std
        train_method % 'LWR', 'LS', 'RLS' , 'RLWR'
        lambda % forgetting factor for recursive training methods
        sigma_dmp_w % initial value of covariance matrix for dmp weights (used in online updates)
        sigma_noise % variance of noise in force error measurement
        k_Ferr % gain applied to force error used to updata the dmp weights
        dmp_tau % dmp duration
        train_dmp_offline % if true trains offline the DMP (before online updata/execution)

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
        Mo % object's mass
        
        %% Object lifting params
        load_r_p % percent of the object load carried by the robot
        load_h_p % percent of the object load carried by the human
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
        function this = OL_1D_DMP_KF_nod()
            
            setMatlabPath();
            this.init();
            
            disp('Object lift - 1D - DMP with KF update - no object dynamics');
            
        end

        %% Initialization
        function init(this)

            this.initProgramArguments();
            this.initProgramVariables();

        end

        function initProgramArguments(this)
            
            %% DMP parameters
            this.D_z = 50.0;
            this.K_z = 0;
            this.dmp_tau = 10; % 10 sec
            this.N_kernels = 50 * this.dmp_tau;
            this.kernelStdScaling = 1.0;
            this.train_method = 'LWR';
            this.lambda = 0.98;
            this.sigma_dmp_w = 1e2;
            this.k_Ferr = 1.0;
            this.sigma_noise = 1e-2;
            this.train_dmp_offline = false;
            
            
            %% Reference model
            this.y0_ref = 0.0;
            this.g_ref = 1.5;
            this.a6 = 20.0;
            this.a7 = 0.0;
            this.tau_ref = 1.2;


            %% Human model
            this.M_h = 1.0;
            this.K_h = 250.0;
            this.D_h = 2*sqrt(this.M_h*this.K_h);

            
            %% Robot model
            this.M_r = 1.0;
            this.K_r = 0.0;
            this.D_r = 100; %this.D_z; %2*sqrt(cmd_args.M_r*cmd_args.K_r);

            
            %% Object model
            this.Mo = 10.0;
            this.load_r_p = 0.7;
            this.load_h_p = 0.3;
            this.load_h_p_var = 0.0;

            
            %% Simulation params
            this.sim_timestep = 0.002;
            this.pos_tol_stop = 0.01;
            this.vel_tol_stop = 0.005;
            this.max_sim_time_p = 3.0;
            
        end
        
        function initProgramVariables(this)
            
            %% global params
            this.grav = 9.81;
            
            
            %% Set up Reference model
            this.ref_model = RefModel(this.y0_ref, this.g_ref, this.tau_ref, this.a6, this.a7);
            
            
            %% Set up DMP model
            this.canClockPtr = LinCanonicalClock(this.dmp_tau);
            this.shapeAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
            this.goalAttrGatingPtr = ConstGatingFunction(1.0, 1.0);
            this.dmp = DMP_VT(this.N_kernels, this.D_z, this.K_z/this.D_z, this.canClockPtr, this.shapeAttrGatingPtr, this.goalAttrGatingPtr, this.kernelStdScaling);
            
            %% Init log data struct
            this.log_data = struct('Time',[], ...,
                                  'y_dmp_data',[],'dy_dmp_data',[],'ddy_dmp_data',[],'x_data',[], 'w_dmp_data',[], ...
                                  'P_lwr_data',[], ...
                                  'y_r_data',[],'dy_r_data',[],'ddy_r_data',[], 'u_r_data',[], 'F_ext_r_data',[], ...
                                  'y_ref_data',[],'dy_ref_data',[],'ddy_ref_data',[], ...
                                  'y_h_data',[],'dy_h_data',[],'ddy_h_data',[], 'u_h_data',[], 'F_ext_h_data',[],  ...
                                  'F_err_data',[], 'w_r_data',[], 'w_h_data',[], ...
                                  'F_c_data',[], 'F_c1_data',[], 'F_c2_data',[]);
            
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

            this.canClockPtr.setTau(this.dmp_tau);
            
        end
        
        %% Run simulation
        function simulation(this)
            
            %% copy some pointers for ease of use
            M_r = this.M_r;  D_r = this.D_r;  K_r = this.K_r;
            M_h = this.M_h;  D_h = this.D_h;  K_h = this.K_h;
            log_data = this.log_data;
            
            %% set initial values
            y0 = this.y0_ref;
            g0 = this.g_ref; 
            g = g0; 
            x = 0.0;
            dx = 0.0;
            t = 0.0;

            y_dmp = y0;
            dy_dmp = 0.0;
            ddy_dmp = 0.0;
            z = 0;
            dz = 0;

            w_o = this.Mo*this.grav;
            w_h = this.load_h_p*w_o;
            w_r = this.load_r_p*w_o;

            y_r = y0;
            dy_r = 0.0;
            ddy_r = 0.0;
            u_r = 0.0;
            F_ext_r = -w_r;

            y_h = y0;
            dy_h = 0.0;
            ddy_h = 0.0;
            u_h = 0.0;
            F_ext_h = -w_h;

            F_err = 0.0;

            F_c1 = 0.0;
            F_c2 = 0.0;

            P_lwr = this.sigma_dmp_w*ones(this.dmp.N_kernels,1);
            Sigma_w = diag(P_lwr);

            tau = this.tau_ref;
            dt = this.sim_timestep;

            iters = 0;

            %% Simulation
            disp('Simulation...')
            tic
            while (true)

                %% ===========  Get refernce trajectories =================

                %% get the human's model reference
                [y_ref, dy_ref, ddy_ref] = this.ref_model.getRef(t);

                %% get the DMP's model reference
                Y_c = 0.0;
                Z_c = 0.0;
                [dy_dmp, dz] = this.dmp.getStatesDot(x, y_dmp, z, y0, g, Y_c, Z_c);
                dx = this.canClockPtr.getPhaseDot(x);
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

                log_data.y_h_data = [log_data.y_h_data y_h];
                log_data.dy_h_data = [log_data.dy_h_data dy_h];   
                log_data.ddy_h_data = [log_data.ddy_h_data ddy_h];

                log_data.y_ref_data = [log_data.y_ref_data y_ref];
                log_data.dy_ref_data = [log_data.dy_ref_data dy_ref];   
                log_data.ddy_ref_data = [log_data.ddy_ref_data ddy_ref];

                log_data.u_h_data = [log_data.u_h_data u_h];
                log_data.F_ext_h_data = [log_data.F_ext_h_data F_ext_h];

                log_data.u_r_data = [log_data.u_r_data u_r];
                log_data.F_ext_r_data = [log_data.F_ext_r_data F_ext_r];

                log_data.F_err_data = [log_data.F_err_data F_err];

                log_data.w_dmp_data = [log_data.w_dmp_data this.dmp.w];

                log_data.P_lwr_data = [log_data.P_lwr_data P_lwr];

                log_data.w_r_data = [log_data.w_r_data w_r];
                log_data.w_h_data = [log_data.w_h_data w_h];

                log_data.F_c1_data = [log_data.F_c1_data F_c1];
                log_data.F_c2_data = [log_data.F_c2_data F_c2];


                %% Robot and Human model simulation

                % desired load carried by human
                a_h = this.load_h_p + (2*rand()-1)*this.load_h_p_var;
                w_h_hat = w_o*a_h; % simulates the scenario where the human  carries a varying load because he cannot estimate well how much force he applies

                % desired load carried by robot
                w_r = this.load_r_p*w_o;

                v_r = K_r*(y_dmp-y_r) + D_r*dy_dmp + M_r*ddy_dmp;

                % Force exerted by the human
                u_h = K_h*(y_ref-y_h) + D_h*dy_ref + M_h*ddy_ref + w_h_hat;

                F_c = w_h_hat + M_h * ( inv(M_r)*(-D_r*dy_r + v_r) - inv(M_h)*(-D_h*dy_h + u_h) );
   
                F_c1 =  M_h*(ddy_dmp-ddy_ref);
                F_c2 = ( D_h*(dy_h-dy_ref) - M_h*inv(M_r)*D_r*(dy_r-dy_dmp) - M_h*inv(M_r)*K_r*(y_r-y_dmp) + K_h*(y_h-y_ref) );
%                 F_c = F_c1 + F_c2;
   
                
                % External force exerted on the human
                F_ext_h = - w_h_hat + F_c;

                % External force exerted on the robot
                F_ext_r = - (w_o-w_h_hat) - F_c;

                % Force exerted by the robot
                u_r = v_r - F_ext_r;

                % Robot model dynamics
                ddy_r = inv(M_r) * ( - D_r*dy_r + u_r + F_ext_r);

                % Human model dynamics
                ddy_h = inv(M_h) * ( - D_h*dy_h + u_h + F_ext_h); 


                %% Force error
                F_err = this.k_Ferr*(F_ext_r + w_r);

                %% DMP model online adaption
                Sigma_w = this.dmp.update_weights_with_KF(x, F_err, 0.0, 0.0, Sigma_w, this.sigma_noise);
                P_lwr = diag(Sigma_w);

                %% Stopping criteria
                err_p = max(abs(g-y_r));
                if (err_p <= this.pos_tol_stop ...
                    && t>=tau && abs(dy_r)<this.vel_tol_stop && abs(dy_h)<this.vel_tol_stop)
                    break; 
                end

                iters = iters + 1;
                if (t>=this.max_sim_time_p*tau)
                    warning('Iteration limit reached. Stopping simulation...\n');
                    break;
                end

                %% Numerical integration
                t = t + dt;

                x = x + dx*dt;

                y_dmp = y_dmp + dy_dmp*dt;
                z = z + dz*dt;

                y_r = y_r + dy_r*dt;
                dy_r = dy_r + ddy_r*dt;

                y_h = y_h + dy_h*dt;
                dy_h = dy_h + ddy_h*dt;

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
            plot(log_data.Time, log_data.y_dmp_data, 'm-.', log_data.Time, log_data.y_r_data, 'b-',log_data.Time, log_data.y_h_data, 'g-', log_data.Time, log_data.y_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Position', 'FontSize',fontsize, 'Interpreter','latex');
            legend({'DMP','Robot','Human','Ref'}, 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;
            subplot(3,1,2);
            plot(log_data.Time, log_data.dy_dmp_data, 'm-.', log_data.Time, log_data.dy_r_data, 'b-',log_data.Time, log_data.dy_h_data, 'g-', log_data.Time, log_data.dy_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Velocity', 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;
            subplot(3,1,3);
            plot(log_data.Time, log_data.ddy_dmp_data, 'm-.', log_data.Time, log_data.ddy_r_data, 'b-',log_data.Time, log_data.ddy_h_data, 'g-', log_data.Time, log_data.ddy_ref_data, 'r-.', 'LineWidth',lineWidth);
            title('Acceleration', 'FontSize',fontsize, 'Interpreter','latex');
            ylabel('[$m/s^2$]', 'FontSize',fontsize, 'Interpreter','latex');
            axis tight;

            figure;
            t0 = log_data.Time(1);
            tend = log_data.Time(end);
            plot(log_data.Time,log_data.F_ext_r_data,'b-' ,log_data.Time,log_data.F_ext_h_data,'g-', log_data.Time,-log_data.w_r_data,'r--', log_data.Time,-log_data.w_h_data,'m--','LineWidth',lineWidth);
            title('Forces exterted on robot and human', 'FontSize',14, 'Interpreter','latex');
            legend({'$F_{ext,r}$','$F_{ext,h}$','$Load_{robot}$','$Load_{human}$'}, 'FontSize',fontsize, 'Interpreter','latex');
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
            plot(log_data.Time, log_data.P_lwr_data(ind,:), 'LineWidth', 1.5);
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
            
            figure;
            subplot(2,1,1);
            hold on;
            plot(log_data.Time, log_data.F_c1_data);
            plot(log_data.Time, log_data.F_c2_data);
            plot(log_data.Time, log_data.F_c1_data+log_data.F_c2_data);
            legend({'$F_{c1}$','$F_{c2}$','$F_c$'}, 'FontSize',fontsize, 'Interpreter','latex');
            hold off;
            subplot(2,1,2);
            plot(log_data.Time,log_data.ddy_dmp_data, log_data.Time,log_data.ddy_ref_data, 'LineWidth',1.5);
            legend({'$\ddot{y}_{dmp}$','$\ddot{y}_{ref}$'}, 'FontSize',fontsize, 'Interpreter','latex');
        end

    end
end
