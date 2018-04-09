%% OL_2D_DMP_rup class
%  OL: Object load.
%  2D: 2 dimensions
%  DMP_rup: DMP with recursive update method
%

classdef OL_2D_DMP_rup < handle
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
        s_n_start % initial value of noise
        s_n_end % final value of noise
        s_n_a % rate of exponential convergence from s_n_start to end s_n_end
        lambda % forgetting factor for recursive training methods
        k_Ferr % gain applied to force error used to updata the dmp weights

        %% Reference model
        tau_ref % reference model trajectory duration
        gp_ref % percentage of target reached with goal filtering at t=t_ref
        a6
        a7

        %% Human model
        M_h % inertia
        D_h % damping
        K_h % stiffness  
        m_o_h_p % percent of the object load carried by the human
        load_h_p_var % variance in the load carried by the human
        d_h_hat % initial estimate of human's distance from object's CoM

        %% Robot model
        M_r % inertia
        D_r % damping
        K_r % stiffness   
        m_o_r_p % object load carried by the robot
        d_r_hat % initial estimate of robot's distance from object's CoM

        %% Object Model
        M_o % object's inertia
        obj % struct with the length, height, CoM and mass of the object
        S0_o % initial object pose
        Sg_o % target/goal object pose

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
        ref_model % human's reference
        
        %% plot online
        pl_h % struct with plot handles
        pl_update_cycle % the timestep with which the plot is updated

    end

    methods
        %% constructor
        function this = OL_2D_DMP_rup()
            
            setMatlabPath();
            this.init();
            
            disp(['Object lift - 2D - DMP with ' this.est_method ' update - with object dynamics']);
            
        end

        %% Initialization
        function init(this)

            this.initProgramArguments();
            this.initProgramVariables();

        end

        %% Initializes the program's arguements
        function initProgramArguments(this)
            
            %% DMP parameters
            this.D_z = [50.0; 50.0; 20.0];
            this.K_z = [0.0; 0.0; 0.0];
            this.dmp_tau = 10; % 10 sec
            this.N_kernels = 50 * this.dmp_tau;
            this.kernel_std_scaling = 1.0;
            this.train_method = 'LWR';
            this.train_dmp_offline = false;
            
            %% recursive estimation parameters
            this.est_method = 'KF'; % {'KF', 'RLS', 'RLWR'}
            this.sigma_dmp_w = 1e2;
            this.s_n_start = 8e-1; % initial value of noise
            this.s_n_end = 2e-4; % final value of noise
            this.s_n_a = 0.07; % rate of exponential convergence from s_n_start to end s_n_end
            this.lambda = 0.99;
            % k_Ferr, defined at the end
            
            %% Object model
            this.obj = struct('m',[], 'Iz',[], 'h',[], 'l',[], 'CoM',[]);
            this.obj.m = 10.0; % object mass
            this.obj.Iz = 10.0; % object rotational mass in z axis
            this.obj.h = 0.15; % object height
            this.obj.l = 0.8; % object length
            this.obj.CoM = 0.4; % object's CoM (distance from its left end)
            this.S0_o = [0.0; 0.0; 0.0];
            this.Sg_o = [0.2; 0.5; 0.0];
            this.M_o = diag([this.obj.m; this.obj.m; this.obj.Iz]);

            
            %% Reference model
%             this.gp_ref = 0.995;
            this.a6 = 0.0;
            this.a7 = 0.0;
            this.tau_ref = 1.5;


            %% Human model
            this.M_h = diag([15.0; 15.0; 7]);
            this.K_h = diag([500.0; 500.0; 75.0]);
            this.D_h = 2*sqrt(this.M_h.*this.K_h);
            this.m_o_h_p = 0.2;
            this.load_h_p_var = 0.0;
            this.d_h_hat = this.obj.l - this.obj.CoM;

            
            %% Robot model
            this.M_r = diag([20.0; 20.0; 15]);
            this.K_r = diag([750.0; 750.0; 100.0]);
            this.D_r = 2*sqrt(this.M_r.*this.K_r);
            this.m_o_r_p = 0.8; 
            this.d_r_hat = this.obj.CoM;

            this.k_Ferr = 1./diag(this.M_h);
            
            %% Simulation params
            this.sim_timestep = 0.002;
            this.pos_tol_stop = 0.02;
            this.vel_tol_stop = 0.005;
            this.max_sim_time_p = 1.3;
            
        end
        
        %% Initializes the program's variables
        function initProgramVariables(this)
            
            %% global params
            this.grav = 9.81;
            
            
            %% Set up DMP model
            this.can_clock_ptr = LinCanonicalClock(this.dmp_tau);
            this.shape_attrGating_ptr = ConstGatingFunction(1.0, 1.0);
            this.goal_attrGating_ptr = ConstGatingFunction(1.0, 1.0);
            this.dmp = cell(3,1);
            for i=1:3
                this.dmp{i} = DMP_VT(this.N_kernels, this.D_z(i), this.K_z(i)/this.D_z(i), this.can_clock_ptr, this.shape_attrGating_ptr, this.goal_attrGating_ptr, this.kernel_std_scaling);
            end
            
            %% Init online plot
            this.initOnlinePlot();
            
            %% Init log data struct
            this.log_data = struct('Time',[], ...,
                                  'S_dmp_data',[],'dS_dmp_data',[],'ddS_dmp_data',[],'x_data',[], 'w_dmp_data',[], ...
                                  'P_w_data',[], ...
                                  'S_r_data',[],'dS_r_data',[],'ddS_r_data',[], 'U_r_data',[], 'F_c_r_data',[], ...
                                  'S_ref_data',[],'dS_ref_data',[],'ddS_ref_data',[], ...
                                  'S_h_data',[],'dS_h_data',[],'ddS_h_data',[], 'U_h_data',[], 'F_c_h_data',[],  ...
                                  'S_o_data',[],'dS_o_data',[],'ddS_o_data',[], 'F_c_o_data',[],  ...
                                  'F_err_data',[], 'F_c_r_d_data',[], 'F_c_h_d_data',[]);
            this.log_data.w_dmp_data = cell(3,1);
            this.log_data.P_w_data = cell(3,1);
            
            %% load train data
            load data/data.mat Ts Time_data Y_data dY_data ddY_data
            this.Ts = Ts;
            this.Time_data = Time_data;
            this.Y_data = Y_data;
            this.dY_data = dY_data;
            this.ddY_data = ddY_data;
            
        end
        
        %% Initializes the program's online object lifting plot
        function initOnlinePlot(this)

            this.pl_update_cycle = 0.015;
            
            ax = axes();
            
            % clear all previous graphics objects 
            cla(ax);
            
            % Enable 'hold on' on the ax object
            NextPlot0 = ax.NextPlot; %% save previous state
            ax.NextPlot = 'add';
            
            % set the size of the axes
            unitsPrev = ax.Parent.Units;
            ax.Parent.Units = 'normalized';
            ax.Parent.OuterPosition = [0 0 0.4 0.8];
            ax.Parent.Units = unitsPrev; % restore it to previous value
            
            %% ================================================
            %% =====  Define parameters for plotting  =========
            %% ================================================
            
            ax_length = 0.2;
            
            frameText_offset = [-0.085 -0.065 0.0];
            frame_label_fontsize = 15;
            frame_label_color = [0 0 0];
            text_fontsize = 16;
            text_interp = 'latex';
            
            quiv_arrowHead_size = 0.3;
            
            axis_lineStyle = '-';
            axis_width = 3.5;
            x_ax_color = [1 0 0];
            y_ax_color = [0 1 0];
            obj_body_color = [0.87 0.49 0.0]; % light brown
            obj_width = 20.0;
            
            obj_frameTxt = '\{O\}';
            robot_frameTxt = '\{R\}';
            human_frameTxt = '\{H\}';
            
            frame_center_marker = '*';
            frame_center_color = [0 0 1];
            frame_center_size = 13;
            frame_center_lineWidth = 3.0;
            
            Fc_color = [1 0 1];
            Fc_scale = 0.005;
            Fc_lineStyle = '-';
            Fc_lineWidth = 2.0;
            Fc_text_offset = [-0.2 -0.2 0];
            Fc_label_fontsize = 16;
            Fc_text_color = [0 0 0];
            
            
            %% ==========================
            axis(ax, 'equal');
            
            % define the limits of the figure
            ax.XLim = [-1 1] + this.S0_o(1);
            ax.YLim = [-1 1] + this.S0_o(2);
            
            % define the figure x-label
            ax.XLabel.String = 'x [$m$]';
            ax.XLabel.Interpreter = text_interp;
            ax.XLabel.FontSize = text_fontsize;

            % define the figure y-label
            ax.YLabel.String = 'y [$m$]';
            ax.YLabel.Interpreter = text_interp;
            ax.YLabel.FontSize = text_fontsize;

            % define the figure title
            ax.Title.String = 'Human-robot object lifting in $2D$';
            ax.Title.Interpreter = text_interp;
            ax.Title.FontSize = text_fontsize;

            %% =======================================
            %% =============   Object   ==============
            %% =======================================
            
            % Object body
            objLine = line(ax);
            objLine.XData = [];
            objLine.YData = [];
            objLine.Color = obj_body_color; 
            objLine.LineWidth = obj_width;
            objLine.LineStyle = '-';

            % Object frame x-axis
            objXax = quiver(ax,0,0,0,0);
            objXax.Color = x_ax_color;
        	objXax.LineStyle = axis_lineStyle;
        	objXax.LineWidth = axis_width;    
        	objXax.AutoScale = 'off';
            objXax.MaxHeadSize = quiv_arrowHead_size;
            
            % Object frame y-axis
            objYax = quiver(ax,0,0,0,0);
            objYax.Color = y_ax_color;
        	objYax.LineStyle = axis_lineStyle;
        	objYax.LineWidth = axis_width;    
        	objYax.AutoScale = 'off';
            objYax.MaxHeadSize = quiv_arrowHead_size;
            
            % Object frame center
            objCenter = line(ax);
            objCenter.Marker = frame_center_marker;
            objCenter.MarkerSize = frame_center_size;
            objCenter.LineWidth = frame_center_lineWidth;
            objCenter.Color = frame_center_color;
            objCenter.XData = [];
            objCenter.YData = [];

            % Object frame text
            objFrameTxt = text(ax, 0.0 , 0.0, obj_frameTxt, 'FontSize', frame_label_fontsize, 'FontWeight', 'bold', 'Color', frame_label_color);
            
            %% ======================================
            %% =============   Robot   ==============
            %% ======================================

            % Robot frame x-axis
            robotXax = quiver(ax,0,0,0,0);
            robotXax.Color = x_ax_color;
        	robotXax.LineStyle = axis_lineStyle;
        	robotXax.LineWidth = axis_width;    
        	robotXax.AutoScale = 'off';
            robotXax.MaxHeadSize = quiv_arrowHead_size;
            
            % Robot frame y-axis
            robotYax = quiver(ax,0,0,0,0);
            robotYax.Color = y_ax_color;
        	robotYax.LineStyle = axis_lineStyle;
        	robotYax.LineWidth = axis_width;    
        	robotYax.AutoScale = 'off';
            robotYax.MaxHeadSize = quiv_arrowHead_size;
            
            % Robot frame center
            robotCenter = line(ax);
            robotCenter.Marker = frame_center_marker;
            robotCenter.MarkerSize = frame_center_size;
            robotCenter.LineWidth = frame_center_lineWidth;
            robotCenter.Color = frame_center_color;
            robotCenter.XData = [];
            robotCenter.YData = [];
            
            % Robot frame text
            robotFrameTxt = text(ax, 0.0 , 0.0, robot_frameTxt, 'FontSize', frame_label_fontsize, 'FontWeight', 'bold', 'Color', frame_label_color);
            
            %% ======================================
            %% =============   Human   ==============
            %% ======================================

            % Human frame x-axis
            humanXax = quiver(ax,0,0,0,0);
            humanXax.Color = x_ax_color;
        	humanXax.LineStyle = axis_lineStyle;
        	humanXax.LineWidth = axis_width;    
        	humanXax.AutoScale = 'off';
            humanXax.MaxHeadSize = quiv_arrowHead_size;
            
            % Human frame y-axis
            humanYax = quiver(ax,0,0,0,0);
            humanYax.Color = y_ax_color;
        	humanYax.LineStyle = axis_lineStyle;
        	humanYax.LineWidth = axis_width;    
        	humanYax.AutoScale = 'off';
            humanYax.MaxHeadSize = quiv_arrowHead_size;
            
            % Human frame center
            humanCenter = line(ax);
            humanCenter.Marker = frame_center_marker;
            humanCenter.MarkerSize = frame_center_size;
            humanCenter.LineWidth = frame_center_lineWidth;
            humanCenter.Color = frame_center_color;
            humanCenter.XData = [];
            humanCenter.YData = [];
            
            % Human frame text
            humanFrameTxt = text(ax, 0.0 , 0.0, human_frameTxt, 'FontSize', frame_label_fontsize, 'FontWeight', 'bold', 'Color', frame_label_color);
    
            %% ===============================
            %% ========= Goal frame ==========
            %% ===============================
            p_g = this.Sg_o(1:2);
            theta_g = this.Sg_o(3);
            goalAx_x = p_g + this.rotz(theta_g)*[ax_length; 0];
            goalAx_y = p_g + this.rotz(theta_g)*[0; ax_length];
            
            % Goal frame x-axis     
            goalXax = quiver(ax,0,0,0,0);
            goalXax.XData = p_g(1);
            goalXax.YData = p_g(2);
            goalXax.UData = goalAx_x(1)-p_g(1);
            goalXax.VData = goalAx_x(2)-p_g(2);
            goalXax.Color = x_ax_color;
        	goalXax.LineStyle = '--';
        	goalXax.LineWidth = axis_width;    
        	goalXax.AutoScale = 'off';
            goalXax.MaxHeadSize = quiv_arrowHead_size;
            
            % Goal frame y-axis
            goalYax = quiver(ax,0,0,0,0);
            goalYax.XData = p_g(1);
            goalYax.YData = p_g(2);
            goalYax.UData = goalAx_y(1)-p_g(1);
            goalYax.VData = goalAx_y(2)-p_g(2);
            goalYax.Color = y_ax_color;
        	goalYax.LineStyle = '--';
        	goalYax.LineWidth = axis_width;    
        	goalYax.AutoScale = 'off';
            goalYax.MaxHeadSize = quiv_arrowHead_size;
            
            % Goal frame center
            goalCenter = line(ax);
            goalCenter.Marker = frame_center_marker;
            goalCenter.MarkerSize = frame_center_size;
            goalCenter.LineWidth = frame_center_lineWidth;
            goalCenter.Color = frame_center_color;
            goalCenter.XData = p_g(1);
            goalCenter.YData = p_g(2);
            
            % Goal frame text
            goalFrameTxt = text(ax, 0.0 , 0.0, '\{Goal\}', 'FontSize', frame_label_fontsize, 'FontWeight', 'bold', 'Color', frame_label_color);
            goalFrameTxt.Position = [p_g(1) p_g(2) 0] + frameText_offset;
            
            
            %% =====================================================
            %% =============   Robot coupling Force   ==============
            %% =====================================================   
            
            % Robot coupling Force arrow
            Fc_r_quiv = quiver(ax,0,0,0,0);
            Fc_r_quiv.Color = Fc_color;
        	Fc_r_quiv.LineStyle = Fc_lineStyle;
        	Fc_r_quiv.LineWidth = Fc_lineWidth;    
        	Fc_r_quiv.AutoScale = 'off';
            Fc_r_quiv.MaxHeadSize = quiv_arrowHead_size;
            
            % Robot coupling Force text
            Fc_r_label = text(ax, ax.XLim(1)+0.1 , ax.YLim(1)+0.2, 'F_{c,r}', 'FontSize', Fc_label_fontsize, 'FontWeight', 'normal', 'Color', Fc_text_color);
            Fc_r_arrow_text = text(ax, 0 , 0, 'F_{c,r}', 'FontSize', Fc_label_fontsize, 'FontWeight', 'normal', 'Color', Fc_text_color);
            
            %% =====================================================
            %% =============   Human coupling Force   ==============
            %% =====================================================   
            
            % Human coupling Force arrow
            Fc_h_quiv = quiver(ax,0,0,0,0);
            Fc_h_quiv.Color = Fc_color;
        	Fc_h_quiv.LineStyle = Fc_lineStyle;
        	Fc_h_quiv.LineWidth = Fc_lineWidth;    
        	Fc_h_quiv.AutoScale = 'off';
            Fc_h_quiv.MaxHeadSize = quiv_arrowHead_size;
            
            % Human coupling Force text
            Fc_h_label = text(ax, ax.XLim(1)+0.1 , ax.YLim(1)+0.1, 'F_{c,h}', 'FontSize', Fc_label_fontsize, 'FontWeight', 'normal', 'Color', Fc_text_color);
            Fc_h_arrow_text = text(ax, 0 , 0, 'F_{c,h}', 'FontSize', Fc_label_fontsize, 'FontWeight', 'normal', 'Color', Fc_text_color);
            
            %% ==========================================================
            %% ==========================================================
            
            this.pl_h = struct('ax',ax, 'ax_length',ax_length, 'frameText_offset',frameText_offset, ...
                               'objCenter',objCenter, 'objLine',objLine, 'objXax',objXax, 'objYax',objYax, 'objFrameTxt',objFrameTxt, ...
                               'robotCenter',robotCenter, 'robotXax',robotXax, 'robotYax',robotYax, 'robotFrameTxt',robotFrameTxt, ...
                               'humanCenter',humanCenter, 'humanXax',humanXax, 'humanYax',humanYax, 'humanFrameTxt',humanFrameTxt, ...
                               'Fc_r_quiv',Fc_r_quiv, 'Fc_r_label',Fc_r_label, 'Fc_h_quiv',Fc_h_quiv, 'Fc_h_label',Fc_h_label, ...
                               'Fc_r_arrow_text',Fc_r_arrow_text, 'Fc_h_arrow_text',Fc_h_arrow_text, ...
                               'Fc_scale',Fc_scale, 'Fc_text_offset',Fc_text_offset);
            
        end
            
        %% Train the dmp model
        function trainModel(this)
            
            if (this.train_dmp_offline)
                trainParamsName = {'lambda', 'P_cov'};
                trainParamsValue = {this.lambda, this.sigma_dmp_w};
                disp('DMP training...')
                tic
                for i=1:3
                    this.dmp{i}.setTrainingParams(this.train_method, trainParamsName, trainParamsValue);
                    this.dmp{i}.trainMulti(this.Time_data{i}, this.Y_data{i}, this.dY_data{i}, this.ddY_data{i});   
                end
                toc
            else
                disp('No training. Initializing DMP with zero weights...')
                for i=1:3
                    this.dmp{i}.w = zeros(size(this.dmp{i}.w));
                end
            end

            this.can_clock_ptr.setTau(this.dmp_tau);
            
        end
        
        %% Run simulation
        function simulation(this)
            
            %% =========== set initial values ==============
            
            t = 0.0;
            dt = this.sim_timestep;
            iters = 0;
            
            % =========== robot =========== 
            d_r_norm = this.obj.CoM; % distance of robot from object CoM
            S_r = this.findPoseFromPose(this.S0_o, d_r_norm);
            dS_r = zeros(3,1);
            ddS_r = zeros(3,1);
            U_r = zeros(3,1);
            F_c_r = zeros(3,1);
            F_c_r_d = zeros(3,1);
            M_o_r = this. m_o_r_p*this.M_o;

            % =========== dmp =========== 
            S_dmp = S_r;
            dS_dmp = zeros(3,1);
            ddS_dmp = zeros(3,1);
            x = 0.0;
            dx = 0.0;
            
            F_err_prev = zeros(3,1);
            F_err = zeros(3,1);

            P_w = this.sigma_dmp_w*ones(this.N_kernels,1);
            Sigma_w = diag(P_w);
            P_w = {P_w; P_w; P_w};
            Sigma_w = {Sigma_w; Sigma_w; Sigma_w};
            
            sigma_noise = this.s_n_start;

            % =========== human =========== 
            d_h_norm = this.obj.l - this.obj.CoM; % distance of human from object CoM
            S_h = this.findPoseFromPose(this.S0_o, -d_h_norm);
            S_g_h = this.findPoseFromPose(this.Sg_o, -d_h_norm); % human's goal pose
            dS_h = zeros(3,1);
            ddS_h = zeros(3,1);
            U_h = zeros(3,1);
            F_c_h = zeros(3,1);
            F_c_h_d = zeros(3,1);
            M_o_h = this. m_o_h_p*this.M_o;
   
            % =========== human reference =========== 
            this.ref_model = cell(3,1);
            for i=1:3
               this.ref_model{i} = RefModel(S_h(i), S_g_h(i), this.tau_ref, this.a6, this.a7); 
            end
            
            S_ref = S_h;
            dS_ref = zeros(3,1);
            ddS_ref = zeros(3,1);
            
            % =========== object =========== 
            S_o = this.S0_o;
            dS_o = zeros(3,1);
            ddS_o = zeros(3,1);
            F_c_o = F_c_r+F_c_h;
            w_o = [0; -this.obj.m*this.grav; 0];

            plot_t = 0.0;

            %% =========== Simulation =========== 
            disp('Simulation...')
            tic
            while (true)

                d_r = S_o(1:2) - S_r(1:2);
                d_h = S_o(1:2) - S_h(1:2);
                
                this.d_r_hat = d_r;
                this.d_h_hat = d_h;
                
                %% ===========  Get refernce trajectories  =================

                % ===========  get the human's model reference ===========  
                [S_ref, dS_ref, ddS_ref] = this.getHumanRef(t);

                % ===========  get the robot's reference (DMP model) ===========  
                Y_c = zeros(3,1);
                Z_c = zeros(3,1); % 0.97*F_err;
                [ddS_dmp, dx] = this.getRobotRef(x, S_dmp, dS_dmp, Y_c, Z_c);

                %% ===========  data logging ===========  

                this.logData(t, x, ...
                              S_dmp,dS_dmp,ddS_dmp, P_w, F_err, ...
                              S_r,dS_r,ddS_r, U_r, F_c_r, F_c_r_d, ...
                              S_h,dS_h,ddS_h, U_h, F_c_h, F_c_h_d, ...
                              S_o,dS_o,ddS_o, F_c_o, ...
                              S_ref,dS_ref,ddS_ref);
                          

                %% ===========  Robot and Human model simulation  ===========  

                % Control applied by the robot to track its reference
                V_r = this.K_r*(S_dmp-S_r) + this.D_r*dS_dmp + this.M_r*ddS_dmp;

                % Control applied by the human to track its reference
                V_h = this.K_h*(S_ref-S_h) + this.D_h*dS_ref + this.M_h*ddS_ref;
                
                % Nominal coupling force exerted on the human
                F_c_h_d = this.calc_Fc_d(dS_h, this.M_h, this.D_h, V_h, this.d_h_hat, dS_o, M_o_h, true);
                
                % Nominal coupling force exerted on the robot
                F_c_r_d = this.calc_Fc_d(dS_r, this.M_r, this.D_r, V_r, this.d_r_hat, dS_o, M_o_r, true);
                % F_c_r_d = -m_o_r*this.grav;
                
                % Control applied by the human to track its reference and compensate for coupling forces
                U_h = V_h + F_c_h_d .* (1 + (2*rand()-1)*this.load_h_p_var*diag(M_o_h));
                 
                % Actual coupling forces exerted on the robot and the human
                [F_c_r, F_c_h, F_c_o, ddS] = this.calc_Fc(dS_r, this.M_r, this.D_r, V_r, d_r, dS_h, this.M_h, this.D_h, U_h, d_h, dS_o, this.M_o, true);
                
                % Control applied by the robot to track its reference and compensate for coupling forces
                U_r = V_r + F_c_r;

                % Robot model dynamics
                ddS_r = inv(this.M_r) * ( - this.D_r*dS_r + U_r - F_c_r);

                % Human model dynamics
                ddS_h = inv(this.M_h) * ( - this.D_h*dS_h + U_h - F_c_h); 
                
                % Object model dynamics
                ddS_o = inv(this.M_o) * (F_c_o + w_o);
                
                
                %% Force error
                F_err_prev = F_err;
                F_err = this.k_Ferr.*(-F_c_r + F_c_r_d);
%                 this.sigma_noise = 0.1*abs(F_err-F_err_prev).^2;


                %% ===========  DMP model online adaption  =========== 
                for i=1:3
                    if (strcmpi(this.est_method,'KF'))
                        Sigma_w{i} = this.dmp{i}.update_weights_with_KF(x, F_err(i), 0.0, 0.0, Sigma_w{i}, sigma_noise);
                        sigma_noise = (1-this.s_n_a)*sigma_noise + this.s_n_a*this.s_n_end;
                    elseif (strcmpi(this.est_method,'RLS'))
                        Sigma_w{i} = this.dmp{i}.update_weights_with_RLS(x, F_err(i), 0.0, 0.0, Sigma_w{i}, this.lambda);
                    elseif (strcmpi(this.est_method,'RLWR'))
                        Sigma_w{i} = this.dmp{i}.update_weights_with_RLWR(x, F_err(i), 0.0, 0.0, Sigma_w{i}, this.lambda);
                    else
                        error('Unsopported estimation method: %s\n', this.est_method);
                    end
                    P_w{i} = diag(Sigma_w{i});
                end
                
                %%  ===========  plot online  ===========  
                if (plot_t >= this.pl_update_cycle)
                    this.plotOnline(S_r, S_h, S_o, F_c_r, F_c_h, 0);
                    plot_t = 0.0;
                end

                
                %%  ===========  Stopping criteria  ===========  
                pos_err = max(norm(S_g_h-S_h));
                if (this.stopSimulation(t, pos_err, dS_r, dS_h)), break; end

                %%  ===========  Numerical integration  =========== 
                iters = iters + 1;
                
                plot_t = plot_t + dt;
                
                t = t + dt;

                x = x + dx*dt;

                S_ref = S_ref + dS_ref*dt;
                dS_ref = dS_ref + ddS_ref*dt;
                
                S_dmp = S_dmp + dS_dmp*dt;
                dS_dmp = dS_dmp + ddS_dmp*dt;

                S_r = S_r + dS_r*dt;
                dS_r = dS_r + ddS_r*dt;

                S_h = S_h + dS_h*dt;
                dS_h = dS_h + ddS_h*dt;
                
                S_o = S_o + dS_o*dt;
                dS_o = dS_o + ddS_o*dt;

            end
            toc

        end
        
        %% Plot results
        function plotResults(this)
            
            log_data = this.log_data;
            dmp = this.dmp;
            N_kernels = this.N_kernels;
            dt = this.sim_timestep;
            S0 = this.S0_o;
            Sg = this.Sg_o;
            
            Psi_data = zeros(this.N_kernels, length(log_data.x_data));
            for j=1:size(Psi_data,2)
                Psi_data(:,j) = dmp{1}.kernelFunction(log_data.x_data(j));
            end

            fontsize = 14;
            lineWidth = 1.4;
            
            for k=1:2
                
                figure;
                subplot(3,1,1);
                plot(log_data.Time, log_data.S_dmp_data(k,:), 'm-.', log_data.Time, log_data.S_r_data(k,:), 'b-',log_data.Time, log_data.S_h_data(k,:), 'g-', log_data.Time, log_data.S_o_data(k,:), 'y-.', log_data.Time, log_data.S_ref_data(k,:), 'r-.', 'LineWidth',lineWidth);
                title('Position', 'FontSize',fontsize, 'Interpreter','latex');
                legend({'DMP','Robot','Human','Object','Ref'}, 'FontSize',fontsize, 'Interpreter','latex');
                ylabel('[$m$]', 'FontSize',fontsize, 'Interpreter','latex');
                axis tight;
                subplot(3,1,2);
                plot(log_data.Time, log_data.dS_dmp_data(k,:), 'm-.', log_data.Time, log_data.dS_r_data(k,:), 'b-',log_data.Time, log_data.dS_h_data(k,:), 'g-', log_data.Time, log_data.dS_o_data(k,:), 'y-.', log_data.Time, log_data.dS_ref_data(k,:), 'r-.', 'LineWidth',lineWidth);
                title('Velocity', 'FontSize',fontsize, 'Interpreter','latex');
                ylabel('[$m/s$]', 'FontSize',fontsize, 'Interpreter','latex');
                axis tight;
                subplot(3,1,3);
                plot(log_data.Time, log_data.ddS_dmp_data(k,:), 'm-.', log_data.Time, log_data.ddS_r_data(k,:), 'b-',log_data.Time, log_data.ddS_h_data(k,:), 'g-', log_data.Time, log_data.ddS_o_data(k,:), 'y-.', log_data.Time, log_data.ddS_ref_data(k,:), 'r-.', 'LineWidth',lineWidth);
                title('Acceleration', 'FontSize',fontsize, 'Interpreter','latex');
                ylabel('[$m/s^2$]', 'FontSize',fontsize, 'Interpreter','latex');
                axis tight;

                figure;
                t0 = log_data.Time(1);
                tend = log_data.Time(end);
                plot(log_data.Time,log_data.F_c_r_data(k,:),'b-' ,log_data.Time,log_data.F_c_h_data(k,:),'g-', log_data.Time, log_data.F_c_r_d_data(k,:),'r--', log_data.Time, log_data.F_c_h_d_data(k,:),'m--','LineWidth',lineWidth);
                title('Forces exterted on robot and human', 'FontSize',14, 'Interpreter','latex');
                legend({'$F_{c,r}$','$F_{c,h}$','$Load_{robot}$','$Load_{human}$'}, 'FontSize',fontsize, 'Interpreter','latex');
                ylabel('[$N$]', 'FontSize',fontsize, 'Interpreter','latex');
                xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
                axis tight;


                tend = log_data.Time(end);
                n = size(log_data.w_dmp_data{k},2);
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
                w_data = log_data.w_dmp_data{k}(:,w_ind);

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
                    dmp{k}.w = w_data(:,i);

                    legend_str{i} = ['t = ' num2str(t) ' s'];

                    [Time, y, dy, ddy] = DMP_sim(dmp{k}, dt, tend, S0(k), Sg(k));

                    dy_low = min(dy);
                    dy_up = max(dy);
                    if (min_dy > dy_low), min_dy=dy_low; end
                    if (max_dy < dy_up), max_dy=dy_up; end

                    plot(Time, dy, 'Color',Colors(i,:), 'LineWidth',1.0+i*0.15);
                end
                plot(log_data.Time, log_data.dS_ref_data(k,:), 'Color','red', 'LineStyle',':', 'LineWidth',2.0);
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
                plot(log_data.Time, log_data.P_w_data{k}(ind,:), 'LineWidth', 1.5);
                xlim([log_data.Time(1) log_data.Time(end)]);
                title('Increamental LWR - covariance evolution', 'FontSize',fontsize, 'Interpreter','latex');
                subplot(3,1,2);
                plot(log_data.Time, Psi_data(ind,:), 'LineWidth', 1.5);
                xlim([log_data.Time(1) log_data.Time(end)]);
                title('Kenrel activations', 'FontSize',fontsize, 'Interpreter','latex');
                subplot(3,1,3);
                stem(dmp{k}.c * dmp{k}.getTau(), dmp{k}.w);
                xlim([log_data.Time(1) log_data.Time(end)]);
                title('DMP weights', 'FontSize',fontsize, 'Interpreter','latex');
                xlabel('time [$s$]', 'FontSize',fontsize, 'Interpreter','latex');
                
            end

        end

        %% Calculate desired/expected coupling force between robot (or human) and object
        function [Fc_d, ddS] = calc_Fc_d(this, dS, M, D, U, d, dS_o, M_o, is_stiff)
            
            m_o = M_o(1,1);
            w_o = [0; -m_o*this.grav; 0];
            
            dtheta_o = dS_o(3);
            dp_c = [d*dtheta_o^2; 0];
            
            A = [M*this.gMat(-d) ,  (~is_stiff)*eye(3); 
                      M_o        ,     -this.gMat(d)' ];
                
            b = [-D*dS + U - M*dp_c;
                       w_o         ];
                   
            X = A\b;

            ddS = X(1:3);
            Fc_d = X(4:6);
                
        end
        
        %% Calculate the coupling forces between robot, object and human
        function [F_c_r, F_c_h, F_c_o, ddS_o] = calc_Fc(this, dS_r, M_r, D_r, V_r, d_r, dS_h, M_h, D_h, U_h, d_h, dS_o, M_o, is_stiff)
      
            m_o = M_o(1,1);
            w_o = [0; -m_o*this.grav; 0];
            
            dtheta_o = dS_o(3);
            dp_c_r = [d_r*dtheta_o^2; 0];
            dp_c_h = [d_h*dtheta_o^2; 0];
            
            A = [M_r*this.gMat(-d_r) , (~is_stiff)*eye(3) ,   zeros(3); 
                 M_h*this.gMat(-d_h) ,      zeros(3)      ,    eye(3); 
                        M_o          ,   -this.gMat(d_r)'  , -this.gMat(d_h)'];
             
            b = [-D_r*dS_r + V_r - M_r*dp_c_r; 
                 -D_h*dS_h + U_h - M_h*dp_c_h; 
                             w_o            ];
                         
            X = A\b;

            ddS_o = X(1:3);
            F_c_r = X(4:6);
            F_c_h = X(7:9);
            F_c_o = (this.gMat(d_r)'*F_c_r + this.gMat(d_h)'*F_c_h);
            
        end
        
        function S2 = findPoseFromPose(this, S1, d)
            
            theta1 = S1(3);
            p1 = S1(1:2);
            
            theta2 = theta1;
            p2 = p1 + this.rotz(theta2)*[-d; 0];
            
            S2 = [p2; theta2];
            
        end
            
        %% Returns the human's reference trajectory
        function [S_ref, dS_ref, ddS_ref] = getHumanRef(this, t)
            
%             a = -log(this.gp_ref);
%             S_g_err = (1-exp(-a*t/this.tau_ref))*(S_g-S_ref);
%             
%             K_ref = 50;
%             D_ref = 10;
%             ddS_ref = -K_ref*S_g_err - D_ref*dS_ref;

%             T = 0:this.sim_timestep:this.tau_ref;
%             Y_ref = zeros(3,length(T));
%             dY_ref = zeros(3,length(T));
%             ddY_ref = zeros(3,length(T));
%             for i=1:length(T)
%                 for k=1:3
%                     [y_ref, dy_ref, ddy_ref] = this.ref_model{k}.getRef(T(i));
%                     Y_ref(k,i) = y_ref;
%                     dY_ref(k,i) = dy_ref;
%                     ddY_ref(k,i) = ddy_ref;
%                 end
%             end
%             
%             for i=1:3
%                 figure
%                 subplot(3,1,1);
%                 plot(T,Y_ref(i,:));
%                 subplot(3,1,2);
%                 plot(T,dY_ref(i,:));
%                 subplot(3,1,3);
%                 plot(T,ddY_ref(i,:));
%             end
%                 
%             error('stop')
            
            S_ref = zeros(3,1);
            dS_ref = zeros(3,1);
            ddS_ref = zeros(3,1);
            for i=1:3
                [y_ref, dy_ref, ddy_ref] = this.ref_model{i}.getRef(t);
                S_ref(i) = y_ref;
                dS_ref(i) = dy_ref;
                ddS_ref(i) = ddy_ref;
            end
            
            
        end
        
        %% Returns the robot's reference trajectory
        function [ddS_dmp, dx] = getRobotRef(this, x, S_dmp, dS_dmp, Y_c, Z_c)
                
            ddS_dmp = zeros(3,1);
            for i=1:3
                [~, ddy] = this.dmp{i}.getStatesDot(x, S_dmp(i), dS_dmp(i), 0, 0, Y_c(i), Z_c(i));
                ddS_dmp(i) = ddy;
            end
            dx = this.can_clock_ptr.getPhaseDot(x);

            ddS_dmp(3) = 0.0;
            
        end
        
        %% Checks whether the simulation stopping criteria are met
        function stop_sim = stopSimulation(this, t, pos_err, dS_r, dS_h)
            
            stop_sim = false;
                
            if (pos_err <= this.pos_tol_stop ...
                && t>=this.tau_ref && norm(dS_r)<this.vel_tol_stop && norm(dS_h)<this.vel_tol_stop)
                disp('****  Target reached  ****');
                stop_sim = true;
            end

            if (t>=this.max_sim_time_p*this.tau_ref)
                warning('Iteration limit reached. Stopping simulation...\n');
                stop_sim = true;
            end
            
        end
        
        %% For a vector p=(x,y) returns p_bar=(-y,x)
        function x_bar = barVec(this, x)
           
            x_bar(1) = -x(2);
            x_bar(2) = x(1);
            
        end
        
        %% Returns the matrix that transforms a force/torque vector to the force/torque vector at x
        function Gx = gMat(this, x)
           
            Gx = eye(3,3);
            Gx(1:2,3) = this.barVec(x);
            
        end
        
        %% Returns the rotation matrix around z-axis by angle theta
        function Rz = rotz(this, theta)
            
            Rz = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            
        end
        
        %% Updates the online object lifting plot
        function plotOnline(this, S_r, S_h, S_o, F_c_r, F_c_h, pause_dt)
            
            %% ===== Caulculate robot, human and object poses =====
            p_r = S_r(1:2);
            theta_r = S_r(3);
            robAx_x = p_r + this.rotz(theta_r)*[this.pl_h.ax_length; 0];
            robAx_y = p_r + this.rotz(theta_r)*[0; this.pl_h.ax_length];
            
            p_h = S_h(1:2);
            theta_h = S_h(3);
            humAx_x = p_h + this.rotz(theta_h)*[this.pl_h.ax_length; 0];
            humAx_y = p_h + this.rotz(theta_h)*[0; this.pl_h.ax_length];
            
            p_o = S_o(1:2);
            theta_o = S_o(3);
            p_o_left = p_o + this.rotz(theta_o)*[-this.obj.CoM; 0];
            p_o_right = p_o + this.rotz(theta_o)*[this.obj.l-this.obj.CoM; 0];
            objAx_x = p_o + this.rotz(theta_o)*[this.pl_h.ax_length; 0];
            objAx_y = p_o + this.rotz(theta_o)*[0; this.pl_h.ax_length];
            
            %% ================ Object ================
            this.pl_h.objLine.XData = [p_o_left(1) p_o_right(1)];
            this.pl_h.objLine.YData = [p_o_left(2) p_o_right(2)];
            
            this.pl_h.objXax.XData = p_o(1);
            this.pl_h.objXax.YData = p_o(2);
            this.pl_h.objXax.UData = objAx_x(1) - p_o(1);
            this.pl_h.objXax.VData = objAx_x(2) - p_o(2);
            
            this.pl_h.objYax.XData = p_o(1);
            this.pl_h.objYax.YData = p_o(2);
            this.pl_h.objYax.UData = objAx_y(1) - p_o(1);
            this.pl_h.objYax.VData = objAx_y(2) - p_o(2);
            
            this.pl_h.objCenter.XData = p_o(1);
            this.pl_h.objCenter.YData = p_o(2);
            
            this.pl_h.objFrameTxt.Position = [p_o(1) p_o(2) 0] + this.pl_h.frameText_offset;

            
            %% ================ Robot ================           
            this.pl_h.robotXax.XData = p_r(1);
            this.pl_h.robotXax.YData = p_r(2);
            this.pl_h.robotXax.UData = robAx_x(1) - p_r(1);
            this.pl_h.robotXax.VData = robAx_x(2) - p_r(2);
            
            this.pl_h.robotYax.XData = p_r(1);
            this.pl_h.robotYax.YData = p_r(2);
            this.pl_h.robotYax.UData = robAx_y(1) - p_r(1);
            this.pl_h.robotYax.VData = robAx_y(2) - p_r(2);
            
            this.pl_h.robotCenter.XData = p_r(1);
            this.pl_h.robotCenter.YData = p_r(2);
           
            this.pl_h.robotFrameTxt.Position = [p_r(1) p_r(2) 0] + this.pl_h.frameText_offset;
            
            %% ================ Human ================          
            this.pl_h.humanXax.XData = p_h(1);
            this.pl_h.humanXax.YData = p_h(2);
            this.pl_h.humanXax.UData = humAx_x(1) - p_h(1);
            this.pl_h.humanXax.VData = humAx_x(2) - p_h(2);
            
            this.pl_h.humanYax.XData = p_h(1);
            this.pl_h.humanYax.YData = p_h(2);
            this.pl_h.humanYax.UData = humAx_y(1) - p_h(1);
            this.pl_h.humanYax.VData = humAx_y(2) - p_h(2);
            
            this.pl_h.humanCenter.XData = p_h(1);
            this.pl_h.humanCenter.YData = p_h(2);
            
            this.pl_h.humanFrameTxt.Position = [p_h(1) p_h(2) 0] + this.pl_h.frameText_offset;
            
            %% ================ Robot coupling Force arrow ================ 
            this.pl_h.Fc_r_quiv.XData = p_r(1);
            this.pl_h.Fc_r_quiv.YData = p_r(2);
            this.pl_h.Fc_r_quiv.UData = F_c_r(1) * this.pl_h.Fc_scale;
            this.pl_h.Fc_r_quiv.VData = F_c_r(2) * this.pl_h.Fc_scale;
            
            this.pl_h.Fc_r_label.String = sprintf('F_{c,r} = [ %.1f,  %.1f,  %.1f]', F_c_r(1),F_c_r(2),F_c_r(3));
            this.pl_h.Fc_r_arrow_text.Position = [p_r(1)+F_c_r(1)*this.pl_h.Fc_scale, p_r(2)+F_c_r(2)*this.pl_h.Fc_scale, 0];
            
            %% ================ Human coupling Force arrow ================ 
            this.pl_h.Fc_h_quiv.XData = p_h(1);
            this.pl_h.Fc_h_quiv.YData = p_h(2);
            this.pl_h.Fc_h_quiv.UData = F_c_h(1) * this.pl_h.Fc_scale;
            this.pl_h.Fc_h_quiv.VData = F_c_h(2) * this.pl_h.Fc_scale;
            
            this.pl_h.Fc_h_label.String = sprintf('F_{c,h} = [ %.1f,  %.1f,  %.1f]', F_c_h(1),F_c_h(2),F_c_h(3));
            this.pl_h.Fc_h_arrow_text.Position = [p_h(1)+F_c_h(1)*this.pl_h.Fc_scale, p_h(2)+F_c_h(2)*this.pl_h.Fc_scale, 0];
            
            %% ======== draw ========
            drawnow;
            if (pause_dt),pause(pause_dt); end

        end

        %% Logs the simulation's current step data
        function logData(this,t, x, ...
                          S_dmp,dS_dmp,ddS_dmp, P_w, F_err, ...
                          S_r,dS_r,ddS_r, U_r, F_c_r, F_c_r_d, ...
                          S_h,dS_h,ddS_h, U_h, F_c_h, F_c_h_d, ...
                          S_o,dS_o,ddS_o, F_c_o, ...
                          S_ref,dS_ref,ddS_ref)
            
                this.log_data.Time = [this.log_data.Time t];

                this.log_data.x_data = [this.log_data.x_data x];

                this.log_data.S_dmp_data = [this.log_data.S_dmp_data S_dmp];
                this.log_data.dS_dmp_data = [this.log_data.dS_dmp_data dS_dmp];   
                this.log_data.ddS_dmp_data = [this.log_data.ddS_dmp_data ddS_dmp];
                for i=1:3
                    this.log_data.w_dmp_data{i} = [this.log_data.w_dmp_data{i} this.dmp{i}.w];
                    this.log_data.P_w_data{i} = [this.log_data.P_w_data{i} P_w{i}];
                end
                this.log_data.F_err_data = [this.log_data.F_err_data F_err];

                this.log_data.S_r_data = [this.log_data.S_r_data S_r];
                this.log_data.dS_r_data = [this.log_data.dS_r_data dS_r];   
                this.log_data.ddS_r_data = [this.log_data.ddS_r_data ddS_r];
                this.log_data.U_r_data = [this.log_data.U_r_data U_r];
                this.log_data.F_c_r_data = [this.log_data.F_c_r_data F_c_r];
                this.log_data.F_c_r_d_data = [this.log_data.F_c_r_d_data F_c_r_d];
                
                this.log_data.S_h_data = [this.log_data.S_h_data S_h];
                this.log_data.dS_h_data = [this.log_data.dS_h_data dS_h];   
                this.log_data.ddS_h_data = [this.log_data.ddS_h_data ddS_h];
                this.log_data.U_h_data = [this.log_data.U_h_data U_h];
                this.log_data.F_c_h_data = [this.log_data.F_c_h_data F_c_h];
                this.log_data.F_c_h_d_data = [this.log_data.F_c_h_d_data F_c_h_d];
                
                this.log_data.S_o_data = [this.log_data.S_o_data S_o];
                this.log_data.dS_o_data = [this.log_data.dS_o_data dS_o];   
                this.log_data.ddS_o_data = [this.log_data.ddS_o_data ddS_o];
                this.log_data.F_c_o_data = [this.log_data.F_c_o_data F_c_o];

                this.log_data.S_ref_data = [this.log_data.S_ref_data S_ref];
                this.log_data.dS_ref_data = [this.log_data.dS_ref_data dS_ref];   
                this.log_data.ddS_ref_data = [this.log_data.ddS_ref_data ddS_ref];
        end
    end
end
