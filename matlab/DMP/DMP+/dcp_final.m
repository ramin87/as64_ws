function [varargout] = dcp(action,varargin)
    global dcps;

    if nargin < 2,
      error('Incorrect call to dcp');
    end
    ID = varargin{1};

  switch lower(action),
% .........................................................................
  case 'clear'
    if exist('dcps', 'var')
      if length(dcps) >= ID,
        dcps(ID) = [];
      end
    end
  case 'init'
    n_rfs            = varargin{2};
    % the time constants chosen for critical damping
    dcps(ID).alpha_z = 25;
    dcps(ID).beta_z  = dcps(ID).alpha_z/4;
    dcps(ID).alpha_g = dcps(ID).alpha_z/2;
    dcps(ID).alpha_x = dcps(ID).alpha_z/3;
    % initialize the state variables
    dcps(ID).y       = 0; 
    dcps(ID).x       = 0;
    dcps(ID).yd      = 0; 
    dcps(ID).xd      = 0;
    dcps(ID).ydd     = 0; 
    % the current start state of the primitive
    %truncation for kernel function
    dcps(ID).sp = exp(-4.5); %4.5=3^2/2, 3 std away
    
    t = (0:1/(n_rfs-1):1)'; % linear decay
    % the local models, spaced on a grid in time by applying the
    dcps(ID).c = t;
    dcps(ID).psi     = zeros(n_rfs,1);
    dcps(ID).w       = zeros(n_rfs,1);
    dcps(ID).b       = zeros(n_rfs,1);
    decay_to=4;
    dcps(ID).D       = (n_rfs-1)^2*decay_to*ones(n_rfs, 1);
    dcps(ID).goal=0;
  case 'reset_state'
    % initialize the state variables
    dcps(ID).y       = dcps(ID).state0(1); 
    dcps(ID).x       = 1;
    dcps(ID).yd      = dcps(ID).state0(2); 
    dcps(ID).xd      = 0;
    dcps(ID).ydd     = dcps(ID).state0(3); 
    dcps(ID).goal = 0;
  case 'set_goal'
    dcps(ID).goal       = varargin{2};
    dcps(ID).x     = 1;
  case 'run'
    if varargin{2}==0,
        tau=1;
    else
        tau= dcps(ID).tau/varargin{2}; % tau is relative to 0.5 seconds nominal movement time
    end
    dt               = varargin{3}*tau;
    dt_x = dcps(ID).dt_x*tau;
    % the weighted sum of the locally weighted regression models
    dcps(ID).psi = exp(-0.5*((dcps(ID).x-dcps(ID).c).^2).*dcps(ID).D);
    dcps(ID).psi(dcps(ID).psi<dcps(ID).sp)=0;
    in = dcps(ID).x;
    f = sum((in*dcps(ID).w+dcps(ID).b).*dcps(ID).psi)/sum(dcps(ID).psi+1e-8);
    dcps(ID).x  = dt_x+dcps(ID).x;
    
    dcps(ID).ydd = (dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(ID).goal-dcps(ID).y)-dcps(ID).yd)+f);
    dcps(ID).y  = dcps(ID).yd*dt+dcps(ID).y;
    dcps(ID).yd = dcps(ID).yd+dcps(ID).ydd*dt;
        
    varargout(1) = {dcps(ID).y};
    varargout(2) = {dcps(ID).yd};
    varargout(3) = {dcps(ID).ydd};
    varargout(4) = {f};
    case 'get_data'
        X=(1:dcps(ID).dt_x:0)';
        PSI=exp(-0.5*((X*ones(1,length(dcps(ID).c))-ones(length(X),1)*dcps(ID).c').^2).*(ones(length(X),1)*dcps(ID).D'));
        F=sum((X*dcps(ID).w'+ones(length(X), 1)*dcps(ID).b').*PSI,2)./(sum(PSI,2)+1e-8);
        
        varargout(1)={X};
        varargout(2)={PSI};
        varargout(3)={F};
    case 'shift_left'
        r=varargin{2};
        dcps(ID).w=dcps(ID).w/r;
        dcps(ID).b=dcps(ID).b+(r-1)*dcps(ID).w;
        dcps(ID).D=dcps(ID).D/r^2;
        dcps(ID).c=dcps(ID).c*r+1-r;
        dcps(ID).dt_x=dcps(ID).dt_x*r;
    case 'shift_right'
        r=varargin{2};
        dcps(ID).w=dcps(ID).w/r;
        dcps(ID).b=dcps(ID).b;
        dcps(ID).D=dcps(ID).D/r^2;
        dcps(ID).c=dcps(ID).c*r;
        dcps(ID).dt_x=dcps(ID).dt_x*r;
    case 'join'
        id2=varargin{2};
        id_new=varargin{3};
        if nargin > 4,
            shift=varargin{4};
            dt=varargin{5};
        else
            shift=dcps(ID).goal-dcps(id2).state0(1);
        end
        
        dcp('init', id_new, length(dcps(ID).c)+length(dcps(id2).c)-1);
        dcps(id_new).tau=dcps(ID).tau+dcps(id2).tau;
        if nargin>4,
            dcps(id_new).tau=dcps(id_new).tau+dt;
        end
        if nargin>4,
            dcps(id_new).dt_x=1/(1/dcps(ID).dt_x+1/dcps(id2).dt_x-1);
        else
            dcps(id_new).dt_x=1/(1/dcps(ID).dt_x+1/dcps(id2).dt_x);
        end
        dcps(id_new).goal=dcps(id2).goal+shift;
        dcps(id_new).state0=dcps(ID).state0;
        
        
        r1=dcps(ID).tau/dcps(id_new).tau;
        r2=dcps(id2).tau/dcps(id_new).tau;
        dcp('shift_left', ID, r1);
        dcp('shift_right', id2, r2);
        
        [X1, PSI1, F1]=dcp('get_data', ID);
        [X2, PSI2, F2]=dcp('get_data', id2);
        
        
        
        t1=dcps(ID).c-3*sqrt(1./dcps(ID).D);
        inds=find(t1<1-r1);
        
        ct1=dcps(ID).c(inds(2:end)); % drop the first kernel as it overlaps with the second dmp
        Dt1=dcps(ID).D(inds(2:end));
        
        t2=dcps(id2).c+3*sqrt(1./dcps(id2).D);
        inds=find(t2>r2);
        ct2=dcps(id2).c(inds);
        Dt2=dcps(id2).D(inds);
        
        ct=[ct2; ct1]; % arranged in ascending order
        Dt=[Dt2; Dt1];
        
        ind1=(X1<=ct(end)+3*sqrt(1./Dt(end)) & X1>1-r1);
        x1=X1(ind1);
        ind2=(X2>=ct(1)-3*sqrt(1./Dt(1)) & X2<=r2);
        x2=X2(ind2);
        
        goal_change=dcps(ID).alpha_z*dcps(ID).beta_z*(dcps(ID).goal-dcps(id_new).goal);
        
        if nargin>4,
            X=[x1; 1-r1; x2];
            yd=(dcps(id2).state0(1)-dcps(ID).goal)/dt;
            ydd=(dcps(id2).state0(2)-yd)/dt;
            ft=ydd-dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(id_new).goal-dcps(ID).goal)-yd);
            Ft=[F1(ind1)+goal_change; ft; F2(ind2)];
        else
            X=[x1; x2];
            Ft=[F1(ind1)+goal_change; F2(ind2)];
        end
%         varargout(1)={[F1(1:end-1)+goal_change; F2(1:end)]};
        
        [w, b]=learn(X, Ft, ct, Dt, dcps(ID).sp);
        dcps(id_new).w=[dcps(id2).w(1:end-length(ct2)); w; dcps(ID).w(length(ct1)+2:end)];
        dcps(id_new).D=[dcps(id2).D(1:end-length(ct2)); Dt; dcps(ID).D(length(ct1)+2:end)];
        dcps(id_new).c=[dcps(id2).c(1:end-length(ct2)); ct; dcps(ID).c(length(ct1)+2:end)];
        dcps(id_new).b=[dcps(id2).b(1:end-length(ct2)); b; dcps(ID).b(length(ct1)+2:end)+goal_change];
        
      case 'update'
          id_new=varargin{2};
          s_ind=varargin{3};
          e_ind=varargin{4};
          T=varargin{5}; %include two extra points behind and one point in front from original traj.
          dt=varargin{6}; % assume dt is consistent with internal dt
          n_k=varargin{7};

          Td=diff(T)/dt;
          Tdd=diff(Td)/dt;
          
           Ft = (Tdd-dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(ID).goal-T(1:end-2))-Td(1:end-1)));
          %this is Ft from s_ind-1 through e_ind-1
          
          dcp('init', id_new, length(dcps(ID).c)+n_k);
          seg_diff=length(T)-3-(e_ind-s_ind-1);   
          dcps(id_new).tau=dcps(ID).tau+dt*(seg_diff+1);
          r1=dcps(ID).tau/dcps(id_new).tau;
          
          dcps(id_new).goal=dcps(ID).goal;
          dcps(id_new).dt_x=1/(1/dcps(ID).dt_x-seg_diff); %dt_x is negative
          
          x_s=1+dcps(ID).dt_x*(s_ind-1);
          x_e=1+dcps(ID).dt_x*(e_ind-1);
          
          [X1, PSI1, F1]=dcp('get_data', ID);
          
          t1=dcps(ID).c-3*sqrt(1./dcps(ID).D);
          c_max=find(t1<x_s, 1, 'last' );
          t2=dcps(ID).c+3*sqrt(1./dcps(ID).D);
          c_min=find(t2>x_e, 1);
          
          
          ind1=find((X1<=dcps(ID).c(c_max)+3*sqrt(1/dcps(ID).D(c_max))), 1);
          x1=X1(ind1);
          ind2=find((X1>=dcps(ID).c(c_min)-3*sqrt(1/dcps(ID).D(c_min))), 1, 'last');
          x2=X1(ind2);
          
          c_s=1-r1+r1*dcps(ID).c(c_max);
          c_e=r1*dcps(ID).c(c_min);
          c=(c_e:(c_s-c_e)/(n_k+1):c_s)'; % ascending order
          D=[dcps(ID).D(c_min)*r1; 1./(diff(c).^2*0.25)];
          
          x1=1-r1+r1*x1;
          x2=r1*x2;
          X=(x1:dcps(id_new).dt_x:x2)';
          
          F=[F1(ind1:s_ind-1); Ft; F1(e_ind:ind2)]; 
          
          
          dcps(id_new).state0=dcps(ID).state0; % to-do if update happens at the start
          
          [w, b]=learn(X, F, c, D, dcps(ID).sp);
         
          
          %shift right, mod, shift_left
          dcps(id_new).w=[dcps(ID).w(1:c_min-1)/r1; w; dcps(ID).w(c_max+1:end)/r1];
          dcps(id_new).b=[dcps(ID).b(1:c_min-1); b; dcps(ID).b(c_max+1:end)+(1-1/r1)*dcps(ID).w(c_max+1:end)];
          dcps(id_new).c=[dcps(ID).c(1:c_min-1)*r1; c; dcps(ID).c(c_max+1:end)*r1+1-r1];
          dcps(id_new).D=[dcps(ID).D(1:c_min-1)/r1^2; D; dcps(ID).D(c_max+1:end)/r1^2]; 
          
%           disp(dcps(id_new).D);
       
  case 'batch_fit'
    dt               = varargin{3};
    T                = varargin{4};
    dcps(ID).tau     =dt*(length(T)-1);
    
    if (nargin>5)
        goal=varargin{5};
    else
        goal=T(end);
    end
    
    dcps(ID).goal=goal;
    
    if (nargin>6)
        X=varargin{6};
    else
        X=(1:-1/(length(T)-1):0)';
    end
    dcps(ID).dt_x=X(2)-X(1); % assume uniform sampling
    
    
    if (nargin > 7) 
      Td               = varargin{7};
    else
      Td               = diff(T)./dt;
      Td =[Td;Td(end)];
    end
    if (nargin > 8) 
      Tdd              = varargin{8};
    else
      Tdd              = [diff(Td)./dt;0];
    end
    Ft  = (Tdd-dcps(ID).alpha_z*(dcps(ID).beta_z*(goal-T)-Td));
    
    % compute the weights for each local model along the trajectory
    PSI = exp(-0.5*((X*ones(1,length(dcps(ID).c))-ones(length(T),1)*dcps(ID).c').^2).*(ones(length(T),1)*dcps(ID).D'));
    PSI(PSI<dcps(ID).sp)=0;
    
    X2=X.^2;
    xy=X.*Ft;
    for j=1:length(dcps(ID).c)
        sx=PSI(:,j)'*X;
        sx2=PSI(:,j)'*X2;
        sw=sum(PSI(:,j));
        det=sx2*sw-sx^2+1e-8;
        sxy=PSI(:,j)'*xy;
        sy=PSI(:,j)'*Ft;
        dcps(ID).w(j)=[sw -sx]/det*[sxy; sy];
        dcps(ID).b(j)=[-sx sx2]/det*[sxy; sy];
    end
    
    F=  sum((X*dcps(ID).w'+ones(length(X), 1)*dcps(ID).b').*PSI,2)./(sum(PSI,2)+1e-8);
    yd     = Td(1);
    ydd    = Tdd(1);  
    y     = T(1);
    Y     = zeros(size(T));
    Yd    = zeros(size(T));
    Ydd   = zeros(size(T));
    dcps(ID).state0=[y yd ydd];
    
    for i=1:length(T),
      
      Ydd(i) = ydd;
      Yd(i)  = yd;
      Y(i)   = y;
      
      ydd=(dcps(ID).alpha_z*(dcps(ID).beta_z*(goal-y)-yd)+F(i));
      y    = yd*dt+y;
      yd   = ydd*dt+yd;
    end
    
    varargout(1) = {Y};
    varargout(2) = {Yd};
    varargout(3) = {Ydd};
    varargout(4) = {F};
    varargout(5) = {Ft};
  case 'batch_fit_g'
    dt               = varargin{3};
    T                = varargin{4};
    dcps(ID).tau     =dt*(length(T)-1);
    
    if (nargin>5)
        goal=varargin{5};
    else
        goal=T(end);
    end
    
    dcps(ID).goal=goal;
    
    if (nargin>6)
        X=varargin{6};
    else
        X=(1:-1/(length(T)-1):0)';
    end
    dcps(ID).dt_x=X(2)-X(1); % assume uniform sampling
    
    
    if (nargin > 7) 
      Td               = varargin{7};
    else
      Td               = [diff(T)./dt;0];
    end
    if (nargin > 8) 
      Tdd              = varargin{8};
    else
      Tdd              = [diff(Td)./dt;0];
    end
    Ft  = (Tdd-dcps(ID).alpha_z*(dcps(ID).beta_z*(goal-T)-Td));
    
    % compute the weights for each local model along the trajectory
    PSI = exp(-0.5*((X*ones(1,length(dcps(ID).c))-ones(length(T),1)*dcps(ID).c').^2).*(ones(length(T),1)*dcps(ID).D'));
    t1=sum(PSI, 2)*ones(1, length(dcps(ID).c));
    
    PSI(PSI<dcps(ID).sp)=0;
    
    PSI_X=PSI.*(X*ones(1,length(dcps(ID).c)))./t1;
    weight=[PSI_X PSI./t1];
    t_a=weight'*weight;
    t_b=weight'*Ft;
    t_ans=t_a\t_b;
    
    dcps(ID).w=t_ans(1:length(dcps(ID).c));
    dcps(ID).b=t_ans(length(dcps(ID).c)+1:2*length(dcps(ID).c));
    F=  sum((X*dcps(ID).w'+ones(length(X), 1)*dcps(ID).b').*PSI,2)./(sum(PSI,2)+1e-8);
    yd     = Td(1);
    ydd    = Tdd(1);  
    y     = T(1);
    Y     = zeros(size(T));
    Yd    = zeros(size(T));
    Ydd   = zeros(size(T));
    dcps(ID).state0=[y yd ydd];
    
    for i=1:length(T),
      
      Ydd(i) = ydd;
      Yd(i)  = yd;
      Y(i)   = y;
      
      ydd=(dcps(ID).alpha_z*(dcps(ID).beta_z*(goal-y)-yd)+F(i));
      y    = yd*dt+y;
      yd   = ydd*dt+yd;
    end
    
    varargout(1) = {Y};
    varargout(2) = {Yd};
    varargout(3) = {Ydd};
    varargout(4) = {F};
    varargout(5) = {Ft};
  end
end

function [w, b]=learn(X, Ft, c, D, sp)
    w=zeros(size(c));
    b=zeros(size(c));
    PSI = exp(-0.5*((X*ones(1,length(c))-ones(length(X),1)*c').^2).*(ones(length(X),1)*D'));
    PSI(PSI<sp)=0;
    
    X2=X.^2;
    xy=X.*Ft;
    for j=1:length(c)
        sx=PSI(:,j)'*X;
        sx2=PSI(:,j)'*X2;
        sw=sum(PSI(:,j));
        det=sx2*sw-sx^2+1e-8;
        sxy=PSI(:,j)'*xy;
        sy=PSI(:,j)'*Ft;
        w(j)=[sw -sx]/det*[sxy; sy];
        b(j)=[-sx sx2]/det*[sxy; sy];
    end
end