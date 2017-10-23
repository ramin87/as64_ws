% Calculate DMP from demonstration
%
% This code uses force data from an actual experiment, that involves
% human-robot collaboration on a horizontal plane. The user guided the robot
% along a path multiple times. The recorder external forces are used here as
% an input for an admittance controller. 
%
% At the first run the algorithm records the Control Points. 
%
 
clc
clear all
% r = lwr_create(7);

% Load Path 
Ts=0.01; %in seconds
time = Ts:Ts:7;
time0 = time - time(1); %start clock from 0

cp_time_step=0.1/Ts; %every 10 Ts
cp_pos_step=0.01; %max 0.01m
cp1_radius = 0.005; %radius in mm

% Load the data file from the LWR robot 
load('demonstration.mat','P');


cp=[];
hf2=figure(2); clf; daspect([1 1 1]); hold on; xlabel('X'); ylabel('Y'); axis([-0.7 -0.4 -0.28 0.02 ])
slow_down_sim=0;
left_cp1_radius=false;
finished_loop=false;
prev_cp_i=2;

p = P;%(round(time/Ts),:);

figure(2); clf
plot(p(:,1),p(:,2))

%% DMPs
%Dynamic system
taf=1; %m equivalent
ay=1; %c equivalent
by=ay/4; %k/c equivalent

cs_type = 'lin'; %type of canonical system

% p = cp; %use control point to train DMP

% f_d = imitate_path(p,Ts,ay,by);
% w = gen_weights(f_d, Ts, length(time))

%generate centers of Basis Functions
N=50;
ax=1;
first = exp(-ax*time0(end));
last = 1. - first;
des_c = linspace(first, last, N); %
% c = -log(des_c); % x = exp(-c), solving for c. Make phasing system for time independence
c = ([1:N]+0.1-1 )/(N-1);
h=[1:N].^1*200./c;

% h = [1:N].^2./c;
% c = logspace(-0.1,-5,N);
% c=linspace(0.01,0.99,N);
% set variance of Gaussian basis functions
% trial and error to find this spacing
% h = ones(1,N) * 100;% N^1.5 ./ c;
% h = 2*N./c;


% plot_BF(h,c,Ts,time0(end),cs_type,ax)

%
%imitate path
if (size(p,1) < size(p,2))
    p = p'; %reshape to columns
end


y0 = p(1,:);
y_g = p(end,:);

pd = [zeros(1, size(p,2)); diff(p)/Ts];
pdd = [zeros(1, size(p,2)); diff(pd)/Ts];

f_d = pdd - ay*(by * (y_g - p) - pd);

% gen_weights 
[x_track, xd_track] = rollout(Ts, length(p),cs_type,ax);
for t=1:length(p)
    for i=1:N
       psi_track(t,i) = exp(-h(i)*(x_track(t)-c(i))^2);
    end
end

%Train. Calculate BF weights using weighted linear regression
w = zeros(size(p,2), N); %preallocate
tic
k = y_g - y0;     % spatial scaling term
for d=1:size(p,2)
    for i=1:N
        numer = x_track' * diag(psi_track(:,i)) * f_d(:,d);
        denom = x_track' * diag(psi_track(:,i)) * x_track + 1e-10;
        w(d, i) = numer / (k(d) * denom);
    end
end
toc
% w(isnan(w))=0; %replace NaN with zeros remedy

%Reinitialize dmp
y(1,:)=y0; yd = zeros(1,size(p,2));

%phase
x = x_track;
xd=xd_track;

%delayed goal (to avoid high acceleration at start)
ag=-5;
y_gd(1,:) = y(1,:);

%gating
av=2;
v_max=1.000010;
v(1)=1; %vd(1)=0;

%Run trained dmp
for i=2:length(p)
    
    f(i,:) = attractor(N,h,c,w,y(1,:),y_g,x(i-1));
    ydd(i,:) = ay*(by*(y_g-y(i-1,:)) - yd(i-1,:) + f(i,:) ) / taf;
    
    %phase system
    xd(i) = -ax*x(i-1) / taf; %exponential
%     x(i)=x(i-1)+xd(i)*Ts;
    %delayd goal system to avoid large acceleration at the beginning
%     y_gdd(i,:) = -ag*(y_g - y_gd(i-1,:));     
    
    %sigmoid gating
    vd(i) = -av*v(i-1)*( 1 - v(i-1)/v_max );
    
    %integrate
%     y_gd(i,:)=y_gd(i-1,:)+y_gdd(i,:)*Ts;
    v(i)=v(i-1)+vd(i)*Ts;
    yd(i,:) = yd(i-1,:)+ydd(i,:)*Ts;
    y(i,:) = y(i-1,:)+yd(i,:)*Ts;
    
end

%
figure(2); clf
plot3(p(:,1),p(:,2),p(:,3),'LineWidth',2)
hold on
plot3(y(:,1),y(:,2),y(:,3),'r','LineWidth',2)
% plot(cp(:,1),cp(:,2),'*b')
legend('Recorded','DMP')
title(['Discrete DMP (#BFs: ' num2str(N) ')'])

%%
% %Arrow plot of forces
% plotstep=5;
% scl=50;
% figure(3); clf
% plot(p(:,1),p(:,2),'LineWidth',2)
% hold on
% plot(y(:,1),y(:,2),'r','LineWidth',2)
% for kv = 1:plotstep:length(p)
% %     plot([y(kv,1) y(kv,1)+f(kv,1)/scl],[y(kv,2) y(kv,2)+f(kv,2)/scl])
%     quiver(y(kv,1),y(kv,2), f(kv,1)/scl, f(kv,2)/scl)
% end
% % plot(cp(:,1),cp(:,2),'*b')
% legend('Recorded','DMP')
% % title(['Discrete DMP (#BFs: ' num2str(N) ')'])
