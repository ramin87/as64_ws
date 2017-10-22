clc;
close all;
clear;

global a_z b_z a_py tau ...
    MAX_ITERS TOL_STOP dt ...
    Kd Dd Fdist ...
    lineWidth fontSize

a_z = 40;
b_z = a_z/4;
a_py = b_z;
tau = 1;

MAX_ITERS = 6000;
TOL_STOP = 1e-3;
dt = 0.002;

Kd = 150;
Dd = 4;
Fdist = 0;

lineWidth = 1.2;
fontSize = 12;

q0 = [0.4 0.2 0.6 0.1]';%rand(4,1);
q0 = q0 / norm(q0);

q_T = [0.2 0.6 0.1 0.8]';%rand(4,1);
q_T = q_T / norm(q_T);

t = 0;
q = q0;
q_robot = q0;
iters = 0;
dq = zeros(4,1);
dq_robot = zeros(4,1);

eta = zeros(3,1);
deta = zeros(3,1); 

Time = [];
q_data = [];
q_robot_data = [];
eta_data = [];
Fdist_data = [];

v_scale = 1 / tau;

while (true)
    
    Time = [Time t];
    q_data = [q_data q];
    q_robot_data = [q_robot_data q_robot];
    eta_data = [eta_data eta];
    Fdist_data = [Fdist_data Fdist];

    e_q = quatLog(quatProd(q_T,quatInv(q)));
    deta = a_z*(b_z*e_q - eta) / tau;
    e_q_robot = quatLog(quatProd(q_robot,quatInv(q)));
    v_rot = (eta + a_py*e_q_robot) / tau;
    dq = 0.5*quatProd([0; v_rot],q);
    
    v_rot_robot = v_rot - (Kd/Dd)*quatLog(quatProd(q_robot,quatInv(q))) + Fdist/Dd;
    
    %v_rot = 2*quatProd(dq,quatInv(q));
    %v_rot = v_rot(2:4);
    
    if (imag(q) ~= 0)
        iters
        t
        e_q
        e_q_robot
        q
        v_rot
        q_robot
        v_rot_robot
        eta
        deta
        pause
    end

    err = quatDist(q_robot,q_T);
    if (err<TOL_STOP || iters>MAX_ITERS), break; end
    
    %err
    
    t = t + dt;
    iters = iters + 1;
    
    %q = q + dq*dt;
    %q = q/norm(q);
    q = quatProd(quatExp(v_rot*dt),q);
    
    q_robot = quatProd(quatExp(v_rot_robot*dt),q_robot);
    
    eta = eta + deta*dt;    
    
    Fdist = Fdist_fun(t);
end

iters

k = 5;
figure;
subplot(k,1,1);
hold on;
plot(Time,q_data(1,:),'Linestyle','-','Color',[1 0 0],'LineWidth',lineWidth);
plot(Time,q_robot_data(1,:),'Linestyle','-','Color',[0.5 0 0],'LineWidth',lineWidth);
plot(Time(end),q_T(1),'r*','Markersize',10);
title('Quaternion evolution','Interpreter','latex','fontsize',fontSize);
legend({'$\eta_r$','$\eta$','$\eta_T$'},'Interpreter','latex','fontsize',fontSize);
hold off;
subplot(k,1,2);
hold on;
plot(Time,q_data(2,:),'Linestyle','-','Color',[0 1 0],'LineWidth',lineWidth);
plot(Time,q_robot_data(2,:),'Linestyle','-','Color',[0 0.5 0],'LineWidth',lineWidth);
plot(Time(end),q_T(2),'g*','Markersize',10);
legend({'$\epsilon_{1r}$','$\epsilon_1$','$\epsilon_{1T}$'},'Interpreter','latex','fontsize',fontSize);
hold off;
subplot(k,1,3);
hold on;
plot(Time,q_data(3,:),'Linestyle','-','Color',[0 0 1],'LineWidth',lineWidth);
plot(Time,q_robot_data(3,:),'Linestyle','-','Color',[0 0 0.5],'LineWidth',lineWidth);
plot(Time(end),q_T(3),'b*','Markersize',10);
legend({'$\epsilon_{2r}$','$\epsilon_{2T}$'},'Interpreter','latex','fontsize',fontSize);
hold off;
subplot(k,1,4);
hold on;
plot(Time,q_data(4,:),'Linestyle','-','Color',[1 0 1],'LineWidth',lineWidth);
plot(Time,q_robot_data(4,:),'Linestyle','-','Color',[0.5 0 0.5],'LineWidth',lineWidth);
plot(Time(end),q_T(4),'m*','Markersize',10);
legend({'$\epsilon_{3r}$','$\epsilon_3$','$\epsilon_{3T}$'},'Interpreter','latex','fontsize',fontSize);
hold off;
subplot(k,1,5);
plot(Time,Fdist_data,'LineWidth',lineWidth);
legend({'Disturbance force'},'Interpreter','latex','fontsize',fontSize);
xlabel('time [$s$]','Interpreter','latex','fontsize',fontSize);

