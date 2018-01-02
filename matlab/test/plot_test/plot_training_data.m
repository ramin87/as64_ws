function plot_training_data(Time_demo, yd_data, dyd_data, ddyd_data)

D = size(yd_data,1);

lineWidth = 1.3;
fontsize = 14;

figure;
for i=1:D
    subplot(3,D,1+(i-1));
    plot(Time_demo,yd_data(i,:),'b','LineWidth',lineWidth);
    title(['Dim' num2str(i)],'Interpreter','latex','fontsize',fontsize);
    if (i==1), ylabel('$[m]$','Interpreter','latex','fontsize',fontsize); end
    axis tight;
    
    subplot(3,D,1+(i-1)+D);
    plot(Time_demo,dyd_data(i,:),'g','LineWidth',lineWidth);
    if (i==1), ylabel('$[m/s]$','Interpreter','latex','fontsize',fontsize); end
    axis tight;
    
    subplot(3,D,1+(i-1)+2*D);
    plot(Time_demo,ddyd_data(i,:),'r','LineWidth',lineWidth);
    if (i==1), ylabel('$[m/s^2]$','Interpreter','latex','fontsize',fontsize); end
    xlabel('time [$s$]','Interpreter','latex','fontsize',fontsize);
    axis tight;
end


end

