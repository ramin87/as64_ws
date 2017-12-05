function plot_DMP_train(Time, F, Fd, Psi, s)

    fontsize = 14;
    
    n_sp = 4;
    
    figure;
    
    subplot(n_sp,1,1);
    hold on;
    plot(Time,F, Time, Fd, 'LineWidth',1.2);
    legend({'$F$','$F_d$'}, 'Interpreter','latex', 'fontsize',fontsize);
    hold off;
    
    subplot(n_sp,1,2);
    plot(Time, F-Fd, 'r', 'LineWidth',1.2);
    legend({'$F-F_d$'}, 'Interpreter','latex', 'fontsize',fontsize);
    
    subplot(n_sp,1,3);
    hold on
    Fmax = max(abs([F(:); Fd(:)]));
    plot(Time,F/Fmax, Time, Fd/Fmax, 'LineStyle', '--', 'LineWidth',1.2);
    for i=1:size(Psi,1)
        plot(Time,Psi(i,:));
    end
    hold off;
    
    subplot(n_sp,1,n_sp);
    plot(Time, s);
    legend({'s'}, 'Interpreter','latex', 'fontsize',fontsize);

end