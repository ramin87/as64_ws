function plot_F1_F2_Psi(T1, F1, T2, F2, T3, Psi, lineWidth, fontsize, title_, legF1, legF2)
        
scale = 1/max(abs([F1 F2]));

figure;
subplot(2,2,1);
plot(T1,F1, T2,F2, 'LineWidth',lineWidth);
legend({legF1,legF2},'Interpreter','latex','fontsize',fontsize);
title(title_,'Interpreter','latex','fontsize',fontsize);
axis tight;
subplot(2,2,2);
if (T1(end)>T2(end))
    T = T1;
else
    T = T2;
end
[F1_a, F2_a] = makeSignalsEqualLength(T1, F1, T2, F2, T);
plot(T, F1_a-F2_a, 'Color',[1 0 0], 'LineWidth',lineWidth);
legend({[legF1 '-' legF2]},'Interpreter','latex','fontsize',fontsize);
axis tight;
subplot(2,2,[3 4]);
hold on;
plot(T1,F1*scale, T2,F2*scale, 'LineStyle', '--', 'LineWidth',lineWidth);
for k=1:size(Psi,1)
    plot(T3,Psi(k,:));
end
title('Kernel activations','Interpreter','latex','fontsize',fontsize);
axis tight;
hold off;
        
end