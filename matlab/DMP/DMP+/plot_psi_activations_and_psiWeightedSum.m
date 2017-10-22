function plot_psi_activations_and_psiWeightedSum(x_data,Psi_data, f_data, c, w)

D = length(w);

lineWidth = 2;
for d=1:D
    N_kernels = size(Psi_data{d},1);
    figure;
    subplot(2,1,1);
    hold on;
    for i=1:N_kernels
        plot((x_data),(Psi_data{d}(i,:)));
        axis tight;
    end
    title('Psi activations');
    set(gca,'Xdir','reverse');
    hold off;
    subplot(2,1,2);
    hold on;
    plot((x_data),(f_data{d}),'LineWidth',lineWidth,'Color',[0 1 0]);
    bar(c{d},w{d});
    title('Weighted summation');
    set(gca,'Xdir','reverse');
    xlim([0 1]);
    axis tight;
    hold off;
end

end