function plot_signals_and_errorSignal(Time,y_data, Time2,y2_data, y_label, y2_label, title_, lineWidth)

fontsize = 14;

if (nargin < 5), y_label='$y$'; end
if (nargin < 6), y2_label='$y2$'; end
if (nargin < 7), lineWidth=1.1; end

D = size(y_data,1);

dtw_win = floor(max([size(y_data,2), size(y2_data,2)])/3);
[dtw_dist, ind_y, ind_yd] = dtw(y_data, y2_data, dtw_win);
y_data_dtw = y_data(:,ind_y);
y2_data_dtw = y2_data(:,ind_yd);
% sum_C = sum(C)
% dtw_dist
%mean_error = sum(C)/length(C)

figure;
for i=1:D
    subplot(D,2,1+(i-1)*2);
    plot(Time,y_data(i,:),Time2,y2_data(i,:),'LineWidth',lineWidth);
    legend({y_label,y2_label},'Interpreter','latex','fontsize',fontsize);
    if (i==1), title(title_,'Interpreter','latex','fontsize',fontsize); end
    axis tight;
    subplot(D,2,2+(i-1)*2);
%     yd = [yd_data(:,i); repmat(yd_data(end,i), length(y_data(:,i))- length(yd_data(:,i)),1)];
%     plot(Time,y_data(:,i)-yd,'r');
    plot(y_data_dtw(i,:)-y2_data_dtw(i,:),'r','LineWidth',lineWidth);
    legend({'error=DMP-demo'},'Interpreter','latex','fontsize',fontsize);
    axis tight;
end

end
