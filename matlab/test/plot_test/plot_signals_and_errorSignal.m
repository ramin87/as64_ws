function plot_signals_and_errorSignal(Time1,y1_data, Time2,y2_data, y_label, y2_label, title_, lineWidth)

fontsize = 14;

if (nargin < 5), y_label='$y$'; end
if (nargin < 6), y2_label='$y2$'; end
if (nargin < 7), lineWidth=1.1; end

D = size(y1_data,1);

y1_data_align = cell(D,1);
y2_data_align = cell(D,1);
Time = cell(D,1);
for i=1:D
   [Time{i}, y1_data_align{i}, y2_data_align{i}] = temp_align_signals(Time1, y1_data(i,:), Time2, y2_data(i,:)); 
end

figure;
for i=1:D
    subplot(D,2,1+(i-1)*2);
    plot(Time{i},y1_data_align{i}, Time{i},y2_data_align{i}, 'LineWidth',lineWidth);
    legend({y_label,y2_label},'Interpreter','latex','fontsize',fontsize);
    if (i==1), title(title_,'Interpreter','latex','fontsize',fontsize); end
    axis tight;
    subplot(D,2,2+(i-1)*2);
    plot(Time{i}, y1_data_align{i}-y2_data_align{i},'r','LineWidth',lineWidth);
    legend({'error=DMP-demo'},'Interpreter','latex','fontsize',fontsize);
    axis tight;
end

end
