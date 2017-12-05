function plot_filtering(filter_b, filter_a, Fs, Fmax, Time, Fd, Fd_filt, f, P1, P1_filt)

fontsize = 14;

% filtering process
figure;
% Plot filter
subplot(3,2,[1 2])
[h,w] = freqz(filter_b, filter_a, 1000);
hold on;
MagnF = 20*log10(abs(h));
plot(w/pi*Fs/2, MagnF);
plot([Fmax Fmax], [min(MagnF(find(abs(MagnF)~=Inf))) 20], 'r--', 'LineWidth', 2);
title('Filtering process', 'Interpreter','latex', 'fontsize',fontsize);
xlabel('Frequency ($Hz$)', 'Interpreter','latex', 'fontsize',fontsize);
ylabel('Filter Magnitude (dB)', 'Interpreter','latex', 'fontsize',fontsize);
hold off;
% Plot Initial and filtered signal along with their frequency spectrum
subplot(3,2,3);
plot(Time, Fd);
title('Desired signal $F_d$', 'Interpreter','latex', 'fontsize',fontsize);
ylabel('$F_d$', 'Interpreter','latex', 'fontsize',fontsize);
subplot(3,2,4);
plot(f,P1);
title('Single-Sided Amplitude Spectrum of $F_d(t)$', 'Interpreter','latex', 'fontsize',fontsize);
ylabel('|P1(f)|');
subplot(3,2,5);
plot(Time, Fd_filt);
ylabel('Filtered $F_d$', 'Interpreter','latex', 'fontsize',fontsize);
subplot(3,2,6);
plot(f,P1_filt);
xlabel('f (Hz)');
ylabel('filtered |P1(f)|');


end