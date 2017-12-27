function [f, P1] = get_single_sided_Fourier(Fd, Fs)

    L = length(Fd);
    X = Fd;
    Y = fft(X);
    P2 = abs(Y/L);
    
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:floor(L/2))/L;
    
    P1 = P1/max(P1);

end