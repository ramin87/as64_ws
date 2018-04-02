function [y, dy, ddy] = getDemoData(y0, g, t, a6, a7)

    dt = diff(t);
    tau = t(end);
    t = t/tau;
    
    a0 = y0;
    a1 = 0;
    a2 = 0;

    A = [1 1 1; 3 4 5; 6 12 20];
    b = [g-a0-a1-a2-a6-a7; 0-a1-2*a2-6*a6-7*a7; 0-2*a2-30*a6-42*a7];
    a345 = A\b;
    a3 = a345(1);
    a4 = a345(2);
    a5 = a345(3);

    %% Create data
    y = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5 + a6*t.^6 + a7*t.^7;

    dy = [0 diff(y)./dt];
    ddy = [0 diff(dy)./dt];
    
end
