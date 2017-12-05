function y = my_sinc(t, tc, Ts)

x = (pi*(t-tc)/Ts);

if (x == 0)
    y = 1;
else
   y = sin(x) / x; 
end


end