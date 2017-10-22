clc;
close all;
clear;

x_end = 0.002;

syms a_u;

f = (1 + a_u/2)*exp(-a_u/2) - x_end;

a = vpasolve(f,a_u)

a_u = a;

err = subs(f)



a1 = -500;
a3 = 500;
a2 = (a1+a3)/2;
a2_prev = a2;

g = @(x) (1 + x/2)*exp(-x/2) - x_end;
dg = @(x) -(x/2).^2*exp(-x/2);

tol_stop = 1e-12;

iters = 0;

while (true)
   
   a2_prev = a2;
   
   a2 = (a1+a3)/2;
    
   g1 = g(a1);
   
   g2 = g(a2);
   
   g3 = g(a3);
   
   if (g1*g2<0)
       a3 = a2;
   elseif (g2*g3<0)
       a1 = a2;
   else
       error('No feasible solution exists in this interval');
   end
    
   a2 = (a1+a3)/2;
   
   iters = iters+1;
   
   if (abs(a2-a2_prev) < tol_stop), break; end
end

a_u = a2
err = g(a_u)

iters


iters = 0;
a0 = log(x_end);
a1_prev = a0;

while (true)
   
   a1 = a0 - g(a0)/dg(a0);
   
   iters = iters+1;
   
   if (abs(a1-a1_prev) < tol_stop), break; end
   
   a1_prev = a1;
   a0 = a1;
end

a_u = a1
err = g(a_u)

iters



