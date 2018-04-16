clc;
close all;
clear;

syms M_r D_r K_r u_r v_r ddy_r dy_r y_r F_cr...
     M_o ...
     ddy dy y ...
     g ...
     real
 
 disp('Robot-Human-Object 1D');
 
 A = [M_r 0; M_o -1];
 X = [ddy; F_cr; F_ch];
 b = [-D_r*dy + v_r; -D_h*dy + u_h; -M_o*g];
 
 X_sol = collect(simplify(A\b));
 
 disp('Coupling forces:');
 F_cr = X_sol(2)
 F_ch = X_sol(3)