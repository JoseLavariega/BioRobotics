function leg_derive_everything()
clear
name = 'bounding_leg';

%Variables for time and generalized coordinates
syms t real
syms th1 dth1 ddth1 real
syms th2 dth2 ddth2 real
syms th3 dth3 ddth3 th3_0 real
syms x dx ddx y dy ddy real
syms m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 g kappa real
syms l_OA l_OB l_AC l_DE l_EF real 
syms tau1 tau2 tau3 Fx Fy real
syms Ir N real

%Group them
q   = [th1  ; th2 ; th3; x; y];      % generalized coordinates
dq  = [dth1 ; dth2 ; dth3; dx; dy];    % first time derivatives
ddq = [ddth1;ddth2; ddth3; ddx; ddy];  % second time derivatives
u   = [tau1 ; tau2; tau3];              % controls
F   = [Fx ; Fy];                       % Forces

p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N... 
       l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5...
       l_OA l_OB l_AC l_DE l_EF...
       g kappa th3_0]';        % parameters
   
   % think about this some more
   

