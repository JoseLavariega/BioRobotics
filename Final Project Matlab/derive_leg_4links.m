clear
name = 'leg';

%Define variables for time, generalized coordinates.
syms t real
syms th1 dth1 ddth1 real
syms th2 dth2 ddth2 real
syms th3 dth3 ddth3 real
syms m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 g real
syms l_OA l_OB l_AC l_DE l_EF real 
syms tau1 tau2 Fx Fy real
syms Ir N real

%Group
q  = [th1; th2; th3]; % gen coords
dq = [dth1; dth2; dth3]; % 1st time derivative
ddq = [ddth1; ddth2; ddth3]; % 2nd time derivative
u = [tau1; tau2]; % controls
F = [Fx; Fy];

p = [m1 m2 m3 m4 I1 I2 I3 I4 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';        % parameters

% Generate Vectors and Derivativess
ihat = [0; -1; 0];
jhat = [1; 0; 0];

xhat = [1; 0; 0];
yhat = [0; 1; 0];

khat = cross(ihat,jhat);
e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;