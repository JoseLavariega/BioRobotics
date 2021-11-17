function derive_everything() 
name = 'jumping_leg';

% Original Code except for th -> th1 and tau -> tau1, no change to function
% though
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
%syms t y dy ddy th1 dth1 ddth1 tau1 Fy l c1 c2 m1 m2 mh I1 I2 g real
% %added
% syms th2 dth2 ddth2 c3 m3 I3 real
% 
% % Group th1em for later use.
% q   = [y; th1];      % generalized coordinates
% dq  = [dy; dth1];    % first time derivatives
% ddq = [ddy; ddth1];  % second time derivatives
% u   = tau1;          % control forces and moments
% Fc   = Fy;           % constraint forces and moments
% p   = [l; c1; c2; m1; m2; mh; I1; I2; g];  % parameters
% 
% %%% Calculate important vectors and th1eir time derivatives.
% 
% % Define fundamental unit vectors.  The first element should be th1e
% % horizontal (+x cartesian) component, th1e second should be th1e vertical (+y
% % cartesian) component, and th1e th1ird should be right-handed orth1ogonal.
% ihat = [1; 0; 0];
% jhat = [0; 1; 0];
% khat = cross(ihat,jhat);
% 
% % Define oth1er unit vectors for use in defining oth1er vectors.
% er1hat =  cos(th1)*ihat + sin(th1) * jhat;
% er2hat = -cos(th1)*ihat + sin(th1) * jhat;
% 
% % A handy anonymous function for taking first and second time derivatives
% % of vectors using th1e chain rule.  See Lecture 6 for more information. 
% ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 
% 
% % Define vectors to key points.
% rf = y*jhat;
% rcm1 = rf+c1*er1hat;
% rk = rf+l*er1hat;
% rcm2 = rk + c2*er2hat;
% rh = rk + l*er2hat;
% keypoints = [rh rk rf];
% 
% % Take time derivatives of vectors as required for kinetic energy terms.
% drcm1 = ddt(rcm1);
% drcm2 = ddt(rcm2);
% drh   = ddt(rh);
% 
% %%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
% 
% % F2Q calculates th1e contribution of a force to all generalized forces
% % for forces, F is th1e force vector and r is th1e position vector of th1e 
% % point of force application
% F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 
% 
% % M2Q calculates th1e contribution of a moment to all generalized forces
% % M is th1e moment vector and w is th1e angular velocity vector of th1e
% % body on which th1e moment acts
% M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 
% 
% % Define kinetic energies. See Lecture 6 formula for kinetic energy
% % of a rigid body.
% T1 = (1/2)*m1*dot(drcm1, drcm1) + (1/2)* I1 * dth1^2;
% T2 = (1/2)*m2*dot(drcm2, drcm2) + (1/2)* I2 * (-dth1)^2;
% Th = (1/2)*mh*dot(drh, drh);
% 
% % Define potential energies. See Lecture 6 formulas for gravitational 
% % potential energy of rigid bodies and elastic potential energies of
% % energy storage elements.
% V1 = m1*g*dot(rcm1, jhat);
% V2 = m2*g*dot(rcm2, jhat);
% Vh = mh*g*dot(rh, jhat);
% 
% % Define contributions to generalized forces.  See Lecture 6 formulas for
% % contributions to generalized forces.
% QF = F2Q(Fy*jhat,rf);
% Qtau1 = M2Q(-tau1*khat, -dth1*khat);
% 
% % Sum kinetic energy terms, potential energy terms, and generalized force
% % contributions.
% T = T1 + T2 + Th;
% V = V1 + V2 + Vh;
% Q = QF + Qtau1;
% 
% % Calculate rcm, th1e location of th1e center of mass
% rcm = (m1*rcm1 + m2*rcm2 + mh*rh)/(m1+m2+mh);
% 
% % Assemble C, th1e set of constraints
% C = y;  % When y = 0, th1e constraint is satisfied because foot is on th1e ground
% dC= ddt(C);
% 
% %% All th1e work is done!  Just turn th1e crank...
% %%% Derive Energy Function and Equations of Motion
% E = T+V;                                         % total system energy
% L = T-V;                                         % th1e Lagrangian
% eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form th1e dynamics equations
% 
% size(eom)
% 
% %%% Rearrange Equations of Motion. 
% A = jacobian(eom,ddq);
% b = A*ddq - eom;
% 
% 
% %%% Write functions to evaluate dynamics, etc...
% z = sym(zeros(length([q;dq]),1)); % initialize th1e state vector
% z(1:2,1) = q;  
% z(3:4,1) = dq;


%Our attempt, please work 
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy th1 dth1 ddth1 tau1 Fy l c1 c2 m1 m2 mh I1 I2 g real
%added
syms th2 dth2 ddth2 c3 m3 I3 tau2 real

% Group th1em for later use.
q   = [y; th1; th2];      % generalized coordinates
dq  = [dy; dth1; dth2];    % first time derivatives
ddq = [ddy; ddth1; ddth2];  % second time derivatives
u   = [tau1; tau2];          % control forces and moments
Fc   = Fy;           % constraint forces and moments
p   = [l; c1; c2; c3; m1; m2; m3; mh; I1; I2; I3; g];  % parameters

%%% Calculate important vectors and th1eir time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orth1ogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

% Define oth1er unit vectors for use in defining oth1er vectors.
er1hat = -cos(th1)*ihat + sin(th1) * jhat;
er2hat = cos(th1)*ihat + sin(th1) * jhat;
er3hat = -cos(th1+th2)*ihat + sin(th1+th2) *jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using th1e chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
rf = y*jhat;
rcm1 = rf+c1*er1hat;
ra = rf+l*er1hat;
rcm2 = ra + c2*er2hat;
rk = ra + l*er2hat;
rcm3 = rk + c3*er3hat;
rh = rk + l*er3hat;
keypoints = [rh rk ra rf];

% Take time derivatives of vectors as required for kinetic energy terms.
drcm1 = ddt(rcm1);
drcm2 = ddt(rcm2);
drcm3 = ddt(rcm3);
drh   = ddt(rh);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates th1e contribution of a force to all generalized forces
% for forces, F is th1e force vector and r is th1e position vector of th1e 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates th1e contribution of a moment to all generalized forces
% M is th1e moment vector and w is th1e angular velocity vector of th1e
% body on which th1e moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body.
T1 = (1/2)*m1*dot(drcm1, drcm1) + (1/2)* I1 * dth1^2;
T2 = (1/2)*m2*dot(drcm2, drcm2) + (1/2)* I2 * (-dth1)^2;
T3 = (1/2)*m3*dot(drcm3, drcm3) + (1/2)* I3 * (dth1)^2;
Th = (1/2)*mh*dot(drh, drh);

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V1 = m1*g*dot(rcm1, jhat);
V2 = m2*g*dot(rcm2, jhat);
V3 = m3*g*dot(rcm3, jhat);
Vh = mh*g*dot(rh, jhat);

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
QF = F2Q(Fy*jhat,rf);
Qtau1 = M2Q(tau1*khat, dth1*khat);
Qtau2 = M2Q(-tau2*khat, -dth2*khat);

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
T = T1 + T2 + T3 + Th;
V = V1 + V2 + V3 + Vh;
Q = QF + Qtau1 + Qtau2;

% Calculate rcm, the location of the center of mass
rcm = (m1*rcm1 + m2*rcm2 + m3*rcm3 + mh*rh)/(m1+m2+m3+mh);

% Assemble C, th1e set of constraints
C = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC= ddt(C);

%% All th1e work is done!  Just turn th1e crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom);

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;


%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize th1e state vector
z(1:3,1) = q;  
z(4:6,1) = dq;

% Write functions to a separate folder because we don't usually have to see th1em
directory = '../AutoDerived/';
% Write a function to evaluate the energy of th1e system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of th1e system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate th1e b vector of th1e system given th1e current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate th1e X and Y coordinates and speeds of th1e center of mass given th1e current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
