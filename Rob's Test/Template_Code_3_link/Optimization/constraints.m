function [cineq ceq] = constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

tf = x(1);
ctrl.tf = x(2);
ctrl.T = x(3:end);

[t,z,u_temp, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]);
COM = COM_jumping_leg(z,p);

cineq = [max(z(2,:))-pi/2, -min(z(2,:)), max(z(3,:))-pi/2, -min(z(3,:))];                                          

ceq = [t(indices(1)) - ctrl.tf];                                            
                                                            
% simply comment out any alternate constraints when not in use
    
end