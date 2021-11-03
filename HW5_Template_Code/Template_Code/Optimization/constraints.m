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
    ctrl.tf= x(2);
    ctrl.T = x(3:end);
    [t_out,z_out,u_out,idc] = hybrid_simulation(z0, ctrl, p, [0 tf]);
    
    leg_not_below_ground = -min(z_out(2,:));  %leg below ground
    leg_not_hyperextend  =  max(z_out(2,:) - pi/2); %leg not hyperextended
    cineq = [leg_not_below_ground , leg_not_hyperextend];   
        
    tf_ctrl_takeoff = t_out(idc(1))- ctrl.tf ; %times match
    
    %Apex CoM height y_cm(x,z0,p) =0.4
    com = COM_jumping_leg(z_out,p);
    com_x = com(1,:);
    com_y = com(2,:);
    com_ydot =com(4,:);
    
    apex_com_height = com_y(end)-0.4; %y is at apex
    vel_constraint  = com_ydot(end);  %velocity at apex is 0

    ceq = [tf_ctrl_takeoff, apex_com_height, vel_constraint];                                            
                                                            
% simply comment out any alternate constraints when not in use
end