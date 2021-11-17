function [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,tspan)
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
%
    t0 = tspan(1); tend = tspan(end);   % set initial and final times
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(2,1);
    iphase_list = 1;
    for i = 1:num_step-1
%     for i = 1:1
        t = tout(i);
        
        iphase = iphase_list(i);
%         zout(:,i)
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase);
        zout(:,i+1) = zout(:,i) + dz*dt;
        zout(4:6,i+1) = discrete_impact_contact(zout(:,i+1), p);
        zout(1:3,i+1) = zout(1:3,i) + zout(4:6, i+1)*dt;
        uout(:,i+1) = u; 
        
        if(zout(1, i+1) > 0 && iphase == 1) % jump
            iphase = 2;
        elseif(zout(1,i+1) < 0 && iphase == 2) % max height
            iphase = 3;
        end
        iphase_list(i+1) = iphase;
    end
    
    j=1;
    indices = 0;
    for i = 1:num_step-1
        if (iphase_list(i+1) == iphase_list(i))
            indices(j) = indices(j)+1;
        else
            j = j+1;
            indices(j) = 0;
        end
    end
end

%% Discrete Contact
function qdot = discrete_impact_contact(z,p)
    qdot = z(4:6);
    rE = z(1); 
    vE = z(4); 

    if(rE<0 && vE < 0)
      J  = [1, 0, 0];
      M = A_jumping_leg(z,p);
      Ainv = inv(M);
      
      lambda_z = 1/(J * Ainv * J.');
      F_z = lambda_z*(0 - vE);
      qdot = qdot + Ainv*J.'*F_z;
    end
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
%     z
%     p
%     u
    A = A_jumping_leg(z,p);                % get full A matrix
    b = b_jumping_leg(z,u,0,p);               % get full b vector
    
    x = A\b;               % solve system for accelerations (and possibly forces)
    dz(1:3,1) = z(4:6); % assign velocities to time derivative of state vector
    dz(4:6,1) = x(1:3);   % assign accelerations to time derivative of state vector
end

%% Control
function u = control_laws(t,z,ctrl,iphase)

    if iphase == 1
        u = BezierCurve(ctrl.T, t/ctrl.tf);
    else
        
        % PD Control in flight
        th1 = z(2,:);            % leg angle
        dth1 = z(5,:);           % leg angular velocity
        
        th2 = z(3,:);
        dth2 = z(6,:);

        th1d = pi/4;             % desired leg angle
        th2d = pi/4;
        
        k = 5;                  % stiffness (N/rad)
        b = .5;                 % damping (N/(rad/s))

        u = [-k*(th1-th1d) - b*dth1; -k*(th2-th2d) - b*dth2]; % apply PD control
        
        
        
    end

end