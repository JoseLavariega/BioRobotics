function simulate_leg_4links()
    %% Definte fixed paramters
    setpath
    m0 = 0.2;
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;
    
    restitution_coeff = 0.;
    friction_coeff = 10;
    ground_height = 0;
    y_offset = 0.5;
    
    %New parameters for new linkage
    m5 = 0.1;
    l_E_m5 = 0.01;
    l_EF   = 0.02;
    I5     = 9.25 * 10^-6;
%     kappa = 0.15;
    kappa = 1; %we changed this just to see
    
    global flying
    flying = 0;
    
    %Initial Conditions
    th1_0 = 0;
    th2_0 = pi/4;
    th3_0 = pi/4;
    x_0   = 0;
    y_0   = find_y0(th1_0, th2_0, th3_0, l_OA, l_OB, l_AC, l_DE, l_EF) + y_offset;  
    
    %% Parameter vector
    p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 l_OA l_OB l_AC l_DE l_EF g kappa th3_0]';
    % add in  m0
    %% Simulation Parameters Set 2 -- Operational Space Control
    p_traj.omega = 0;
    p_traj.x_0   = 0;
    p_traj.y_0   = 10;
    p_traj.r     = 0.025;
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 5;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step);
    
    z0 = [th1_0; th2_0; th3_0; x_0; y_0; ...
          0;0; 0;0;0];
      
    z_out = zeros(10,num_step);
    z_out(:,1) = z0;
    
    for i=1:num_step-1
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj);
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        z_out(6:10,i+1) = joint_limit_constraint(z_out(:,i+1),p);
        z_out(6:10,i+1) = discrete_impact_contact(z_out(:,i+1), p, restitution_coeff, friction_coeff, ground_height);

        % Position update
        z_out(1:5,i+1) = z_out(1:5,i) + z_out(6:10,i+1)*dt;
    end
    
    %% Compute Energy
%     E = energy_leg(z_out,p);
%     figure(1); clf
%     plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
%     
% %     %% Compute foot position over time
%     rO = zeros(2,length(tspan));
%     vO = zeros(2,length(tspan));
%     for i = 1:length(tspan)
%         rO(:,i) = position_rO(z_out(:,i),p);
%         vO(:,i) = velocity_rO(z_out(:,i),p);
%     end
% %     
%     figure(2); clf;
%     plot(tspan,rO(1,:),'r','LineWidth',2)
%     hold on
%     plot(tspan,p_traj.x_0 + p_traj.r * cos(p_traj.omega*tspan) ,'r--');
%     plot(tspan,rO(2,:),'b','LineWidth',2)
%     plot(tspan,p_traj.y_0 + p_traj.r * sin(p_traj.omega*tspan) ,'b--');
%         
%     xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','x_d','y','y_d'});
% 
%     figure(3); clf;
%     plot(tspan,vO(1,:),'r','LineWidth',2)
%     hold on
%     plot(tspan,vO(2,:),'b','LineWidth',2)
%     
%     xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
%     
%     figure(4)
%     plot(tspan,z_out(1:3,:)*180/pi)
%     legend('q1','q2', 'q3');
%     xlabel('Time (s)');
%     ylabel('Angle (deg)');
%     
%     figure(5)
%     plot(tspan,z_out(4:6,:)*180/pi)
%     legend('q1dot','q2dot', 'q3dot');
%     xlabel('Time (s)');
%     ylabel('Angular Velocity (deg/sec)');
%     
    %% Animate Solution
    figure(6); clf;
    hold on

    plot([-.2 2],[ground_height ground_height],'k'); 
    
    animateSol(tspan, z_out,p);
end

function tau = control_law(t, z, p, p_traj)
    % Controller gains, Update as necessary for Problem 1
    K_1 = 12.5; % Spring stiffness X
    K_2 = 12.5; % Spring stiffness Y
    D_1 = 3;  % Damping X
    D_2 = 3;  % Damping Y
    
    t_start = 1.5;
    t_jump = 2.0;
    t_land = 5.0;
    
    max_length = 0.3;
    
    global flying;
    
    if(z(5) > max_length && t > t_jump) %in the air
            t_land = t;
            flying = 1;
    end
    
    if(z(5) <= max_length && t > t_jump && flying == 1) %landing
            'setting condtions to land';
            t_land = t;
            flying = 2;
    end
    
    flying
    
    if (t <= t_start) %fall to ground
        th1d = -pi/6;
        th2d = pi/4;   
    elseif(t > t_start && t < t_jump) %squat to jump pos
        th1d = -pi/4;
        th2d = 5*pi/6;
    elseif(t >= t_jump && flying == 0) %jump
        'jumping';
        K_1 = 20;
        K_2 = 20;
        D_1 = 0.75;
        D_2 = 0.75;
        
        th1d = -2*pi/3;
        th2d = pi/6;
        
    elseif(flying == 1) %flying
        "in flight";
        K_1 = 12.5;
        K_2 = 12.5;
        D_1 = 1;
        D_2 = 1;
        
        th1d = pi/3;
        th2d = pi/6;
    elseif(flying == 2)
        "trying to land";
        K_1 = 12.5;
        K_2 = 12.5;
        D_1 = 1;
        D_2 = 1;
        
        th1d = 0;
        th2d = pi/2;
    end
    
    %output joint torques
    tau  = [K_1 * (th1d - z(1) ) + D_1 * ( - z(6) );
          K_2 * (th2d - z(2) ) + D_2 * ( - z(7) )];
      
end


function dz = dynamics(t,z,p,p_traj)
    % Get mass matrix
    A = A_leg(z,p); %bounding_leg
    
    % Compute Controls
    tau = control_law(t,z,p,p_traj);
    %tau = zeros(2,1);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_leg(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:5) = z(6:10);
    dz(6:10) = qdd;
%     disp(qdd);
end

function qdot = discrete_impact_contact(z,p, rest_coeff, fric_coeff, yC)
   
    rA = position_rA(z,p);
    drA= velocity_rA(z,p);
    
    rB = position_rB(z,p);
    drB= velocity_rB(z,p);
    rC = position_rC(z,p);
    drC= velocity_rC(z,p);
    rD = position_rD(z,p);
    drD= velocity_rD(z,p);

    rE = position_foot(z, p);
    rF = position_foot_rF(z,p);
    rE_dot = velocity_foot(z, p);
    rF_dot = velocity_foot_rF(z,p);
    
    qdot = z(6:10);
    C_y = rE(2)-yC;
    C_ydot = rE_dot(2);%Constraaint definition
    
    C_y_f = rF(2)-yC;
    C_ydot_f= rF_dot(2);
    
    C_y_A = rA(2)-yC;
    C_ydot_A= drA(2);
    
    C_y_B = rB(2)-yC;
    C_ydot_B= drB(2);
    C_y_C = rC(2)-yC;
    C_ydot_C= drC(2);
    
    C_y_D = rD(2)-yC;
    C_ydot_D= drD(2);
    
    
%     % E
%     if((C_y<0 && C_ydot < 0)) %enforcing constrait violation
%       J  = jacobian_E(z,p);
%       M_joint = A_leg(z,p);
%       M_joint_inv = inv(M_joint); %Mass operationalspace
%       
%       J_x = J(1,:);
%       J_y = J(2,:);
%       lambda_y = inv(J_y*M_joint_inv*J_y.');
%       F_y = lambda_y*(-rest_coeff*rE_dot(2) - J_y*qdot);
%       qdot = qdot + M_joint_inv*J_y.'*F_y;
%       
%       %For X:
%       lambda_x = inv(J_x*M_joint_inv*J_x.');
%       F_x = lambda_x * (0 - J_x * qdot);
%       if( abs(F_x) > fric_coeff*F_y)
%           F_x = sign(F_x)*F_y*fric_coeff; %% good insight for formulating this not my own.
%       end
%       qdot = qdot + M_joint_inv*J_x.'*F_x;
%       z_test = z;
%       z_test(6:10) = qdot;
%       rE_dot = velocity_foot(z_test, p);
%       
%     end
     
    
    
    % F
    if( (C_y_f<0 && C_ydot_f<0))%enforcing constrait violation on F (end)
      J  = jacobian_foot(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*rF_dot(2) - J_y*qdot);
      qdot = qdot + M_joint_inv*J_y.'*F_y;
      
      %For X:
      lambda_x = inv(J_x*M_joint_inv*J_x.');
      F_x = lambda_x * (0 - J_x * qdot);
      if( abs(F_x) > fric_coeff*F_y)
          F_x = sign(F_x)*F_y*fric_coeff; %% good insight for formulating this not my own.
      end
      qdot = qdot + M_joint_inv*J_x.'*F_x;
      z_test = z;
      z_test(6:10) = qdot;
      rF_dot = velocity_foot_rF(z_test, p);
      
    end
   
%     %B
    if( (C_y_B < 0 && C_ydot_B<0) ) %|| (C_y_C < 0 && C_ydot_C<0) || (C_y_D < 0 && C_ydot_D<0) ) %enforcing constrait violation
      J  = jacobian_B(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*drB(2) - J_y*qdot);
      qdot = qdot + M_joint_inv*J_y.'*F_y;
      
      %For X:
      lambda_x = inv(J_x*M_joint_inv*J_x.');
      F_x = lambda_x * (0 - J_x * qdot);
      if( abs(F_x) > fric_coeff*F_y)
          F_x = sign(F_x)*F_y*fric_coeff; %% good insight for formulating this not my own.
      end
      qdot = qdot + M_joint_inv*J_x.'*F_x;
      z_test = z;
      z_test(6:10) = qdot;
      drB = velocity_foot(z_test, p);
    end
    
   
%     %C
    if(  (C_y_C < 0 && C_ydot_C<0) )%|| (C_y_D < 0 && C_ydot_D<0) ) %enforcing constrait violation
      J  = jacobian_C(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*drC(2) - J_y*qdot);
      qdot = qdot + M_joint_inv*J_y.'*F_y;
      
      %For X:
      lambda_x = inv(J_x*M_joint_inv*J_x.');
      F_x = lambda_x * (0 - J_x * qdot);
      if( abs(F_x) > fric_coeff*F_y)
          F_x = sign(F_x)*F_y*fric_coeff; %% good insight for formulating this not my own.
      end
      qdot = qdot + M_joint_inv*J_x.'*F_x;
      z_test = z;
      z_test(6:10) = qdot;
      drC = velocity_foot(z_test, p);
    end

end

function qdot = joint_limit_constraint(z,p)
   %th1
    th1_min = -3*pi/4; %constraint
    th1_max = 3*pi/4;
    C1_min = z(1) - th1_min; 
    C1_max = th1_max - z(1);
    dC1 = z(6);
    qdot = z(6:10);
    
    J = [1 0 0 0 0];
    A = A_leg(z,p);

    if (C1_min < 0 && dC1 <0)% if constraint is violated
        test_print = "min angle violated"
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (0 - dC1);
        inv(A);
        qdot = qdot + inv(A)*J.'*F_c;        % generalformulation of constraint
    elseif (C1_max < 0 && dC1 > 0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (dC1);
        qdot = qdot + inv(A)*J.'*F_c;
    end
    
    %th2
    J = [0 1 0 0 0];
    
    th2_min = pi/6; %constraint
    th2_max = 5*pi/6;
    C2_min = z(2) - th2_min; 
    C2_max = th2_max - z(2);
    dC2 = z(7);
    
    qdot = z(6:10);
    
     if (C2_min < 0 && dC2 <0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (0 - dC2);
        qdot = qdot + inv(A)*J.'*F_c;        % generalformulation of constraint
    elseif (C2_max < 0 && dC2 > 0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (dC2);
        qdot = qdot + inv(A)*J.'*F_c;
     end
    
    %th3
    th3_min = pi/6; %constraint
    th3_max = 5*pi/6;
    C3_min = z(3) - th3_min; 
    C3_max = th3_max - z(3);
    dC3 = z(8);
    
    qdot = z(6:10);
    
    J = [0 0 1 0 0];
    
     if (C3_min < 0 && dC3 <0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (0 - dC3);
        qdot = qdot + inv(A)*J.'*F_c;        % generalformulation of constraint
    elseif (C3_max < 0 && dC3 > 0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (dC3);
        qdot = qdot + inv(A)*J.'*F_c;
    end

end

function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_EF = plot([0],[0],'LineWidth',2);
   
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.25 2 -.25 1]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_leg(z,p);

        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        rF = keypoints(:,6);
        rO = keypoints(:,7);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OB,'XData',[rO(1) rB(1)]);
        set(h_OB,'YData',[rO(2) rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);
        
        set(h_EF,'XData',[rE(1) rF(1)]);
        set(h_EF,'YData',[rE(2) rF(2)]);

        pause(.01)
    end
end

function y0 = find_y0(th1, th2, th3, l_OA, l_OB, l_AC, l_DE, l_EF)
ihat = [0; -1; 0];
jhat = [1; 0; 0];

khat = cross(ihat,jhat);
e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;
e3hat =  cos(th1+th2+th3)*ihat + sin(th1+th2+th3)*jhat;

rO = 0;
rA = rO + l_OA * e1hat;
rB = rO + l_OB * e1hat;
rC = rA  + l_AC * e2hat;
rD = rB  + l_AC * e2hat;
rE = rD  + l_DE * e1hat;
rF = rE + l_EF * e3hat;

y0 = -rF(2);

end