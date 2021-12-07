function simulate_leg_4links()
    %% Definte fixed paramters
    %setpath
    m0 = 0.2;
    m1 =.220 + .1;         m2 =.0368; 
    m3 = .00783;            m4 = .02272;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.041; 
    l_AC=.096;              l_DE=.177;
    l_O_m1=0.033;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.12;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;  
    
    restitution_coeff = 0.;
    friction_coeff = 0.5;
    ground_height = 0;
    y_offset =0.26;
    
    %New parameters for new linkage
    m5 = 0.01283;
    l_E_m5 = 0.012;
    l_EF   = 0.066;
    I5     = 9 * 10^-6;
    
    global flying
    flying = 0;
    
    % MODIFIED PARAMETERS 
    kappa = [0.00667 0.02 0.04 0.006 0.01166 0.04 0.0044 0.010769 0.02667];         % to be determined. Amount of rubber bands
    squat_th1 = linspace(-pi/4,pi/2,50);         % TBD. squat(starting) angle The index is the configuration to be used. 
    squat_th2 = linspace(pi/6,5*pi/6,50);        % TBD. Squat(starting) angle
    jump_th1  = [-2.8*pi/10];       % TBD. Impulse Angle
    jump_th2  = [pi/8];           % TBD. Impulse Angle
    
    %Want to find,stiffness,angle that maximizes horizontal forward
    %distance traveled out of a single hop.
    
    %Initial Conditions
    th1_0 = 0;
    th2_0 = pi/2;
    th3_0 = -pi/6; % to be determined
    x_0   = 0;
    y_0   = find_y0(th1_0, th2_0, th3_0, l_OA, l_OB, l_AC, l_DE, l_EF) + y_offset;
    %TODO: Print y_offset
    
    

    %% Simulation Parameters Set 2 -- Operational Space Control
    p_traj.omega = 0;
    p_traj.x_0   = 0;
    p_traj.y_0   = -.12; %stand at 0,0  %10
    p_traj.r     = 0;                   %0.025
    
    
    current_index = 1;
    
    distances_storage  = cell(length(kappa),1);
    %distances_traveled = zeros(length(squat_th1),length(squat_th2));
  
  for kappa_sel=1:length(kappa)
      
      distances_traveled = zeros(length(squat_th1),length(squat_th2));
   for th_1=1:length(squat_th1)
    for th_2=1:length(squat_th2) %put here your independent variable
    %% Perform Dynamic simulation
        %% Parameter vector
        p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 l_OA l_OB l_AC l_DE l_EF g kappa(kappa_sel) th3_0]';
        %In the line above, modify kappa to kappa(sel)as needed
    
    
        dt = 0.001;
        tf = 1.8;
        num_step = floor(tf/dt);
        tspan = linspace(0, tf, num_step);

        z0 = [th1_0; th2_0; th3_0; x_0; y_0; ...
               0;        0;      0;  0;   0];

        z_out = zeros(10,num_step);
        z_out(:,1) = z0;
        last_valid_index =1;

        for i=1:num_step-1
            
            
            dz = dynamics(tspan(i), z_out(:,i), p, p_traj, squat_th1(th_1), squat_th2(th_2),jump_th1(1),jump_th2(1)); 
            %cCHANGETHE PARAMETERS ABOVE AS NEEDED
            
            % Velocity update with dynamics
            z_out(:,i+1) = z_out(:,i) + dz*dt;

            z_out(6:10,i+1) = joint_limit_constraint(z_out(:,i+1),p);
            z_out(6:10,i+1) = discrete_impact_contact(z_out(:,i+1), p, restitution_coeff, friction_coeff, ground_height);

            % Position update
            z_out(1:5,i+1) = z_out(1:5,i) + z_out(6:10,i+1)*dt;
            
            
            if(isnan(z_out(4,i)))
                disp(z_out(4,i))
                break;
                
            elseif(z_out(4,i)>3)
                    break;
            end
            
            last_valid_index = i;
            
           
        end
        z_out(1:10, last_valid_index)
        distances_traveled(th_1,th_2) = z_out(4,last_valid_index);
    
    end
    
   end
   
   for i=1:length(squat_th1)
       for j=1:length(squat_th2)
           if(distances_traveled(i,j) > 1.1 || distances_traveled(i,j)<-1.1)
               distances_traveled(i,j) = 0;
           end
       end
   end
   
   distances_traveled
   kappa_sel
   distances_storage{kappa_sel} = distances_traveled;
   
  end
    
    %% Compute Energy
    E = energy_leg(z_out,p);
    figure(1); clf
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    %% Compute foot position over time
    rE = zeros(2,length(tspan));
    vE = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rE(:,i) = position_foot(z_out(:,i),p);
        vE(:,i) = velocity_foot(z_out(:,i),p);
    end
    
    figure(2); clf;
    plot(tspan,rE(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,p_traj.x_0 + p_traj.r * cos(p_traj.omega*tspan) ,'r--');
    plot(tspan,rE(2,:),'b','LineWidth',2)
    plot(tspan,p_traj.y_0 + p_traj.r * sin(p_traj.omega*tspan) ,'b--');
    
    
    xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','x_d','y','y_d'});

    figure(3); clf;
    plot(tspan,vE(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,vE(2,:),'b','LineWidth',2)
    
    xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
    
    figure(4)
    plot(tspan,z_out(1:3,:)*180/pi)
    legend('q1','q2', 'q3');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    
    figure(5)
    plot(tspan,z_out(4:6,:)*180/pi)
    legend('q1dot','q2dot', 'q3dot');
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/sec)');
    
    
    figure(6)
    plot3(tspan,rE(1,:),rE(2,:))
    xlabel('Time(s)');
    ylabel('HorizontalDistance');
    zlabel('Vertical Distance(m)');
    
    
    %% Animate Solution
%     figure(7); clf;
%     hold on
%    
%   
%     % Target traj
%     TH = 0:.1:2*pi;
%     plot( p_traj.x_0 + p_traj.r * cos(TH), ...
%           p_traj.y_0 + p_traj.r * sin(TH),'k--'); 
%     
%     % Ground Q2.3
%     plot([-.2 .2],[ground_height ground_height],'k'); 
%     
%     animateSol(tspan, z_out,p);
    %%
    
    % Independent Variable Code
    %figure(8); 
    %clf;
%     hold on
%     scatter(kappa, distances_traveled,'filled' ); %change to values iterated on
%     xlabel('Independent Variable (kappa, thetas_bend, or thetas_launch');
%     ylabel('Horizontal Distance Traveled (Origin)')

    %hold on
    
    figure(8); 
    clf;
    distances_traveled = distances_storage{1};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(9); 
    clf;
    distances_traveled = distances_storage{2};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(10); 
    clf;
    distances_traveled = distances_storage{3};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(11); 
    clf;
    distances_traveled = distances_storage{4};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(12); 
    clf;
    distances_traveled = distances_storage{5};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(13); 
    clf;
    distances_traveled = distances_storage{6};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(14); 
    clf;
    distances_traveled = distances_storage{7};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(15); 
    clf;
    distances_traveled = distances_storage{8};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
    figure(16); 
    clf;
    distances_traveled = distances_storage{9};
    surf(squat_th1, squat_th2, distances_traveled);
    xlabel('theta 1 (rad)');
    ylabel('theta 2 (rad)');
    zlabel('distance traveled (m)');
    
   
    
    
    distances_traveled
    %distances_traveled(5,5)
    
    %length(squat_th1)
    %length(squat_th2)
end

% function tau = control_law(t,z,p,p_traj)
% 
% 
% end


function tau = control_law(t, z, squat_th1, squat_th2,jump_th1,jump_th2)
    % Controller gains, Update as necessary for Problem 1
    K_1 = 15.; % Spring stiffness X
    K_2 = 10.; % Spring stiffness Y
    D_1 = 3.5;  % Damping X
    D_2 = 2.;  % Damping Y

    t_start = 0.5;
    t_jump  = 1.0;
    t_land  = 2.5;
    
    max_length = 0.3;
    global flying;
    
    % Desired position of foot is a circle
%     omega_swing = p_traj.omega;
%     rEd = [p_traj.x_0 p_traj.y_0 0]' + ...
%             p_traj.r*[cos(omega_swing*t) sin(omega_swing*t) 0]';
%     rEd = [p_traj.x_0 p_traj.y_0 0]';
    % Compute desired velocity of foot
%     vEd = p_traj.r*[-sin(omega_swing*t)*omega_swing    ...
%                      cos(omega_swing*t)*omega_swing   0]';

%     vEd = [0 0 0]';
%     aEd = [0 0 0]';
    % Desired acceleration
%      aEd = p_traj.r*[-cos(omega_swing*t)*omega_swing^2 ...
%                      -sin(omega_swing*t)*omega_swing^2 0]';
    
    % Actual position and velocity 
    %z
%     rE = position_foot(z,p);
%      vE = velocity_foot(z,p);
    
    % Compute virtual foce for Question 1.4 and 1.5
%     f  = [K_x * (rEd(1) - rE(1) ) + D_x * ( - vE(1) ) ;
%           K_y * (rEd(2) - rE(2) ) + D_y * ( - vE(2) )]; 
%           
%     th1_d = 0;
%     th2_d = 7*pi/8;
%     f = [K_x*(th1_d- z(1)) + D_x*(-z(6)); 
%         K_y*(th2_d-z(2)) + D_y*(-z(7))];
    
%     %% Task-space ompensation and feed forward for Question 1.8 <- Typo?
%     % get the M,V,G matrices:
%     M_joint = A_leg(z,p); %A_bounding_leg
%     V_joint = Corr_leg(z,p);
%     G_joint = Grav_leg(z,p);
%     
%     inv_M_joint = inv(M_joint);
%     q_dot = z(6:10);
% %     
% %     % Map to joint torques  
%     J  = jacobian_foot(z,p);
%     J_dot = jacobian_dot_foot(z,p);
    
%     % compute Lambda,mu, rho
%     lambda = inv(J*inv_M_joint*J');
% %     lambda
% %     J_dot
% %     q_dot
% %     J
% %     inv_M_joint
%     mu = lambda*J*inv_M_joint*V_joint - lambda*J_dot*q_dot;
%     rho = lambda*J*inv_M_joint*G_joint;
%     
%     f(1:2) = lambda*(aEd(1:2) + f(1:2)) + mu + rho; %use the resources at our disposal
%     %f(1:2) = lambda*(aEd(1:2) + f(1:2)) + mu;
%     %f(1:2) = lambda*(aEd(1:2) + f(1:2)) + rho;
%     %f(1:2) = lambda*(f(1:2)) + mu + rho;    
%     tau = J' * f;
%     tau = [0;0;0; 0;0];


    if(z(5) > max_length && t > t_jump) %in the air
            t_land = t;
            flying = 1;
    end
    
    if(z(5) <= max_length && t > t_jump && flying == 1) %landing
            'setting condtions to land';
            t_land = t;
            flying = 2;
    end
    
    
    if (t <= t_start) %fall to ground
        th1_d = -pi/6;
        th2_d = pi/4;   
    elseif(t > t_start && t < t_jump) %squat to jump pos
        th1_d = squat_th1;
        th2_d = squat_th2;
    elseif(t >= t_jump && flying == 0) %jump
        'jumping';
        K_1 = 20;
        K_2 = 20;
        D_1 = 0.75;
        D_2 = 0.75;
        
        th1_d = jump_th1;
        th2_d = jump_th2;
        
        %beyond this point this is only a landing controller, no
        %longe relevant. 
    elseif(flying == 1) %flying
        "in flight";
        K_1 = 12.5;
        K_2 = 12.5;
        D_1 = 1;
        D_2 = 1;
        
        th1_d = jump_th1;
        th2_d = jump_th2;
    elseif(flying == 2)
        "trying to land";
        K_1 = 12.5;
        K_2 = 12.5;
        D_1 = 1;
        D_2 = 1;
        
        th1_d = 0;
        th2_d = pi/4;
    end
    
    % Controller
    tau = [K_1*(th1_d- z(1)) + D_1*(-z(6)); 
        K_2*(th2_d-z(2)) + D_2*(-z(7)); 0;0;0];
    
    
    
    
end


function dz = dynamics(t,z,p,p_traj,squat_th1,squat_th2, jump_th1,jump_th2)
    % Get mass matrix
    A = A_leg(z,p); %bounding_leg
    
    % Compute Controls
    tau = control_law(t,z,squat_th1, squat_th2,jump_th1,jump_th2);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_leg(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:5) = z(6:10);
    dz(6:10) = qdd;
    %disp(qdd);
end


%%  Constraint Functions

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
    
    
    % E
    if((C_y<0 && C_ydot < 0)) %enforcing constrait violation
      J  = jacobian_E(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*rE_dot(2) - J_y*qdot);
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
      rE_dot = velocity_foot(z_test, p);
      
    end
     
    
    
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
    
   
    % A
    if( (C_y_A < 0 && C_ydot_A<0)) % ||...
          %(C_y_B < 0 && C_ydot_B<0) || (C_y_C < 0 && C_ydot_C<0) || (C_y_D < 0 && C_ydot_D<0) ) %enforcing constrait violation
      J  = jacobian_A(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*drA(2) - J_y*qdot);
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
      drA = velocity_rA(z_test, p);
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
   
    
    %D
    if(   (C_y_D < 0 && C_ydot_D<0) ) %enforcing constrait violation
      J  = jacobian_D(z,p);
      M_joint = A_leg(z,p);
      M_joint_inv = inv(M_joint); %Mass operationalspace
      
      J_x = J(1,:);
      J_y = J(2,:);
      lambda_y = inv(J_y*M_joint_inv*J_y.');
      F_y = lambda_y*(-rest_coeff*drD(2) - J_y*qdot);
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
      drD = velocity_foot(z_test, p);
   end
%     
        
        
    
    

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
    axis([-.5 1 -.5 .5]);

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

        pause(.03)
    end
end

% END Constraint Functions


%% Find y0 function

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
