function simulate_leg_4links()
    %% Definte fixed paramters
    %setpath
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
    ground_height = -0.12;
    
    %New parameters for new linkage
    m5 = 0.1;
    l_E_m5 = 0.01;
    l_EF   = 0.02;
    I5     = 9.25 * 10^-6;
    kappa = 0.15;
    
    %Initial Conditions
    th1_0 = -pi/4;
    th2_0 = pi/2;
    th3_0 = -pi/12;
    x_0   = 0;
    y_0   = 0;
    
    
    %% Parameter vector
    p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 l_OA l_OB l_AC l_DE l_EF g kappa th3_0]';
    % add in  m0
    %% Simulation Parameters Set 2 -- Operational Space Control
    p_traj.omega = 0;
    p_traj.x_0   = 0;
    p_traj.y_0   = -.125;
    p_traj.r     = 0.025;
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    %z0 = [-pi/4; pi/2;th3_0; 0; 0; 0];
    z0 = [th1_0; th2_0; th3_0; x_0; y_0; ...
          0;0; 0;0;0];
    
    % z0 = [-pi/4; pi/2;th3_0; 0; 1; 0; 0; 0; 0; 0];
    z_out = zeros(10,num_step);
    %z_out = zeros(6,num_step);
    z_out(:,1) = z0;
    
    for i=1:num_step-1
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj);
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        %z_out(4:6,i+1) = joint_limit_constraint(z_out(:,i+1),p);
        z_out(6:10,i+1) = discrete_impact_contact(z_out(:,i+1), p, restitution_coeff, friction_coeff, ground_height);

        % Position update
        z_out(1:5,i+1) = z_out(1:5,i) + z_out(6:10,i+1)*dt;
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
    
    %% Animate Solution
    figure(6); clf;
    hold on
   
  
    % Target traj
    TH = 0:.1:2*pi;
    plot( p_traj.x_0 + p_traj.r * cos(TH), ...
          p_traj.y_0 + p_traj.r * sin(TH),'k--'); 
    
    % Ground Q2.3
    plot([-.2 .2],[ground_height ground_height],'k'); 
    
    animateSol(tspan, z_out,p);
end

function tau = control_law(t, z, p, p_traj)
    % Controller gains, Update as necessary for Problem 1
    K_x = 125.; % Spring stiffness X
    K_y = 125.; % Spring stiffness Y
    D_x = 10.;  % Damping X
    D_y = 10.;  % Damping Y

    % Desired position of foot is a circle
    omega_swing = p_traj.omega;
    rEd = [p_traj.x_0 p_traj.y_0 0]' + ...
            p_traj.r*[cos(omega_swing*t) sin(omega_swing*t) 0]';
    % Compute desired velocity of foot
    vEd = p_traj.r*[-sin(omega_swing*t)*omega_swing    ...
                     cos(omega_swing*t)*omega_swing   0]';
    % Desired acceleration
    aEd = p_traj.r*[-cos(omega_swing*t)*omega_swing^2 ...
                    -sin(omega_swing*t)*omega_swing^2 0]';
    
    % Actual position and velocity 
    z
    rE = position_foot(z,p);
    vE = velocity_foot(z,p);
    
    % Compute virtual foce for Question 1.4 and 1.5
    f  = [K_x * (rEd(1) - rE(1) ) + D_x * ( - vE(1) ) ;
          K_y * (rEd(2) - rE(2) ) + D_y * ( - vE(2) ) ];
    
    %% Task-space ompensation and feed forward for Question 1.8 <- Typo?
    % get the M,V,G matrices:
    M_joint = A_leg(z,p); %A_bounding_leg
    V_joint = Corr_leg(z,p);
    G_joint = Grav_leg(z,p);
    
    inv_M_joint = inv(M_joint);
    q_dot = z(6:10);
    
    % Map to joint torques  
    J  = jacobian_foot(z,p);
    J_dot = jacobian_dot_foot(z,p);
    
    % compute Lambda,mu, rho
    lambda = inv(J*inv_M_joint*J');
%     lambda
%     J_dot
%     q_dot
%     J
%     inv_M_joint
    mu = lambda*J*inv_M_joint*V_joint - lambda*J_dot*q_dot;
    rho = lambda*J*inv_M_joint*G_joint;
    
    f(1:2) = lambda*(aEd(1:2) + f(1:2)) + mu + rho; %use the resources at our disposal
    %f(1:2) = lambda*(aEd(1:2) + f(1:2)) + mu;
    %f(1:2) = lambda*(aEd(1:2) + f(1:2)) + rho;
    %f(1:2) = lambda*(f(1:2)) + mu + rho;    
    tau = J' * f;
    %tau = [0;0;0; 0;0];
end


function dz = dynamics(t,z,p,p_traj)
    % Get mass matrix
    A = A_leg(z,p); %bounding_leg
    
    % Compute Controls
    tau = control_law(t,z,p,p_traj);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_leg(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:5) = z(6:10);
    dz(6:10) = qdd;
    disp(qdd);
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
    q1_min = -50 * pi/ 180; %constraint
    C = z(1) - q1_min; 
    dC= z(6);
    qdot = z(6:19);
    
    J = [1 0];
    A = A_leg(z,p);

    if (C < 0 && dC <0)% if constraint is violated
        lambda = A(2,2); %insightgotten from a ta
        F_c = lambda * (0 - dC);
        qdot = qdot + inv(A)*J.'*F_c;        % generalformulation of constraint
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
    axis([-.5 .5 -.5 .5]);

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