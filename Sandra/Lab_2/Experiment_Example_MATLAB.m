function output_data = Experiment_Example_MATLAB()
%     figure(1);  clf;       % Create an empty figure to update later
%     subplot(411)
%     h1 = plot([0],[0]);
%     h1.XData = []; h1.YData = [];
%     xlabel('Time (sec)');ylabel('Position (rad)');
%     title('Position');
%     
%     subplot(412)
%     h2 = plot([0],[0]);
%     h2.XData = []; h2.YData = [];
%     xlabel('Time (sec)'); ylabel('Velocity (rad/s)');
%     title('Velocity');
%     
%     subplot(413)
%     h3 = plot([0],[0]);
%     h3.XData = []; h3.YData = [];
%     xlabel('Time (sec)'); ylabel('Current (Amps)');
%     title('Current');
%     
%     subplot(414)
%     h4 = plot([0],[0]);
%     h4.XData = []; h4.YData = [];
%     xlabel('Time (sec)'); ylabel('Voltage Input (V)');
%     title('Voltage Input');
%     
%     figure(2);  clf;       % Create an empty figure to update later
%     subplot(411)
%     h5 = plot([0],[0]);
%     h5.XData = []; h5.YData = [];
%     xlabel('Time (sec)');ylabel('Position (rad)');
%     title('Position');
%     
%     subplot(412)
%     h6 = plot([0],[0]);
%     h6.XData = []; h6.YData = [];
%     xlabel('Time (sec)'); ylabel('Velocity (rad/s)');
%     title('Velocity');
%     
%     subplot(413)
%     h7 = plot([0],[0]);
%     h7.XData = []; h7.YData = [];
%     xlabel('Time (sec)'); ylabel('Current (Amps)');
%     title('Current');
%     
%     subplot(414)
%     h8 = plot([0],[0]);
%     h8.XData = []; h8.YData = [];
%     xlabel('Time (sec)'); ylabel('Voltage Input (V)');
%     title('Voltage Input');
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos1 = new_data(:,2); % position
        vel1 = new_data(:,3); % velocity
        curr1 = new_data(:,4); % current
        volt1 = new_data(:,5); % voltage
        N1 = length(pos1);
        
        pos2 = new_data(:,6); % position
        vel2 = new_data(:,7); % velocity
        curr2 = new_data(:,8); % current
        volt2 = new_data(:,9); % voltage
        N2 = length(pos2);
        
        h1.XData(end+1:end+N1) = t;   % Update subplot 1
        h1.YData(end+1:end+N1) = pos1;
        h2.XData(end+1:end+N1) = t;   % Update subplot 2
        h2.YData(end+1:end+N1) = vel1;
        h3.XData(end+1:end+N1) = t;   % Update subplot 3
        h3.YData(end+1:end+N1) = curr1;
        h4.XData(end+1:end+N1) = t;   % Update subplot 4
        h4.YData(end+1:end+N1) = volt1;
        h5.XData(end+1:end+N2) = t;   % Update subplot 1
        h5.YData(end+1:end+N2) = pos2;
        h6.XData(end+1:end+N2) = t;   % Update subplot 2
        h6.YData(end+1:end+N2) = vel2;
        h7.XData(end+1:end+N2) = t;   % Update subplot 3
        h7.YData(end+1:end+N2) = curr2;
        h8.XData(end+1:end+N2) = t;   % Update subplot 4
        h8.YData(end+1:end+N2) = volt2;
    end
    
    frdm_ip  = '192.168.1.100';     % Nucleo board ip
    frdm_port= 11223;               % Nucleo board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout

    
    % Total experiment time is buffer,trajectory,buffer
    pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
    traj_time         = 0.5;
    post_buffer_time  = 2;
    
%     th1_init = -pi/4;      % angle 1 for starting squat position of leg
%     th2_init = 5*pi/6;     % angle 2 for starting squat position of leg 
% %     th1_d = 2*pi/3;       % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
% %     th2_d = pi/6;          % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
%     th1_d = 0;
%     th2_d = pi/4;
% 
%     
%     % SQUAT GAINS
%     K_p_s = 7;
% %     K_d = 0.1;
% %     K_i = 0.01;
%     K_d_s = 0;
%     K_i_s = 0;
%     
%     % JUMP GAINS
%     K_p = 150;
% %     K_d = 0.1;
% %     K_i = 0.01;
%     K_d = 0;
%     K_i = 0;
%     
%     input = [traj_time, pre_buffer_time, post_buffer_time, th1_init, th2_init, th1_d, th2_d, K_p_s, K_d_s, K_i_s, K_p, K_p, K_d, K_i];    % input sent to Nucleo board

%     th1_init   = -pi/4;      % angle 1 for starting squat position of leg
%     th2_init   = 5*pi/6;     % angle 2 for starting squat position of leg 
% %     th1_d = 2*pi/3;       % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
% %     th2_d = pi/6;          % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
%     th1_d      = 2*pi/3;
%     th2_d      = pi/6;
%     th1_land   = -2*pi/3 ;
%     th2_land   = -pi/6;


    th1_init   = 0;      % angle 1 for starting squat position of leg
    th2_init   = -pi/4;     % angle 2 for starting squat position of leg 
%     th1_d = 2*pi/3;       % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
%     th2_d = pi/6;          % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
    th1_d      = pi/5;
    th2_d      = pi/3;
    th1_land   = -pi/5;
    th2_land   = -pi/3;

    % SQUAT GAINS
    K_p_s = 10;
    K_d_s = 0;
    K_i_s = 0;
    
    % JUMP GAINS
    K_p = 50;
    K_d = 0;
    K_i = 0;
    
    % LAND GAINS
    K_p_l = 10;
    K_d_l = 0;
    K_i_l = 0;
    
   input = [th1_init, th2_init, K_p_s, K_i_s, K_d_s, th1_d, th2_d, K_p, K_i, K_d, th1_land, th2_land,K_p_l, K_i_l, K_d_l, pre_buffer_time, traj_time, post_buffer_time];    % input sent to Nucleo board
    output_size = 9;    % number of outputs expected
    
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
        
    
end
