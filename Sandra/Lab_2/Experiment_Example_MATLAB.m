function output_data = Experiment_Example_MATLAB()
    figure(1);  clf;       % Create an empty figure to update later
    subplot(411)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    xlabel('Time (sec)');ylabel('Position (rad)');
    title('Position');
    
    subplot(412)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    xlabel('Time (sec)'); ylabel('Velocity (rad/s)');
    title('Velocity');
    
    subplot(413)
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    xlabel('Time (sec)'); ylabel('Current (Amps)');
    title('Current');
    
    subplot(414)
    h4 = plot([0],[0]);
    h4.XData = []; h4.YData = [];
    xlabel('Time (sec)'); ylabel('Voltage Input (V)');
    title('Voltage Input');
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos = new_data(:,2); % position
        vel = new_data(:,3); % velocity
        curr = new_data(:,4); % current
        volt = new_data(:,5); % voltage
        N = length(pos);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = pos;
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = vel;
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = curr;
        h4.XData(end+1:end+N) = t;   % Update subplot 4
        h4.YData(end+1:end+N) = volt;
    end
    
    frdm_ip  = '192.168.1.100';     % Nucleo board ip
    frdm_port= 11223;               % Nucleo board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
    
    % The example program provided takes two arguments
    
    % Total experiment time is buffer,trajectory,buffer
    traj_time         = 0.5;
    pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
    post_buffer_time  = 2;
    
    th1_init = -pi/4;      % angle 1 for starting squat position of leg
    th2_init = 5*pi/6;     % angle 2 for starting squat position of leg 
%     th1_d = 2*pi/3;       % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
%     th2_d = pi/6;          % angle 1 for desired angle of leg "jump/push-off" from ground (from simulation)
    th1_d = 0;
    th2_d = pi/4;

    
    % SQUAT GAINS
    K_p_s = 7;
%     K_d = 0.1;
%     K_i = 0.01;
    K_d_s = 0;
    K_i_s = 0;
    
    % JUMP GAINS
    K_p = 150;
%     K_d = 0.1;
%     K_i = 0.01;
    K_d = 0;
    K_i = 0;
    input = [traj_time, pre_buffer_time, post_buffer_time, th1_init, th2_init, th1_d, th2_d, K_p_s, K_d_s, K_i_s, K_p, K_p, K_d, K_i];    % input sent to Nucleo board
    output_size = 6;    % number of outputs expected
    
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
        
    
end
