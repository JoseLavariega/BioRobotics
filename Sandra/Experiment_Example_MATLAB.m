function output_data = Experiment_Example_MATLAB()
    figure(1);  clf;       % Create an empty figure to update later
    subplot(511)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (rad)');
    title('Position');
    
    subplot(512)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity (rad/s)');
    title('Velocity');
    
    subplot(513)
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current (Amps)');
    title('Current');
    
    subplot(514)
    h4 = plot([0],[0]);
    h4.XData = []; h4.YData = [];
    ylabel('Voltage (Volts)');
    title('Voltage');
    
    subplot(515)
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Desired Currennt (Amps)');
    title('Desired Current');
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos = new_data(:,2); % position
        vel = new_data(:,3); % velocity
        curr = new_data(:,4); % current
        volts = new_data(:,5); % voltage
        curr_d = new_data(:,6); % desired current
        N = length(pos);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = pos;
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = vel;
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = curr;
        h4.XData(end+1:end+N) = t;   % Update subplot 4
        h4.YData(end+1:end+N) = volts;       
        h5.XData(end+1:end+N) = t;   % Update subplot 5
        h5.YData(end+1:end+N) = curr_d;        
    end
    
    frdm_ip  = '192.168.1.100';     % Nucleo board ip
    frdm_port= 11223;               % Nucleo board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
    
    % The example program provided takes two arguments
    Kp = 22.0;
    Ki = 1.0;
    Desired_Current = 1.0;
    R = 1.81;
    Kb = 0.172;
    input = [Kp Ki Desired_Current R Kb];    % input sent to Nucleo board
    output_size = 6;    % number of outputs expected
    
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
        
end
