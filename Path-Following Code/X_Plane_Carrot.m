function [f] = X_Plane_Carrot(lambda, delta, k_psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X-Plane simulation using the Carrot-Chasing algorithm
%
% INPUT  - NLGL parameters:
%               * lambda
%               * delta
%               * k_psi
%
% OUTPUT - f: cost for the simulationm which consitst in 1000 iterations
%
% NOTE  - The other matrics such as cross-track error and overshoot are
%         stocked inside a global variable calles METRICS and then used to
%         compute an average over the total number of simulations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize local variables
sim = 1;
population_error = 0;

% Initialize global variables
global INDIVIDUAL
global METRICS
global N_ERRORS
global SUM_X
global SUM_Y
global SUM_Z

% Loop to reset the simulation if a failure in communication occurs
while sim == 1  
    
    % Initiate X-Plane simulation
    [Socket, error_flag, population_error, pitch_deg, roll_deg, Va_knots] = init_X_Plane(population_error);
    
    % Collect initial speed and commands of the aircraft
    Va = Va_knots/1.944; % Aircraft speed (m/s)
    aileron_cmd_init = 0;     % Ailerons command [-1,1]
    elevator_cmd_init = 0.3;  % Elevator command [-1,1]
    rudder_cmd_init = 0;      % Rudder command [-1,1]
    throttle_cmd_init = 0;    % Throttle command [-1,1]
    counter = 1;
       
    % After 10 times failuring to connect, a music starts to play to warn the user
    % that X-Plane may have been closed in the background
    if population_error >= 10
        load handel
        sound(y,Fs)
        pause(4)
    end
    
%%%%%%%%%%%%%%%%%%% Setting the simulation variables  %%%%%%%%%%%%%%%%%%%%
    
    % Initializing Low Level Controllers
    Controllers = set_controllers_horizontal(0.01, [deg2rad(roll_deg), deg2rad(pitch_deg)], Va, [1,1,1,1], [0,0]);
    
    % Loop variables
    iterations = 1000;
    
    % Euleur angles
    roll = zeros(1, 1001);
    roll(1) = deg2rad(roll_deg);
    yaw = zeros(1, 1001);
    pitch = zeros(1, 1001);
    pitch(1) = deg2rad(pitch_deg);
    sideslip = zeros(1, 1001);
    
    % GPS position
    x = zeros(1, 1001);
    x(1) = 1200;
    y = zeros(1, 1001);
    z = zeros(1, 1001);
    
    % Actuator commands
    aileron_cmd = zeros(1, 1000);
    elevator_cmd = zeros(1, 1000);
    rudder_cmd = zeros(1, 1000);
    throttle_cmd = zeros(1, 1000);
    
    % Variables for benchmarks
	partial_error = 0;
	Error = zeros(1, 1000);
    error_x = zeros(1, 1000);
    error_y = zeros(1, 1000);
    error_z = zeros(1, 1000);
    error_steady = zeros(1,600);
    
    % REMOVEE
    pitch_desired = zeros(1, 1000);
    
    % Speeds
    wind_speed = zeros(1, 1000);
    UAV_speed = zeros(1, 1000);
    
    % Other variables
    intervalo_real_iter = zeros(1, 1000);
    
    % 3D loiter coordinates; (x,y,z,r) 
    trajectory = [600 600 20 600]; 
    loiter_position = [trajectory(1); trajectory(2); trajectory(3)];
    radius = trajectory(4);
    
    % Sending initial commands to controllers for the first time
    CTRL = [elevator_cmd_init, aileron_cmd_init, rudder_cmd_init, throttle_cmd_init, 0];
    sendCTRL(CTRL, 0, Socket);
    
    % Delay to assure connection
    pause(1.5);
    pauseSim(0,Socket);
    pause(0.5)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    while counter <= iterations && error_flag == 1
        tic;
        
        % Collect DataRefs 
        DREFS = {'sim/flightmodel/position/local_z', ... % Z position
            'sim/flightmodel/position/local_x',      ... % X position
            'sim/flightmodel/position/local_y',      ... % Y position
            'sim/flightmodel/position/psi',          ... % Yaw angle
            'sim/flightmodel/position/phi',          ... % Roll angle
            'sim/flightmodel/position/theta',        ... % Pitch angle
            'sim/flightmodel/position/beta',         ... % Sideslip angle
            'sim/flightmodel/position/true_airspeed',... % UAV speed
            'sim/weather/wind_speed_kt'};                % Wind speed
        
        % Get the data and catch the error if there is a communication problem
        try
            result = getDREFs(DREFS, Socket);
        catch
            error_flag = 0;
            result = zeros(1,9);
            N_ERRORS = N_ERRORS + 1;
        end
        
        % Update aircraft and wind speeds
        UAV_speed(counter) = result(8);
        wind_speed(counter) = result(9);
        
        % Create variables
        UAV_position = [x(counter); y(counter); z(counter)];
        
        % Carrot Chasing Algorithm
        [roll_d, pitch_d, error] = Carrot_Chasing(UAV_position, loiter_position, radius, yaw(counter), lambda, delta, k_psi, UAV_speed(counter));
        
        % REMOVE
        if abs(pitch_d) > pi/8
            pitch_d = sign(pitch_d)*pi/8;
        end
        
        % Compute benchmarks
        Error(counter) = sqrt((error(1) - radius)^2 + error(3)^2);
        partial_error = partial_error + Error(counter)^2;
        error_x(counter) = error(1) - radius;
        error_y(counter) = error(2);
        error_z(counter) = error(3);
        
        % REMOVEEEE
        pitch_desired(counter) = pitch_d;
        
        % Steady state error
        if counter > 400 
            error_steady(counter - 400) = error_x(counter);
        end
        
        % Update aircraft attitude 
        yaw(counter+1) = result(4)*pi/180;    
        roll(counter+1) = result(5)*pi/180; 
        pitch(counter+1) = result(6)*pi/180;
        sideslip(counter+1) = my_wrap_to_pi(result(7)*pi/180);
        
        % Low Level Controllers 
        [input, Controllers] = control_6DOF_horizontal([roll_d, pitch_d], [roll(counter+1), pitch(counter+1)], Va, UAV_speed(counter), sideslip(counter+1), Controllers);
        
        % Computing elevator command according to vertical error
        if abs(error(3)) < 2
            [elevator_cmd(counter)] = limit_deflection(pitch(counter+1), input(3), pi/20);
        else
            [elevator_cmd(counter)] = limit_deflection(pitch(counter+1), input(3), pi/8);
        end
        
        % Computing aileron command according to the roll angle
        [aileron_cmd(counter)] = limit_deflection(roll(counter+1), input(2), pi/5);
        
        % Computing other commands   
        elevator_cmd(counter) =  input(3);
        throttle_cmd(counter) = input(1);
        rudder_cmd(counter) = input(4);
        
        % Send commands to the aircraft
        CTRL = [elevator_cmd(counter), aileron_cmd(counter), rudder_cmd(counter), throttle_cmd(counter), 0, 0];
        sendCTRL(CTRL, 0, Socket)
        pause(0.05);
        
        % Update position
        if counter == 1
            INICIAL_X = -result(1);
            INICIAL_Y = result(2);
            INICIAL_Z = result(3);
            x(counter+1) = 1200;
            y(counter+1) = 0;
            z(counter+1) = 0;
        else
            x(counter+1) = -result(1) - INICIAL_X + 1200;
            y(counter+1) = result(2) - INICIAL_Y;
            z(counter+1) = result(3) - INICIAL_Z;
        end
        
        % Update time variables
        intervalo_real_iter(counter) = toc;
        if (counter + 1) == 2 % Used to avoid a pic in the fist iteration that is cause by noise
            intervalo_real_iter(counter) =  0.0787;
        end
        
        % In case there is an error in direction
        if (counter + 1 == 10 && mean(roll(2:10)) < 0) || (counter <= 20 && abs(roll(counter+1)) > 0.96)
            error_flag = 0;
            N_ERRORS = N_ERRORS + 1;
        end
        
        % Update counter 
        counter = counter + 1;
    end
    
    % End X-Plane connection 
    closeUDP(Socket);
    if error_flag == 1
        sim = 0;
    end
    
end

% Metrics Calculation
vertical_error_avg = mean(abs(error_z));
horizontal_error_avg = mean(abs(error_x));
error_avg = mean(Error);
steady_state_error_avg = mean(abs(error_steady));
total_error_avg = sqrt(partial_error/(counter - 1));
overshoot = max(z);
aileron_cmd_avg = mean(abs(aileron_cmd));
elevator_cmd_avg = mean(abs(elevator_cmd));
rudder_cmd_avg = mean(abs(rudder_cmd));
throttle_cmd_avg = mean(abs(throttle_cmd));
wind_speed_avg = mean(abs(wind_speed));
time_step_avg = mean(intervalo_real_iter);
mean_speed = mean(UAV_speed);

% Error Normalization 
norm_erro_vertical = vertical_error_avg/0.5684;
norm_erro_horizontal = horizontal_error_avg/13.9924;

% Cost Function
f = norm_erro_vertical + norm_erro_horizontal + aileron_cmd_avg + elevator_cmd_avg + rudder_cmd_avg + throttle_cmd_avg;

% Metrics Vector
metric(1) = vertical_error_avg;     % Vertical error
metric(2) = horizontal_error_avg;   % Cross-track error
metric(3) = error_avg; 
metric(4) = steady_state_error_avg; % Steady state error
metric(5) = total_error_avg; 
metric(6) = overshoot;              % Overshoot
metric(7) = aileron_cmd_avg;        % Commanded aileron
metric(8) = elevator_cmd_avg;       % Commanded elevator
metric(9) = rudder_cmd_avg;         % Commanded rudder
metric(10) = throttle_cmd_avg;      % Commanded throttle
metric(11) = wind_speed_avg;        % Wind speed
metric(12) = time_step_avg; 
metric(13) = mean_speed;            % UAV speed
metric(14) = f;                     % Cost

% Graphics 
figure(1)
plot (x, y,'r');
grid on
xlabel 'X [m]'
ylabel 'Y [m]'
legend('Path of Algorithm');
title('Horizontal Path of algorithm (xy)')
 
figure(2)
plot (z(1:1000), 'r');
grid on
xlabel 'Time [s]'
ylabel 'Z [m]'
legend('Path of Algorithm');
title('Height comparison')

% figure(3)
% plot (linspace(1,1000,1000), pitch_desired, 'r', linspace(0,1000,1001), pitch(), 'g');
% grid on
% xlabel 'Iteration'
% ylabel 'Pitch (rad)'
% legend('Desired Pitch', 'Aircraft Pitch');

% Update global variables
METRICS(:,INDIVIDUAL) = metric;
SUM_X = SUM_X + x;
SUM_Y = SUM_Y + y;
SUM_Z = SUM_Z + z;
INDIVIDUAL = INDIVIDUAL + 1;
end