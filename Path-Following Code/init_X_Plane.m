function [Socket, error_flag, population_error, initial_pitch, initial_yaw, Va_knots] = init_X_Plane(population_error)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function used to initialize X-Plane with initial conditions.
%
% INPUT  - population_error: variable that informs the number of times a 
%          connection was tried to be stablished but an error occured.
%
% OUTPUT - Socket: variable connected to the UDP used to stablish the
%          connection.
%          error_flag: it's 1 when there are no errors and 0 if the 
%          connection fails to be stablished.
%          population_error: variable that informs the number of times a 
%          connection was tried to be stablished but an error occured. It
%          is incremented every time an error occurs.
%          initial_pitch: initial pitch angle of the aircraft.
%          initial_yaw: initial yaw angle of the aircarft.
%          Va_knots: initial speed of the aircarft in knots.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % Declare global variables and create flags
    global N_ERRORS 
    error_flag = 1;
    
    % Stablish the connection between MATLAB and X-Plane 
    Socket = openUDP('127.0.0.1', 49009);
    
    % Assure connection and catch eventual errors
    try
        getDREFs('sim/test/test_float', Socket);
    catch
        error_flag = 0;
        N_ERRORS = N_ERRORS + 1;
        population_error = population_error + 1;
    end
    
    %% Initial conditions for a stabilised cruise flight
    
    % Initial Position
    pos_lat = 47.46;   % In degrees
    pos_long = -122.3; % In degrees
    altitude_m = 1000; % In meters
    
    % Initial Attitude
    initial_pitch = 2; % In degrees
    initial_roll = 0;  % In degrees
    initial_yaw = 0;   % In degrees
    initial_alpha = 2; % Atack angle (degrees)
    initial_beta = 0;  % Sideslip angle (degrees)
    
    % Initial Speed
    Va_knots = 100;    % Aircraft speed (knots)
    
    % Initial forces acting in the aircraft
    lift_force = 0;
    drag_force = 0;
    engine_force = 0;
    
    % Pause simulation while everything is being initialized
    pauseSim(1,Socket);
    
    %% Aircraft initial position
    
    %         Lat      Lon        Alt           Pitch          Roll       Heading    Gear
    POSI = [pos_lat, pos_long, altitude_m, initial_pitch, initial_roll, initial_yaw,  0  ];
    sendPOSI(POSI, 0, Socket);
    
    pause(1.5);
    
    %% Setting the aircraft initial conditions

    %                     Alpha       ,    Velocity  ,  PQR    ,  Moments , Aero ,Forces, Engine Forces
    data = struct('h',[    18         ,       3      ,   16    ,    15    ,  64  ,        65       ], ...
                  'd',[ initial_alpha , initial_beta ,    0    ,   -999   , -999 , -999 , -999, -999; ...  % Alpha data and beta
                         Va_knots     ,   Va_knots   , Va_knots, Va_knots , -999 , -999 , -999, -999; ...  % Velocity data
                            0         ,       0      ,    0    ,  -999    , -999 , -999 , -999, -999; ...  % PQR data
                            0         ,       0      ,    0    ,  -999    , -999 , -999 , -999, -999; ...  % LMN data
                        lift_force    ,  drag_force  ,    0    ,  -999    , -999 , -999 , -999, -999; ...  % Aero forces
                       engine_force   ,       0      ,    0    ,  -999    , -999 , -999 , -999, -999;]);   % Engine forces
    
    sendDATA(data, Socket);
    pause(1.5);

end