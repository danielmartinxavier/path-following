function [input, Controllers] = control_6DOF_horizontal(angles_desired, angles, vel_desired, vel, beta, Controllers)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control of attitude and altitude for coordinate turns. 
%
% INPUT  - angles_desired: vector of size 2 containing:
%               * angles_desired(1): desired roll angle  - roll_d
%               * angles_desired(2): desired pitch angle - pitch_d
%          angles: vector of size 2 containing:
%               * angles(1): meausred roll angle  - roll
%               * angles(2): meausred pitch angle - pitch
%          vel_desired: desired speed of the UAV
%          vel: measured speed of the UAV
%          beta: measured sideslip
%          Controllers: vector containing PID parameters defined by
%          function 'set_controllers_horizontal'.
%
% OUTPUT - input: vector of size 4 containing the commands computed by Low
%          Level Controllers for:
%               * input(1): throttle - to control speed
%               * input(2): ailerons - to control roll
%               * input(3): elevator - to control pitch
%               * input(4): rudder   - to control yaw
%          Controllers: vector containing the following information:
%               * Controllers.<name>.P: value of proportional gain
%               * Controllers.<name>.I: value of integral gain
%               * Controllers.<name>.D: value of derivative gain
%               * Controllers.<name>.Old_Error: error between reference and
%               desired value
%               * Controllers.<name>.Old_Output: reference value
%               * Controllers.<name>.Old_Input: command
%
% NOTE  - More information can be found on Principles of Guidance, Navigation 
%         and Control of UAVs of Elkain et al. and in the Chapter 7, Section
%         The Steady Turn of Dynamics of Flight of Etkins.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    input = zeros(4,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Pitch PID with system output pitch and input elevator
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Pitch,input(3)] = PID_attitude_step(angles_desired(2), angles(2), Controllers.PID_Pitch);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Roll PID with system output roll and input aileron
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Roll,input(2)] = PID_attitude_step(angles_desired(1), angles(1), Controllers.PID_Roll);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Yaw PID with system output yaw and input rudder
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Beta,input(4)] = PID_attitude_step(0, beta, Controllers.PID_Beta);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PID for airspeed hold mode - y = velocity in m/s and u = throttle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [Controllers.PID_Vel,input(1)] = PID_step(vel_desired, vel, Controllers.PID_Vel);
    
    
end
