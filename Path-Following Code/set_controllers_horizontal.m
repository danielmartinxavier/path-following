function Controllers = set_controllers_horizontal(dt, angle_ini, airspeed, max_control, desired_angle_ini)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set controllers gains for the horizontal flight mode.
%
% INPUT  - dt: time step used by integral and derivative gains
%          angle_ini: vector of size 2 containing:
%               * angle_ini(1): initial roll angle 
%               * angle_ini(2): initial pitch angle 
%          airspeed: initial speed of the UAV 
%          max_control: vector of sieze 4 containig the maximum command 
%          values for each control surface (rudder, ailerons, elevator) and
%          the throttle
%          desired_angle_ini: measured sideslip
%               * desired_angle_ini(1): initial desired roll angle 
%               * desired_angle_ini(2): initial desired pitch angle 
%
% OUTPUT - Controllers: vector containing the following information:
%               * Controllers.<name>.Dt: time step used by integral and
%               derivative gains
%               * Controllers.<name>.Kp: value of proportional gain
%               * Controllers.<name>.Ki: value of integral gain
%               * Controllers.<name>.Kd: value of derivative gain
%               * Controllers.<name>.Old_Error: error between reference and
%               desired value
%               * Controllers.<name>.Old_Output: reference value
%               * Controllers.<name>.Min_Input: minimal command
%               * Controllers.<name>.Max_Input: maximal command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Controllers.function = @control_6DOF_horizontal;
    
     % Pitch PID with system output pitch and input dpitch
    Controllers.PID_Pitch.Dt = dt;
    Controllers.PID_Pitch.Kp = 3.5;
    Controllers.PID_Pitch.Ki = 3;
    Controllers.PID_Pitch.Kd = 0.2;
    Controllers.PID_Pitch.Tf = 0;
    Controllers.PID_Pitch.P = 0;
    Controllers.PID_Pitch.D = 0;
    Controllers.PID_Pitch.I = 0;
    Controllers.PID_Pitch.Old_Error = angdiff(angle_ini(2),desired_angle_ini(2));
    Controllers.PID_Pitch.Old_Output = angle_ini(2);
    Controllers.PID_Pitch.Old_Input = 0;
    Controllers.PID_Pitch.Min_Input = -max_control(3);
    Controllers.PID_Pitch.Max_Input = max_control(3);

    % Roll PID with system output roll and input droll
    Controllers.PID_Roll.Dt = dt;
    Controllers.PID_Roll.Kp = 3; 
    Controllers.PID_Roll.Ki = 0;
    Controllers.PID_Roll.Kd = 0.1; 
    Controllers.PID_Roll.Tf = 0;
    Controllers.PID_Roll.P = 0;
    Controllers.PID_Roll.D = 0;
    Controllers.PID_Roll.I = 0;
    Controllers.PID_Roll.Old_Error = angdiff(angle_ini(1),desired_angle_ini(1));
    Controllers.PID_Roll.Old_Output = angle_ini(1);
    Controllers.PID_Roll.Old_Input = 0;
    Controllers.PID_Roll.Min_Input = -max_control(2);
    Controllers.PID_Roll.Max_Input = max_control(2);

    % Yaw PID with system output beta and input dbeta
    Controllers.PID_Beta.Dt = dt;
    Controllers.PID_Beta.Kp = 5;
    Controllers.PID_Beta.Ki = 2;
    Controllers.PID_Beta.Kd = 0.1;
    Controllers.PID_Beta.Tf = 0;
    Controllers.PID_Beta.P = 0;
    Controllers.PID_Beta.D = 0;
    Controllers.PID_Beta.I = 0;
    Controllers.PID_Beta.Old_Error = 0;
    Controllers.PID_Beta.Old_Output = 0;
    Controllers.PID_Beta.Old_Input = 0;
    Controllers.PID_Beta.Min_Input = -max_control(4);
    Controllers.PID_Beta.Max_Input = max_control(4);

    % Modified values for the vTrue PID (controlled by throttle)
    Controllers.PID_Vel.Dt = dt;
    Controllers.PID_Vel.Kp = 1;
    Controllers.PID_Vel.Ki = 0;
    Controllers.PID_Vel.Kd = 0;
    Controllers.PID_Vel.Tf = 0;
    Controllers.PID_Vel.D = 0;
    Controllers.PID_Vel.I = 0;
    Controllers.PID_Vel.Old_Error = 0;
    Controllers.PID_Vel.Old_Output = airspeed; 
    Controllers.PID_Vel.Old_Input = 0;
    Controllers.PID_Vel.Min_Input = 0;
    Controllers.PID_Vel.Max_Input = max_control(1);
   
end
 
