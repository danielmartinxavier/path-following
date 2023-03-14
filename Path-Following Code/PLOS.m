function [roll_d, pitch_d, error] = PLOS(UAV_position, loiter_position, radius, yaw, pitch, k_pp_psi, k_pp_th, k_plo_psi, k_plo_th, k_t, speed, dt, PLOS_counter)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the PLOS algorithm for 3D loiter paths
%
% INPUT  - UAV_position: vector of size 2 containing the UAV position
%               * UAV_position(1): X coordinate of the UAV position
%               * UAV_position(2): Y coordinate of the UAV position
%          loiter_position: vector of size 2 containing the loiter position
%               * loiter_position(1): X coordinate of the loiter centre
%               * loiter_position(2): Y coordinate of the loiter centre
%          radius: radius of the loiter path
%          yaw: measured yaw angle of the UAV
%          pitch: measured yaw angle of the UAV
%          PLOS parameters:
%               * k_pp_psi
%               * k_pp_th
%               * k_plo_psi
%               * k_plo_th
%               * k_t
%          speed: measured speed of the UAV
%          dt: time between 2 consecutive iterations
%          PLOS counter: used in the Gain Scheduling strategy
%
% OUTPUT - roll_d: desired roll angle for the UAV to converge towards the 
%          desired path
%          pitch_d: desired pitch angle for the UAV to converge towards the 
%          desired path
%          error: difference between the current UAV position and the path, 
%          which is computed by one rotation to obtain a vector in the path
%          frame: 
%               * error(1): horizontal error
%               * error(2): is always equal to 0
%               * error(3): vertical error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    alpha = atan2((UAV_position(2) - loiter_position(2)), (UAV_position(1) - loiter_position(1)));
    alpha_p = (my_wrap_to_pi(alpha + pi/2));
    Rz = [cos(alpha) , sin(alpha), 0; 
          -sin(alpha), cos(alpha), 0;
              0     ,     0    , 1];
    
    % Rotation to obtain the error in the path frame
    error = Rz*(UAV_position - loiter_position);
    
    % Desired yaw rate - Gain Scheduling strategy
    if PLOS_counter > 0 && PLOS_counter <= 200 % counter > 200 && counter <= 400
        rd = (k_pp_psi*my_wrap_to_pi(alpha_p - yaw) + (1 + PLOS_counter*0.015)*k_plo_psi*((abs(error(1))) - radius))*speed;
    elseif PLOS_counter > 200 % counter > 400
        rd = (k_pp_psi*my_wrap_to_pi(alpha_p - yaw) + 4*k_plo_psi*((abs(error(1))) - radius))*speed;
    else  % counter <= 200
        rd = (k_pp_psi*my_wrap_to_pi(alpha_p - yaw) + k_plo_psi*((abs(error(1))) - radius))*speed;
    end
    
    % Desired roll
    roll_d = atan2(rd*speed,9.806);
    
    % Desired pitch rate
    qd = -(k_pp_th*my_wrap_to_pi(-pitch) + k_plo_th*(error(3)))*speed;    
    
    % Desired pitch
    pitch_d = my_wrap_to_pi(k_t*(pitch + qd*dt)); % Mudei
end
