function [roll_d, pitch_d, error] = Carrot_Chasing(UAV_position, loiter_position, radius, yaw, lambda, delta, k_psi, speed)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the Carrot-Chasing algorithm for 3D loiter paths
%
% INPUT  - UAV_position: vector of size 2 containing the UAV position
%               * UAV_position(1): X coordinate of the UAV position
%               * UAV_position(2): Y coordinate of the UAV position
%          loiter_position: vector of size 2 containing the loiter position
%               * loiter_position(1): X coordinate of the loiter centre
%               * loiter_position(2): Y coordinate of the loiter centre
%          radius: radius of the loiter path
%          yaw: measured yaw angle of the UAV
%          Carrot-Chasing parameters:
%               * lambda 
%               * delta
%               * k_psi
%          speed: measured speed of the UAV
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
Rz = [cos(alpha)  , sin(alpha), 0; 
      -sin(alpha) , cos(alpha), 0;
            0     ,      0    , 1];
  
% Rotation to obtain the error in the path frame
error = Rz*(UAV_position - loiter_position);

% Compute virtual targets
x_linha = (radius*cos(alpha + lambda)) + loiter_position(1);
y_linha = (radius*sin(alpha + lambda)) + loiter_position(2);
yaw_d = atan2((y_linha - UAV_position(2)), (x_linha - UAV_position(1)));

% Desired yaw rate
rd = k_psi*my_wrap_to_pi(yaw_d - yaw)*speed;

% Desired roll
roll_d = atan2(rd*speed,9.806);

% Desired pitch - Lookahead Approach
pitch_d = -my_wrap_to_pi(atan(error(3)/delta));
end