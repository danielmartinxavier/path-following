function [roll_d, pitch_d, error] = Vector_Field(UAV_position, loiter_position, radius, yaw, pitch, chi_app, tau, gamma, rho, xi, epsilon, lambda, k_psi, k_th, k_t, speed, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the Vector Field algorithm for 3D loiter paths
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
%          Vector Field parameters:
%               * chi_app
%               * tau
%               * gamma
%               * rho
%               * xi
%               * epsilon
%               * lambda
%               * k_psi
%               * k_th
%               * k_t
%          speed: measured speed of the UAV
%          dt: time between 2 consecutive iterations
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
dir = sign(error(1) - radius);

% Compute the desired yaw rate
yaw_d = my_wrap_to_pi(alpha - pi/2 - dir*pi/3*(abs(error(1) - radius)/radius)^rho);
yaw_c = my_wrap_to_pi(yaw_d + (speed/(epsilon*error(1)))*sin(yaw - alpha));
rd = k_psi*my_wrap_to_pi(yaw_c - yaw)*speed;

% Compute the desired roll
roll_d = atan2(rd*speed,9.806);

% Compute the desired pitch rate   
chi_d = my_wrap_to_pi(chi_app * (2/pi) * atan(xi *(-error(3))));
CHI_til = my_wrap_to_pi(chi_d - pitch);
sat = CHI_til/tau;
if abs(sat) > 1
   sat = sign(sat);
end
CHI_control = my_wrap_to_pi(pitch - ((1/gamma)*chi_app*(2/pi)*(xi/(1+(xi*(-error(3)))^2))*speed*sin(pitch)) ...
    - (lambda/gamma)*sat);
pitch_c = gamma*my_wrap_to_pi(CHI_control + pitch);
q_d = -k_th*my_wrap_to_pi(pitch_c + pitch)*speed;

% Compute the desired pitch
pitch_d = k_t*my_wrap_to_pi(pitch + q_d*dt);
end