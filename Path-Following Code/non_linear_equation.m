function [angular_speed] = non_linear_equation(UAV_position, virtual_target, reference_angle, k, speed, L, flag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function used to compute angular speed (rd, qd) using the non-linear
% guidance law for the NLGL algorithm.
%
% INPUT  - UAV_position: vector of size 2 containing the UAV position
%               * UAV_position(1): X coordinate of the UAV position
%               * UAV_position(2): Y coordinate of the UAV position
%          virtual_target: position of the virtual target found by the 
%          intersection between the path and a virtual circle.
%          reference_angle: yaw or pitch angles used as a reference for the
%          non-linear guidance law to compute the desired roll and pitch 
%          angles, respectively.
%          k: gain used to control the intensity of angular speed.
%          speed: measured UAV speed.
%          L: radius of the virtual circle.
%          flag: used to know whereas the function is been used to compute
%          the desired roll ("Horizontal") or pitch ("Vertical") angles.
%
% OUTPUT - angular_speed: rd or qd computed using the non-linear guidance 
%          law.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
if flag == "Horizontal"
    sigma = atan2(virtual_target(2) - UAV_position(2), virtual_target(1) - UAV_position(1));
elseif flag == "Vertical"
    sigma = atan2(virtual_target(2) - UAV_position(3), virtual_target(1) - UAV_position(1));
end
mu = my_wrap_to_pi(sigma - reference_angle);
angular_speed = (2 * k* (speed^2) * sin(mu))/L;
end