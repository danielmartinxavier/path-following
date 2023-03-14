function [roll_d, pitch_d, error] = NLGL(UAV_position, loiter_position, radius, yaw, pitch, L_psi, L_th, k_psi, k_th, k_t, speed, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the NLGL algorithm for 3D loiter paths.
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
%          NLGL parameters:
%               * L_psi
%               * L_th
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

% Compute intersection between circle of radius L_psi and the loiter centre
[VTP1, VTP2, delta] = Intersection_CC([loiter_position(1), loiter_position(2)], radius, [UAV_position(1), UAV_position(2)], L_psi);

% If no intersection is found, cross-track error is used instead of L_psi
if (delta == -1)
     [VTP1, VTP2, delta] = Intersection_CC([loiter_position(1), loiter_position(2)], radius, [UAV_position(1), UAV_position(2)], abs(error(1)) - radius + 1);
     % If no intersection is found again, roll desired is set to zero
     if (delta == -1)
        roll_d = 0;
     else
     % VTP1 or VTP2 is chosen according to the direction the path is
     % followed
        if (UAV_position(2) > loiter_position(2))
            [rd] = non_linear_equation(UAV_position, VTP2, yaw, k_psi, speed, L_psi, "Horizontal");
        else
            [rd] = non_linear_equation(UAV_position, VTP1, yaw, k_psi, speed, L_psi, "Horizontal");
        end
        roll_d = atan2(rd*speed,9.806);
     end
else
   if (UAV_position(2) > loiter_position(2))
        [rd] = non_linear_equation(UAV_position, VTP2, yaw, k_psi, speed, L_psi, "Horizontal");
   else
        [rd] = non_linear_equation(UAV_position, VTP1, yaw, k_psi, speed, L_psi, "Horizontal");
   end
   roll_d = atan2(rd*speed,9.806);
end

% Compute intersection between circle of radius L_th and the loiter centre
[VTP1, VTP2, delta] = Intersection_RC(L_th, [UAV_position(1), UAV_position(3)], [loiter_position(1) - radius, loiter_position(3)], [loiter_position(1) + radius, loiter_position(3)]);

% If no intersection is found, vertical error is used instead of L_th
if (delta == -1)
    [VTP1, VTP2, delta] = Intersection_RC(abs(error(3)) + 1, [UAV_position(1), UAV_position(3)], [loiter_position(1) - radius, loiter_position(3)], [loiter_position(1) + radius, loiter_position(3)]);
     % If no intersection is found again, pitch desired is set to pitch
    if (delta == -1)
        pitch_d = k_t*my_wrap_to_pi(pitch);
    else
        [qd] = non_linear_equation(UAV_position, VTP1, pitch, k_th, speed, L_th, "Vertical");
        pitch_d = k_t*my_wrap_to_pi(pitch + qd*dt); 
    end
else
    [qd] = non_linear_equation(UAV_position, VTP1, pitch, k_th, speed, L_th, "Vertical");
    pitch_d = k_t*my_wrap_to_pi(pitch + qd*dt);
end
end