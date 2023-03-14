function [Atitude] = limit_deflection(angle, input, limit)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function used to limit the aircart deflection by applying smaller commands
%
% INPUT  - angle: measured angle of the UAV (pitch, roll or yaw)
%          input: command computed by Low Level Controllers
%          limit: maximum abolute value allowed for the angle
%
% OUTPUT - Atitude: new command that should be applied to a certain control 
%          surface to limit the aircrat deflection 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    if angle > limit && input > 0
        Atitude = 0;
    elseif angle < -limit && input < 0
        Atitude = 0;
    else
        Atitude = input;
    end
end