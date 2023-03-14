function [R1, R2, delta] = Intersection_CC(Loiter_center, Loiter_radius, UAV_position, Virtual_radius)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function that finds the intersection between 2 circles, being used by the 
% NLGL algorithm to compute virtual targets. 
%
% INPUT  - Loiter_center: vector of size 2 containing the position of the
%          loiter centre:
%               * Loiter_center(1): X coordinate of the loiter centre
%               * Loiter_center(2): Y coordinate of the loiter centre
%          Loiter_radius: radius of the loiter path
%          UAV_position: vector of size 2 containing the UAV position
%               * UAV_position(1): X coordinate of the UAV position
%               * UAV_position(2): Y coordinate of the UAV position
%          Virtual_radius: radius of the virtual circle used in the NLGL
%          strategy
% OUTPUT - R1: vector of size 2 constaining the coordinates of the first
%          virtual target:
%               * R1(1): X coordinate of the first virtual target
%               * R1(2): Y coordinate of the first virtual target
%          R2: vector of size 2 constaining the coordinates of the second
%          virtual target:
%               * R2(1): X coordinate of the second virtual target
%               * R2(2): Y coordinate of the second virtual target
%          delta: value of which informs the intersection status;
%               * delta = -1: there is no intersection between circles
%               * delta = 0: there is only one point of intersection
%               * delta > 0: there are 2 points of intersection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    New_center = Loiter_center - UAV_position;
    a = New_center(1);
    b = New_center(2);
    c = (Loiter_radius^2 - Virtual_radius^2 - ( New_center * New_center')) / 2;
    
    X = [0 Virtual_radius];
    Y = (-a*X - c)/b;
    
    % Check if the intersection exists
    if Y(1) == Inf % No intersection
        X = -c/a;
        [R1, R2, delta] = Intersection_RC(Virtual_radius, [0, 0], [X, 0], [X, 1]);
    else
        [R1, R2, delta] = Intersection_RC(Virtual_radius, [0, 0], [X(1), Y(1)], [X(2), Y(2)]);
    end
    
    % Return the virtual targets
    R1 = R1 + UAV_position;
    R2 = R2 + UAV_position;
end