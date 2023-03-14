function [R1, R2, delta] = Intersection_RC(L, Center, Waypoint1, Waypoint2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function that finds the intersection between a circle and a line. It's used
% by the NLGL algorithm to find the its virtual targets.
%
% INPUT  - L: radius of the virtual circle used in the NLGL strategy
%          loiter centre:
%               * Loiter_center(1): X coordinate of the loiter centre
%               * Loiter_center(2): Y coordinate of the loiter centre
%          Loiter_radius: radius of the loiter path
%          Center: vector of size 2 containing the UAV position
%               * Center(1): X coordinate of the UAV position
%               * Center(2): Y coordinate of the UAV position
%          Waypoint1: vector of size 2 containing the position of Waypoint
%          1, used to define the line:
%               * Waypoint1(1): X coordinate of Waypoint1
%               * Waypoint1(2): Y coordinate of Waypoint1
%          Waypoint2: vector of size 2 containing the position of Waypoint
%          2, used to define the line:
%               * Waypoint2(1): X coordinate of Waypoint2
%               * Waypoint2(2): Y coordinate of Waypoint2
%
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
    
    % Waypoint translation
    N1 = Waypoint1 - Center;
    N2 = Waypoint2 - Center;

    % Compute intersection
    Q = N1;
    V = N2 - N1;

    a = V * V';
    b = 2 * Q * V';
    c = (Q * Q') - L^2;

    D = b^2 - 4*a*c;
    if D < 0
        delta = -1; % No intersection between circles
    elseif D == 0
        delta = 0;  % Intersection in only one point
    elseif D > 0
        delta = 1;  % Intersection in two points
    end
  
    t(1) = (-b + sqrt(D)) / (2*a);
    t(2) = (-b - sqrt(D)) / (2*a);

    t1_ans = max(t);
    t2_ans = min(t);
    
    % Return the virtual targets
    R1 = Waypoint1 + t1_ans*V;
    R2 = Waypoint1 + t2_ans*V;
    
end