function [parameters] = parameters(algorithm, wind)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function that chooses the appropriate parameters based on the algorithm
% and wind intensity chosen for the simulation.
%
% INPUT  - algorithm: string that selects which path-following algorithm
%          should be launched, presenting the following options:
%               * "Carrot-Chasing"
%               * "NLGL"
%               * "PLOS"
%               * "Vector"
%          wind: string that selects which wind intensity should be used,
%          presenting the following options:
%               * "None" for no wind
%               * "Intermediate" for 5 m/s wind
%               * "Hard" for 10 m/s wind
%
% OUTPUT - parameters: path-following algorithm parameters chosen according
%          to the algorithm and wind intensity sent as input.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%% Carrot Chasing Parameters %%%%%%%%%%%%%%%%%%%%%%%%%
if algorithm == "Carrot-Chasing"
    if wind == "None"                   % No wind
        parameters(1) = 0.2450;         % lambda
        parameters(2) = 21.5;           % delta
        parameters(3) = 1.3125*10^(-2); % k_psi
    elseif wind == "Intermediate"       % 5 m/s wind
        parameters(1) = 0.2201;         % lambda
        parameters(2) = 16.554;         % delta
        parameters(3) = 1.4375*10^(-2); % k_psi
    elseif wind == "Hard"               % 10 m/s wind
        parameters(1) = 0.2288;         % lambda
        parameters(2) = 24.6875;        % delta
        parameters(3) = 2.6926*10^(-2); % k_psi
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%% NLGL Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif algorithm == "NLGL" 
    if wind == "None"                % No wind
        parameters(1) = 170.0;       % L_psi
        parameters(2) = 30.0;        % L_th
        parameters(3) = 0.01968;     % k_psi
        parameters(4) = 0.07845;     % k_th
        parameters(5) = 6.6*10^(-1); % k_t
    elseif wind == "Intermediate"    % 5 m/s wind
        parameters(1) = 159.375;     % L_psi
        parameters(2) = 30.3594;     % L_th
        parameters(3) = 0.0246;      % k_psi
        parameters(4) = 0.08745;     % k_th
        parameters(5) = 0.5306;      % k_t
    elseif wind == "Hard"            % 10 m/s wind
        parameters(1) = 153.642;     % L_psi
        parameters(2) = 30.5357;     % L_th
        parameters(3) = 0.026082;    % k_psi
        parameters(4) = 0.090745;    % k_th
        parameters(5) = 0.6861;      % k_t
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOS Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif algorithm == "PLOS"
    if wind == "None"                   % No wind
        parameters(1) = 4.167*10^(-2);  % k_pp_psi
        parameters(2) = 1.089*10^(-2);  % k_pp_th
        parameters(3) = 1.961*10^(-4);  % k_plo_psi
        parameters(4) = 2.533*10^(-2);  % k_plo_th
        parameters(5) = 0.268;          % k_t
    elseif wind == "Intermediate"       % 5 m/s wind
        parameters(1) = 3.7704*10^(-2); % k_pp_psi
        parameters(2) = 1.7841*10^(-2); % k_pp_th
        parameters(3) = 1.9838*10^(-4); % k_plo_psi
        parameters(4) = 2.5453*10^(-2); % k_plo_th
        parameters(5) = 0.3121;         % k_t
    elseif wind == "Hard"               % 10 m/s wind
        parameters(1) = 3.5798*10^(-2); % k_pp_psi
        parameters(2) = 1.194*10^(-2);  % k_pp_th
        parameters(3) = 2.0664*10^(-4); % k_plo_psi
        parameters(4) = 2.9896*10^(-2); % k_plo_th
        parameters(5) = 0.2513;         % k_t
    end
    
%%%%%%%%%%%%%%%%%%%%% Vector Field Parameters %%%%%%%%%%%%%%%%%%%%%%%%%
elseif algorithm == "Vector" 
    if wind == "None"             % No wind
        parameters(1) = 1.0674;   % chi_app
        parameters(2) = 0.4407;   % tau
        parameters(3) = 2.0034;   % gamma
        parameters(4) = 0.73;     % rho
        parameters(5) = 0.105;    % xi
        parameters(6) = 1.3635;   % epsilon
        parameters(7) = 1.635;    % lambda
        parameters(8) = 0.0272;   % k_psi
        parameters(9) = 0.1214;   % k_th
        parameters(10) = 0.4487;  % k_t
    elseif wind == "Intermediate" % 5 m/s wind
        parameters(1) = 1.1861;   % chi_app
        parameters(2) = 0.6325;   % tau
        parameters(3) = 2.4868;   % gamma
        parameters(4) = 0.5363;   % rho
        parameters(5) = 0.1175;   % xi
        parameters(6) = 2.3510;   % epsilon
        parameters(7) = 1.9475;   % lambda
        parameters(8) = 0.0259;   % k_psi
        parameters(9) = 0.1408;   % k_th
        parameters(10) = 0.4937;  % k_t
    elseif wind == "Hard"         % 10 m/s wind
        parameters(1) = 1.2252;   % chi_app
        parameters(2) = 0.5917;   % tau
        parameters(3) = 2.4948;   % gamma
        parameters(4) = 0.5363;   % rho
        parameters(5) = 0.1175;   % xi
        parameters(6) = 2.4510;   % epsilon
        parameters(7) = 1.8227;   % lambda
        parameters(8) = 0.0159;   % k_psi
        parameters(9) = 0.1458;   % k_th
        parameters(10) = 0.4626;  % k_t
    end
end
end