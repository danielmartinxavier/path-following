%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN script of the simulation. It's used to launch the simulation of 
% path-following and choose the startegy and wind intensity. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all;
%% Import XPC
javaaddpath([pwd '/XPlaneConnect/']);
addpath([pwd '/Functions/'])
import XPlaneConnect.*

%% Definition of simulation parameters

initial_time = datetime;
fprintf('\nSimulation starts at: %s \n',initial_time)

% Initialize global variables
global INDIVIDUAL
global METRICS
global N_ERRORS
global SUM_X
global SUM_Y
global SUM_Z
INDIVIDUAL = 1;
N_ERRORS = 1;
SUM_X = zeros(1, 1001);
SUM_Y = zeros(1, 1001);
SUM_Z = zeros(1, 1001);

%% Choice of path-following algorithm parameters

% Choice of algorithm. Options are: "Carrot-Chasing", "NLGL", "PLOS", "Vector"
algorithm = "Vector"; 
fprintf('\nPath-Following algorithm: %s \n', algorithm)

% Choice of wind speed. Options are: "None", "Intermediate", "Hard"
wind = "Hard"; 
fprintf('\nWind speed: %s \n', wind)

%% Definition of the algorithm parameters

[parameters] = parameters(algorithm, wind);
% Carrot Chasing Parameters
if algorithm == "Carrot-Chasing"
    lambda    = parameters(1);
    delta     = parameters(2);
    k_psi     = parameters(3);
% NLGL+ Parameters
elseif algorithm == "NLGL"       
    L_psi     = parameters(1);
    L_th      = parameters(2);
    k_psi     = parameters(3);
    k_th      = parameters(4);
    k_t       = parameters(5);
% PLOS+ Parameters
elseif algorithm == "PLOS"   
    k_pp_psi  = parameters(1); 
    k_pp_th   = parameters(2); 
    k_plo_psi = parameters(3); 
    k_plo_th  = parameters(4); 
    k_t       = parameters(5); 
% Vector Field Parameters
elseif algorithm == "Vector" 
    chi_app   = parameters(1);
    tau       = parameters(2);
    gamma     = parameters(3);
    rho       = parameters(4);
    xi        = parameters(5);
    epsilon   = parameters(6);
    lambda    = parameters(7); 
    k_psi     = parameters(8);
    k_th      = parameters(9);
    k_t       = parameters(10); 
end
%% Simulation

i = 1;
numb_sim = 10;
fprintf('\nSimulation completed: ')
while i <= numb_sim
    
    % Carrot-Chasing Simulation
    if algorithm == "Carrot-Chasing"
        [f] = X_Plane_Carrot(lambda, delta, k_psi);
    % NLGL+ Simulation
    elseif algorithm == "NLGL" 
        [f] = X_Plane_NLGL(L_psi, L_th, k_psi, k_th, k_t);
    % PLOS+ Simulation
    elseif algorithm == "PLOS" 
        [f] = X_Plane_PLOS(k_pp_psi, k_pp_th, k_plo_psi, k_plo_th, k_t);
    % Vector Field Simulation
    elseif algorithm == "Vector"
        [f] = X_Plane_Vector(chi_app, tau, gamma, rho, xi, epsilon, lambda, k_psi, k_th, k_t);
    end
    fprintf(' %d, ',100*i/numb_sim)
    i = i + 1;
end 

fprintf(' \n ')
save('metricas.txt','METRICS','-ascii')
final_time = datetime;
total_time = char(between(initial_time, final_time));

%% Results

[mean_cost] = metrics(METRICS, N_ERRORS, total_time);

x_graf = SUM_X/numb_sim;
y_graf = SUM_Y/numb_sim;
z_graf = SUM_Z/numb_sim;
save('xyz.txt','x_graf','y_graf','z_graf','-ascii')

%% Graphics 
 
% Creating the trajectory
Trajectory = [600 600 20 600]; % Coordinates: (x,y,z,r)
k = 1; 
th = [0:0.01:2*pi];
th_num = size(th,2);
path_xc(:) = Trajectory(4)*cos(th)+Trajectory(1);
path_yc(:) = Trajectory(4)*sin(th)+Trajectory(2);
while k <= th_num
    path_zc(k) = Trajectory(3);
    k = k + 1;
end

%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%

figure(1)
plot3 (path_xc, path_yc, path_zc,'g', x_graf, y_graf, z_graf,'r');
grid on
xlabel 'X [m]'
ylabel 'Y [m]'
zlabel 'Z [m]'
legend('Desired Path','Path of Algorithm');
title('3D Path of algorithm')

figure(2)
plot (path_xc, path_yc,'g', x_graf, y_graf,'r');
grid on
xlabel 'X [m]'
ylabel 'Y [m]'
legend('Desired Path','Path of Algorithm');
title('Horizontal Path of algorithm (xy)')

desired_height = 20*ones(1,1000);
time = linspace(1, 1000, 1000);
 
figure(3)
plot (time,desired_height,'g',time, z_graf(1:1000), 'r');
grid on
xlabel 'Time [s]'
ylabel 'Z [m]'
legend('Desired Path','Path of Algorithm');
title('Height comparison')