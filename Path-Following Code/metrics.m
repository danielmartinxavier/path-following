function [mean_cost] = metrics(METRICS, N_ERRORS, total_time)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function used to compute and print the metrics used in the comparison of
% path-following algorithms.
%
% INPUT  - METRICS: matrix with variable size which contains all the
%          benchmarks for each simulation. Thus, if X simulations were
%          made, it has X columns and 14 rolls (number of metrics).
%          N_ERRORS: number of communications failures (data loss) in the
%          entire simulation
%          total_time: elapsed time to complete all simulations.
%
% OUTPUT - mean_cost: mean cost over al simulations, representing the
%          average cost of a given algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    vertical_error_avg = mean(METRICS(1,:));
    vertical_error_std = std(METRICS(1,:));
    horizontal_error_avg = mean(METRICS(2,:));
    horizontal_error_std = std(METRICS(2,:));
    error_avg = mean(METRICS(3,:));
    error_std = std(METRICS(3,:));
    steady_state_error_avg = mean(METRICS(4,:));
    steady_state_error_std = std(METRICS(4,:));
    total_error_avg = mean(METRICS(5,:));
    total_error_std = std(METRICS(5,:));
    overshoot = mean(METRICS(6,:));
    overshoot_std = std(METRICS(6,:));
    aileron_cmd_avg = mean(METRICS(7,:));
    aileron_cmd_std = std(METRICS(7,:));
    elevator_cmd_avg = mean(METRICS(8,:));
    elevator_cmd_std = std(METRICS(8,:));
    rudder_cmd_avg = mean(METRICS(9,:));
    rudder_cmd_std = std(METRICS(9,:));
    throttle_cmd_avg = mean(METRICS(10,:));
    throttle_cmd_std = std(METRICS(10,:));
    energy_consumed_avg = (aileron_cmd_avg + elevator_cmd_avg + rudder_cmd_avg + throttle_cmd_avg)/4;
    wind_speed_avg = mean(METRICS(11,:));
    wind_speed_std = std(METRICS(11,:));
    time_step_avg = mean(METRICS(12,:));
    time_step_std = std(METRICS(12,:));
    mean_speed = mean(METRICS(13,:));
    std_speed = std(METRICS(13,:));
    mean_cost = mean(METRICS(14,:));
    std_cost = std(METRICS(14,:));
    
    fprintf('\nNumber of errors: %d \n', N_ERRORS)
    fprintf('\nVertical error: %.4f +- %.2f \n', vertical_error_avg, vertical_error_std)
    fprintf('\nCross-track error: %.4f +- %.1f \n', horizontal_error_avg, horizontal_error_std)
    fprintf('\nMean error: %.4f +- %.1f \n', error_avg, error_std)
    fprintf('\nSteady-state error: %.4f +- %.1f \n', steady_state_error_avg, steady_state_error_std)
    fprintf('\nTotal error: %.4f +- %.1f \n', total_error_avg, total_error_std)
    fprintf('\nOvershoot: %.4f +- %.1f \n', overshoot, overshoot_std)
    fprintf('\nAileron command: %.4f +- %.3f \n', aileron_cmd_avg, aileron_cmd_std)
    fprintf('\nElevator command: %.4f +- %.3f \n', elevator_cmd_avg, elevator_cmd_std)
    fprintf('\nRudder command: %.4f +- %.4f \n', rudder_cmd_avg, rudder_cmd_std)
    fprintf('\nThrottle command: %.4f +- %.3f \n', throttle_cmd_avg, throttle_cmd_std)
    fprintf('\nEnergy consumed: %.4f \n', energy_consumed_avg)
    fprintf('\nWind speed: %.4f +- %.3f \n', wind_speed_avg, wind_speed_std)
    fprintf('\nUAV speed: %.4f +- %.1f \n', mean_speed, std_speed)
    fprintf('\nAverage time per iteration: %.4f +- %.4f \n', time_step_avg, time_step_std)
    fprintf('\nMean cost: %.4f +- %.2f \n', mean_cost, std_cost)
    fprintf('\nElapsed time: %s \n', total_time)
end