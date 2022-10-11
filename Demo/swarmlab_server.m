%% 初始化
clc
clear
close all;

addpath('./statistics');
adj_param =  Multi_scenario();

%% 循环调参仿真

for curr_scenerio = 1 : adj_param.scenario_num
    tic;
    run('example_vasarhelyi.m');
    adj_param = Update_Result(adj_param, swarm_statistics.stat_collisions(1),...
                 swarm_statistics.stat_collisions(2), swarm_statistics.stat_time(2)-swarm_statistics.stat_time(1));
    simu_time = toc;
    fprintf("scenario: %d collision: %d, %d sim_time: %.3f\n", curr_scenerio, swarm_statistics.stat_collisions(1),...
    swarm_statistics.stat_collisions(2), simu_time);
end

csvwrite('result.csv',adj_param.scenario_result);