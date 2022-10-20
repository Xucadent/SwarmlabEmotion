%% 初始化
clc
clear
close all;

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename));
addpath(genpath(project_root));
load('./parameters/param_scenario.mat');

%% 启动仿真
Current_Scenario.Run_Scenario();
