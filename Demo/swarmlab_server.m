%% 初始化
clc
clear
close all;

addpath(genpath('.'));
load('param_scenario.mat');


%% 启动仿真
Current_Scenario.Run_Scenario();
