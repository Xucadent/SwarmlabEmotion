% 读取Scenario_Config.csv文件

config = importdata('./Scenario_Config.csv');

scenario_num = size(config.textdata, 1) - 2;
enable_num = 3;

% 读取文本部分
scenario_id = config.textdata(3 : scenario_num + 2, 1);
scenario_id = str2num(char(scenario_id));
scenario_en = config.textdata(3 : scenario_num + 2, 2:enable_num + 1);
temp = zeros(scenario_num, enable_num);
for i = 1 : enable_num
    temp(:,i) = sum(char(scenario_en(:,i)), 2) < 353;
end
scenario_en = temp;

% 数值与文本部分合并
config = [scenario_id, scenario_en, config.data];

% 保存Multi_Scenario属性并生成副本
addpath(genpath('./Demo'));
Current_Scenario = Multi_Scenario();
Dest_DirName = '';
for i = 1 : scenario_num
    for j = 1 : config(i,5)
        Current_Scenario.scenario_id = config(i,1);
        Current_Scenario.en_video = config(i,2);
        Current_Scenario.en_emotion = config(i,3);
        Current_Scenario.en_stat = config(i,4);
        Current_Scenario.v_obs = config(i,6);
        Current_Scenario.n_agents = config(i,7);
        Current_Scenario.trans_prop = config(i,8);
        Current_Scenario.fear_dist = config(i,9);
        Current_Scenario.g_shill_base = config(i,10);
        Current_Scenario.g_shill_range = config(i,11);
        Current_Scenario.r_coll = config(i,12);
        Current_Scenario.d_ref = config(i,13);
        Current_Scenario.v_ref = config(i,14);
        Current_Scenario.v_max = config(i,15);
        Current_Scenario.a_max = config(i,16);

        save('./Demo/parameters/param_scenario.mat', 'Current_Scenario');

        % 将./DEMO复制到Dest_DirName
        Dest_DirName = ['Scenario_', num2str(Current_Scenario.scenario_id), '_Copy_', num2str(j)];
        copyfile('./Demo', Dest_DirName);
    end
end

% 运行swarmlab_server.m
Work_DirName = '';
for i = 1 : scenario_num
    for j = 1 : config(i,5)
        Work_DirName = ['./Scenario_', num2str(i), '_Copy_', num2str(j)];
        cd(Work_DirName)
        unix('bash swarmlab_server.sh');
        cd ../;
    end
    % 等待程序执行完毕
    while true
        flag = true;
        for j = 1 : config(i,5)
            Work_DirName = ['./Scenario_', num2str(i), '_Copy_', num2str(j), '/result.mat'];
            % 判断是否存在文件Work_DirName
            file = dir(Work_DirName);
            if size(file, 1) < 1
                flag = false;
            end
        end
        if flag
            fprintf('Scenario%d is finished!\n', i);
            break;
        else
            fprintf('Waiting for Scenario%d to end...\n', i);
            pause(5);
        end
    end

end

% 读取结果并保存
addpath(genpath('./Demo'));
Scenario_Result = cell(scenario_num, max(config(:,5)));
Work_DirName = '';
for i = 1 : scenario_num
    for j = 1 : config(i,5)
        Work_DirName = ['./Scenario_', num2str(i), '_Copy_', num2str(j)];
        cd(Work_DirName)
        load('result.mat');
        Scenario_Result{i,j} = stat;
        cd ../;
    end
end
save('./Scenario_Result.mat', 'Scenario_Result');