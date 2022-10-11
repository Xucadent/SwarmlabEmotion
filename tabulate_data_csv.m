clear
clc
maindir = pwd;
var_value = [];
joint_data = [];
subdir = dir(maindir);
for i = 1 : length(subdir)
    % 正则表达式判断文件夹名称是否以"_Raw"结尾
    dir_prefix = regexp(subdir(i).name, '_Raw', 'match');
    % 如果文件夹名称以"_Raw"结尾，则将dir_prefix赋值为除去"_Raw"的文件夹名称
    if ~isempty(dir_prefix)
        dir_prefix = subdir(i).name(1:end-4);
        break;
    end
end

for i = 1 : length(subdir)
    % 正则表达式判断文件夹名称是否是"dir_prefix+数字"
    if ~isempty(regexp(subdir(i).name, [dir_prefix, '\d+'], 'match'))
        xlsxpath = fullfile(maindir, subdir(i).name, 'result.csv');
        var_value = csvread(xlsxpath, 0, 0, [0 0 15 0]);
        xlsx_data = csvread(xlsxpath, 0, 3, [0 3 15 3]);
        joint_data = [joint_data xlsx_data];
    end
end

% 将joint_data每行平均值计算并添加在最后一列
joint_data = [joint_data, mean(joint_data, 2)];

xlswrite([dir_prefix, '_time.xlsx'], [var_value, joint_data]);
