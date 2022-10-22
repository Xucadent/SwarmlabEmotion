# SwarmlabEmotion
 
| 序号 | 启用视频 | 启用过程参量统计 | 启用情感机制 | 障碍物速度 | 集群规模 | 情感传递因子 | 恐惧情感激发距离 | 斥力系数基值 | 斥力系数调节范围 | 碰撞半径 | 优选距离 | 优选速度 | 最大速度 | 最大加速度 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| scenario | en_video | en_emotion | en_stat | v_obs | n_agents | trans_prop | fear_dist | g_shill_base | g_shill_range | r_coll | d_ref | v_ref | v_max | a_max |
| int | bool | bool | bool | float | int | float | float | float | float | float | float | float | float | float |


# 运行
```
1. 复制文件夹中的 Demo, Run_Swarmlab.m, Scenario_Config.csv
2. 在 Scenario_Config.csv 中修改参数(可用Excel)
3. 终端执行 matlab < Run_Swarmlab.m
4. 检查是否生成副本 Scenario_a_Copy_b
5. 检查副本中的 nohup.out 是否有错误输出
6. 全部运行完成后, 将 Scenario_Result.mat 传输到本地进行分析
```