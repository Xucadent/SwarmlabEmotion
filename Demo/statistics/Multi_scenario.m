classdef Multi_scenario
    %连续调参
    
    properties
        scenario_id
        % 输出配置
        en_video        % 是否输出视频 1
        en_emotion      % 是否启用情感 1
        en_stat         % 是否启用过程统计 
        repeat_times    % 重复次数
        % 主要变量
        v_obs           % 障碍物速度 1
        n_agents        % 集群规模 1
        trans_prop      % 传递因数 1
        fear_dist       % 恐惧距离 1
        g_shill_base    
        g_shill_range
        % 集群参数
        r_coll
        d_ref
        v_ref
        v_max
        a_max
        % 统计量
        swarm_stat
    end
    
    methods
        function self = Multi_scenario()
            self.swarm_stat = Swarm_Statistics([], [], []);
        end
        
        function self = Run_Scenario(self)
            p_swarm.nb_agents = self.n_agents;
            p_swarm.g_shill_base = self.g_shill_base;
            p_swarm.g_shill_range = self.g_shill_range;
            p_emotion.transfer_prop = self.trans_prop;
            p_emotion.fear_dist = self.fear_dist;
            p_swarm.r_coll = self.r_coll;
            p_swarm.d_ref = self.d_ref;
            p_swarm.v_ref = self.v_ref;
            p_swarm.v_max = self.v_max;
            p_swarm.a_max = self.a_max;
            p_swarm.en_emotion = self.en_emotion;

            run('example_vasarhelyi.m');
            
            stat = self.swarm_stat;
            fprintf("scenario: %d collision: %d, %d\n", self.scenario_id,...
            stat.stat_collisions(1), stat.stat_collisions(2));

            save('result.mat', 'stat');
        end

    end
end

