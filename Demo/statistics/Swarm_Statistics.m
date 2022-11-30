classdef Swarm_Statistics
    % 集群统计量
    
    properties
        en_distance = false;  % 平均距离
        en_velocity = false;  % 平均速度
        en_collisions = true; % 碰撞曲线和碰撞统计
        en_M_swarm = false;    % 序参量
        en_avrneig = false;   % 平均邻居数
        en_avremotion = false; % 平均情感强度
        en_density = false;    % 平均集群密度
        
        start_edge = -120;      % 进入有效区域边界
        end_edge = 100;         % 到达目标区域边界
        reach_prop = 0.9;       % 结束时到达比例下限
        
        p_emotion;
    end
    
    % 初值和中间变量
    properties
        calc_dt     % 统计间隔
        agent_nb    % 集群规模
        agent_r     % 感知范围
        start_flag  % 进入有效区域标志
        end_flag    % 到达终点标志
        
        time        % 当前时间(准确)
        agent_pos   % 无人机位置
        agent_vel   % 无人机速度
        agent_neig  % 无人机邻居数
        agent_emo Emotion    % 无人机情感(Emotion对象数组)
        collisions  % 当前时刻碰撞次数
    end
    
    % 统计量
    properties
        stat_time
        stat_distance
        stat_velocity
        stat_collisions
        stat_MSwarm
        stat_avrneig
        stat_avremotion
        stat_density
    end
    
    methods
        function self = Swarm_Statistics(p_swarm, p_sim, p_emotion)
            self.p_emotion = p_emotion;
            
            if ~isempty(p_swarm) && ~isempty(p_sim)
                self.calc_dt = p_sim.dt_plot;
                self.agent_nb = p_swarm.nb_agents;
                self.agent_r = p_swarm.r;
                self.start_flag = false;
                self.end_flag = false;
                self.agent_pos = zeros(2, self.agent_nb);
                self.agent_vel = zeros(2, self.agent_nb);
                for i = 1:self.agent_nb
                    self.agent_emo(i) = Emotion(p_emotion);
                end
                self.agent_neig = zeros(1, self.agent_nb);
                self.collisions = zeros(1,2);

                self.stat_time = zeros(1,4);        % 开始(准确)/结束(准确)/开始(统计)/统计次数(统计)
                self.stat_distance = zeros(3,1);    % 最小/最大/平均
                self.stat_velocity = zeros(2,1);    % 平均/方差
                self.stat_collisions = zeros(1,2);  % 无人机之间/无人机和障碍物(总和, 不统计随时间变化关系)
                self.stat_MSwarm = zeros(1,1);      % 每一统计时刻的序参量
                self.stat_avrneig = zeros(1,1);     % 每一统计时刻的平均邻居数
                self.stat_avremotion = zeros(1,1);  % 每一统计时刻的平均情感强度
                self.stat_density = zeros(1,1);     % 每一统计时刻的集群密度
            end  
        end
        
        function [start_flag, end_flag] = get_timeflags(self)
            reach_cnt = 0;
            start_flag = true;
            end_flag = false;
            for agent = 1:self.agent_nb
                if self.agent_pos(1, agent) < self.start_edge
                    start_flag = false;
                elseif self.agent_pos(1, agent) > self.end_edge
                    reach_cnt = reach_cnt + 1;
                end
            end
            if reach_cnt / self.agent_nb >= self.reach_prop
                end_flag = true;
            end
        end
        
        function stat_time = calc_stat_time(self)
            stat_time = self.stat_time;
            if self.start_flag && (self.stat_time(1) == 0)
                stat_time(1) = self.time;
            end
            
            if self.end_flag && (self.stat_time(2) == 0)
                stat_time(2) = self.time;
            end
            
            if mod(self.time, self.calc_dt) == 0
                if self.start_flag && (self.stat_time(3) == 0) 
                    stat_time(3) = self.time;
                end

                if self.stat_time(3)
                    stat_time(4) = stat_time(4) + 1;
                end
            end
        end
        
        function stat_distance = calc_stat_distance(self)
            Dist = pos2dist(self.agent_pos);
            D_tilde = Dist;
            D_tilde(logical(eye(N, N))) = [];
            D_tilde = reshape(D_tilde, N - 1, N);
            D_min = min(D_tilde);
            stat_distance = [min(D_min) max(D_min) mean(D_min)];
            stat_distance = [self.stat_distance; stat_distance];
        end
        
        function stat_velocity = calc_stat_velocity(self)
            v_module = sqrt(sum((self.agent_vel.^2), 1));
            stat_velocity = [mean(v_module) std(v_module)];
            stat_velocity = [self.stat_velocity; velocity];
        end
        
        function stat_collisions = calc_stat_collisions(self)
            stat_collisions = self.stat_collisions + self.collisions(1:2);
        end
        
        function stat_MSwarm = calc_stat_MSwarm(self)
            v_average = 0;
            v_machine = [0;0];
            v_machine_mean = 0;
            M = 0;
            for i = 1 : self.agent_nb
                v_average=sqrt(self.agent_vel(1,i)^2+self.agent_vel(2,i)^2)+v_average;
                v_machine=v_machine+self.agent_vel(:,i);
            end
            v_machine_mean=sqrt(v_machine(1,1)^2+v_machine(2,1)^2);
            M=v_machine_mean/v_average;
            stat_MSwarm = [self.stat_MSwarm; M];
        end
        
        function stat_avrneig = calc_stat_avrneig(self)
            % % 好像现在平均邻居数和集群密度是成正比的, 没必要都算吧
            % stat_avrneig = 0;
            % for i = 1 : self.agent_nb
            %     stat_avrneig = stat_avrneig + self.agent_emo(i).frust;
            % end
            % stat_avrneig = stat_avrneig / self.agent_nb;
            % stat_avrneig = [self.stat_avrneig; stat_avrneig];
        end
        
        function stat_avremotion = calc_stat_avremotion(self)
            stat_avremotion = 0;
            for i = 1 : self.agent_nb
                stat_avremotion = stat_avremotion + self.agent_emo(i).fear;
            end
            stat_avremotion = stat_avremotion / self.agent_nb;
            stat_avremotion = [self.stat_avremotion; stat_avremotion];
        end
        
        function stat_density = calc_stat_density(self)
            % sum_neig = 0;
            % for i = 1 : self.agent_nb
            %     sum_neig = sum_neig + self.agent_emo(i).frust;
            % end
            % stat_density = sum_neig / (self.agent_nb * pi * self.agent_r.^2);
        end
        
        function obj = calc_statistics(self, swarm, collisions, time)
            self.time = time;
            self.agent_pos = swarm.get_pos_ned;
            [self.start_flag, self.end_flag] = self.get_timeflags();
            self.stat_time = self.calc_stat_time();

            if self.en_collisions && self.start_flag
                self.collisions = collisions;
                self.stat_collisions = self.calc_stat_collisions();
            end
            
            if (mod(time, self.calc_dt) == 0) && self.start_flag
                self.agent_vel = swarm.get_vel_ned;
                self.agent_neig = swarm.get_nb_neig;
                self.agent_emo = swarm.get_emotion();
                
                if self.en_distance
                    self.stat_distance = self.calc_stat_distance();
                end

                if self.en_velocity
                    self.stat_velocity = self.calc_stat_velocity();
                end

                if self.en_M_swarm
                    self.stat_MSwarm = self.calc_stat_MSwarm();
                end

                if self.en_avrneig
                    self.stat_avrneig = self.calc_stat_avrneig();
                end

                if self.en_avremotion
                    self.stat_avremotion = self.calc_stat_avremotion();
                end

                if self.en_density
                    self.stat_density = self.calc_stat_density();
                end
            end
            
            obj = self;
        end
    end
end

