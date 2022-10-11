classdef Multi_scenario
    %连续调参
    
    properties
        scenario_num
        scenario_result
        param_min
        param_max
        current_scenario
    end
    
    methods
        function self = Multi_scenario()
            self.scenario_num = 16;
            self.param_min = 0;
            self.param_max = 15;
            self.scenario_result  = zeros(self.scenario_num, 4);
            self.scenario_result(:, 1) = linspace(self.param_min, self.param_max, self.scenario_num);
            self.current_scenario = 1;
        end
        
        function param = Adjust_Param(self)
            param = self.scenario_result(self.current_scenario, 1);
        end
        
        function self = Update_Result(self, coll_agents, coll_obs, delta_time)
            self.scenario_result(self.current_scenario, 2) = coll_agents;
            self.scenario_result(self.current_scenario, 3) = coll_obs;
            self.scenario_result(self.current_scenario, 4) = delta_time;
            self.current_scenario = self.current_scenario + 1;
        end
    end
end

