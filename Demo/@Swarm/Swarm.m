classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic agents of type
    % "Drone"
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Swarm general properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % drones:
    %           vector of Drone objects
    % nb_agents:
    %           size of the above vector
    % equivalent_drone:
    %           for path planner, drone at the barycenter ...
    %           of the swarm for command computations
    % pos_ned:

    properties
        drones % a vector of Drone objects
        nb_agents % size of the above vector
        equivalent_drone % for path planner, drone at the barycenter ...
                         % of the swarm for command computations
        collisions_history
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm()
            self.drones = [];
            self.nb_agents = 0;
            self.collisions_history = [];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_drone(self, p_sim, map)
            self.nb_agents = self.nb_agents + 1;
            drone = Drone(p_sim, map);
            self.drones = [self.drones; drone];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos(self, pos)

            for i = 1:self.nb_agents
                self.drones(i).set_pos(pos(:, i));
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel(self, vel)

            for i = 1:self.nb_drones
                self.drones(i).set_vel(vel(:, i));
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Pos_ned = get_pos_ned(self)
            % Return positions of the agent of the swarm in a matrix shape
            % of size 3 x nb_agents
            %
            %        agent_1   agent_2   ...   agent_N
            %   pn
            %   pe
            %   pd

            Pos_ned = zeros(2, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                Pos_ned(:, i) = drone.pos_ned;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Vel_ned = get_vel_ned(self)
            % Return velocities of the agents of the swarm in a matrix shape
            % of size 3 x nb_agents
            %        agent_1   agent_2   ...   agent_N
            %   vn
            %   ve
            %   vd
            
            Vel_ned = zeros(2, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                Vel_ned(:,i)=drone.vel_xyz;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Nb_neig = get_nb_neig(self)
            Nb_neig = zeros(1, self.nb_agents);
            for i = 1:self.nb_agents
                drone = self.drones(i);
                Nb_neig(:,i)=drone.nb_neig;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Emotion_swarm = get_emotion(self)
            Emotion_swarm(1, self.nb_agents) = Emotion();
            for i = 1:self.nb_agents
                drone = self.drones(i);
                Emotion_swarm(i) = drone.emotion;
            end
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_state(self, state)
            Pos_ned = state(repmat([true true true false false false], ...
                self.nb_drones,1));
            Pos_ned = reshape(Pos_ned,2,[]);
            Vel_xyz = state(repmat([false false false true true true], ...
                self.nb_drones,1));
            Vel_xyz = reshape(Vel_xyz,2,[]);
            
            self.set_pos(Pos_ned);
            self.set_vel(Vel_xyz);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function state = get_state(self)
            Pos_ned = self.get_pos_ned();
            Vel_ned = self.get_vel_ned();
            state = [Pos_ned; Vel_ned];
            state = state(:);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Path_len = get_path_len(self)

            Path_len = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);

                Path_len(1, i) = drone.path_len;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel_commands(self, commands)

            for i = 1:self.nb_agents
                drone = self.drones(i);
                drone.command(1) = 0;
                drone.command(2:3) = commands(:, i);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function colors = get_colors(self)
            colors = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                colors(:, i) = self.drones(i).color;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_state(self, time)

            for i = 1:self.nb_agents
                self.drones(i).update_state(time);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [vel_commands, collisions] = update_command(self, p_swarm, r_coll, dt)

            % Select the swarm algorithm and call the associated update
            [vel_commands, collisions, coll_calibration] = self.compute_vel_vasarhelyi(p_swarm, r_coll, dt);
            self.set_vel_commands(vel_commands);
            
            if isempty(self.collisions_history)
                self.collisions_history = collisions;
            else
                self.collisions_history = [self.collisions_history; collisions];
            end
            
            for agent = 1:self.nb_agents
                if sum(coll_calibration(:, agent)) ~= 0
                    self.drones(agent).set_pos(coll_calibration(1:2, agent));
                    self.drones(agent).set_vel(coll_calibration(3:4, agent));
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function pos_ned_history = get_pos_ned_history(self)
            for i = 1:self.nb_agents
                pos_ned_history(:, (2 * (i - 1) + 1) : (2 * (i - 1) + 2)) = self.drones(i).pos_ned_history;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function vel_xyz_history = get_vel_xyz_history(self)
            vel_xyz_history = [];
            for i = 1:self.nb_agents
                vel_xyz_history(:, (2 * (i - 1) + 1) : (2 * (i - 1) + 2)) = self.drones(i).vel_xyz_history;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % // TODO: add this function to the SwarmViewer
        fig_handle = draw_agents_energy(self, time, period, fig_handle, axes_lim);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_state(self, time, T, period, is_active_cyl, ...
            collisions, map, dirname);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_video(self, time, T, period, fig_handle, path);
    end

end
 