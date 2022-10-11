function [vel_command, collisions, coll_calibration] = compute_vel_vasarhelyi(self, p_swarm, r_agent, dt)
    % VASARHELYI SWARM ALGORITHM
    % This is an implementation of the Vasarhelyi algorithm. It allows the
    % navigation of a swarm of agents in presence of obstacles and walls.
    %
    % Ref:      Vasarhelyi, Science Robotics, 2018
    % 
    % Modif:    a cohesion term has been added to make the agents get
    %           closer when they are farther than r0_rep.
    %
    % Inputs:
    %   p_swarm: swarm parameters
    %   r_agent: safety radius of agents
    %   dt: time step
    %
    % Outputs:
    %   vel_command: commanded velocities for every agent
    %   collisions: [nb_agent_collisions nb_obs_collisions min_dist_obs]
    %
    
    
    %% Initialize variables

    pos = self.get_pos_ned();
    vel = self.get_vel_ned();

    % Initialize variables
    nb_agents = self.nb_agents;
    M = zeros(nb_agents, nb_agents);    % Neighborhood matrix
    dist_mat = zeros(nb_agents, nb_agents);    % Distance matrix
    vel_rep = zeros(2, nb_agents);      % Repulsion velocity
    vel_fric = zeros(2, nb_agents);     % Velocity matching velocity
    vel_wall = zeros(2, nb_agents);     % Arena repulsion velocity
    vel_obs = zeros(2, nb_agents);      % Obstacle repulsion velocity
    vel_goal = zeros(2, nb_agents);     % Destination or direction attraction velocity
    vel_command = zeros(2, nb_agents);  % Total commanded velocity
    
    nb_agent_collisions = 0; % Nb of collisions among agents
    nb_obs_collisions = 0; % Nb of collisions against obstacles
    min_dist_obs = 1e5;
    
    coll_calibration = zeros(4, nb_agents);
    persistent prep rrep agent_coll
    if isempty(prep)
        prep = ones(nb_agents) .* p_swarm.g_shill_base;
    end
    if isempty(rrep)
        rrep = ones(nb_agents) .* p_swarm.r0_rep;
    end
    if isempty(agent_coll)
        agent_coll = zeros(nb_agents, nb_agents);
    end
    
    %% Compute velocity commands
    
    for agent = 1:nb_agents
        
        
        %% Find neighbors
        
        % Compute agent-agent distance matrix
        p_rel = pos - pos(:, agent);
        dist = sqrt(sum((p_rel.^2), 1));
        dist_mat(agent, :) = dist;

        % Define neighbours list
        neig_list = (1:nb_agents)';
        neig_list = neig_list(dist ~= 0);

        % Count collisions
        % nb_agent_collisions = nb_agent_collisions + sum(dist < 2 * r_agent) - 1;
        for i = 1:nb_agents
            if dist(i) < 2 * r_agent 
                if agent_coll(agent, i) == 0 && agent_coll(i, agent) == 0 && i ~= agent
                    nb_agent_collisions = nb_agent_collisions + 1;
                    agent_coll(agent, i) = 1;
                    agent_coll(i, agent) = 1;
                end
            else
                agent_coll(agent, i) = 0;
                agent_coll(i, agent) = 0;
            end
        end

        % Initialize number of neighbours
        nb_neig = nb_agents - 1;

        % Constraint on neighborhood given by the euclidean distance
        if isfield(p_swarm, 'r')
            neig_list = neig_list(dist(neig_list) < p_swarm.r);
            nb_neig = length(neig_list);
        end

        % Constraint on neighborhood given by the topological distance
        if isfield(p_swarm, 'max_neig')
            if nb_neig > p_swarm.max_neig
                [~, idx] = sort(dist(neig_list));
                neig_list = neig_list(idx(1:p_swarm.max_neig));
                nb_neig = p_swarm.max_neig;
            end
        end
        
        % Update drone property
        self.drones(agent).nb_neig = nb_neig;

        % Adjacency matrix (asymmetric in case of limited fov)
        M(agent, neig_list) = 1;
        
        %% Compute agent-agent contributions (vel_fric)
        
        if nb_neig ~= 0
            v_rel = vel - vel(:, agent);
            v_rel_norm = sqrt(sum((v_rel.^2), 1));

            % Compute vel and pos unit vector between two agents
            p_rel_u = -p_rel ./ dist;
            v_rel_u = -v_rel ./ v_rel_norm;

            for agent2 = neig_list'
                
                % Repulsion and attraction

                % Fear Emotion
                % if dist(agent2) < rrep(agent)  % repulsion
                %     vel_rep(:, agent) = vel_rep(:, agent) + ...
                %         p_swarm.p_rep * (rrep(agent) - dist(agent2)) * p_rel_u(:, agent2);
                % else  % attraction
                %     vel_rep(:, agent) = vel_rep(:, agent) + ...
                %         p_swarm.p_rep * (dist(agent2) - rrep(agent)) *- p_rel_u(:, agent2);
                % end

                % PF
                if dist(agent2) < p_swarm.d_ref  % repulsion
                    vel_rep(:, agent) = vel_rep(:, agent) + ...
                        p_swarm.p_rep * (p_swarm.d_ref - dist(agent2)) * p_rel_u(:, agent2);
                else  % attraction
                    vel_rep(:, agent) = vel_rep(:, agent) + ...
                        p_swarm.p_rep * (dist(agent2) - p_swarm.d_ref) *- p_rel_u(:, agent2);
                end

                % Velocity alignement
                v_fric_max = get_v_max(p_swarm.v_fric, dist(agent2) - p_swarm.r0_fric, p_swarm.a_fric, p_swarm.p_fric);

                if v_rel_norm(agent2) > v_fric_max
                    vel_fric(:, agent) = vel_fric(:, agent) + ...
                        p_swarm.C_fric * (v_rel_norm(agent2) - v_fric_max) * v_rel_u(:, agent2);
                end
            end
        end
        
        
        %% Compute wall and obstacle avoidance 

        % Add arena repulsion effect (vel_wall)
        if (p_swarm.is_active_arena == true)
            unit = eye(2);
            %On each axis we have the two repulsions
            for axis = 1:2
                %On each axis there is two forces (each side of the arena)
                for dir = 1:2
                    dist_ab = abs(pos(axis, agent) - p_swarm.x_arena(axis, dir));

                    %Compute velocity of wall shill agent toward center of the arena
                    v_wall_virtual = unit(:, axis) .* p_swarm.v_shill;

                    if dir == 2
                        v_wall_virtual = -v_wall_virtual;
                    end

                    %Compute relative velocity (Wall - Agent)
                    vel_ab = sqrt(sum((vel(:, agent) - v_wall_virtual).^2));

                    r0_shill = p_swarm.r0_shill_base + self.drones(agent).emotion.fear * p_swarm.r0_shill_range;
                    v_wall_max = get_v_max(0, dist_ab - r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                    if vel_ab > v_wall_max
                        vel_wall(:, agent) = vel_wall(:, agent) + ...
                            (vel_ab - v_wall_max) * (v_wall_virtual - vel(:, agent)) ./ vel_ab;
                    end
                end
            end
        end

        % Compute spheric effect (vel_obs 1)
        min_dist_obs = 1e5;
        if (p_swarm.is_active_spheres == true)

            for obs = 1:p_swarm.n_spheres
                % Get obstacle center and radius
                c_obs = p_swarm.spheres(1:3, obs);
                r_obs = p_swarm.spheres(4, obs);

                % Compute distance agent(a)-obstacle(b)
                dist_ab = sqrt(sum((pos(:, agent) - c_obs).^2)) - r_obs;
                nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

                % Set the virtual speed of the obstacle direction out of
                % the obstacle
                v_obs_virtual = (pos(:, agent) - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill;

                % Compute relative velocity agent-obstacle
                vel_ab = sqrt(sum((vel(:, agent) - v_obs_virtual).^2));

                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end
                
                r0_shill = p_swarm.r0_shill_base + self.drones(agent).emotion.fear * p_swarm.r0_shill_range;
                v_obs_max = get_v_max(0, dist_ab - r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                if vel_ab > v_obs_max
                    vel_obs(:, agent) = vel_obs(:, agent) + (vel_ab - v_obs_max) * (v_obs_virtual - vel(:, agent)) ./ vel_ab;
                end
                %vel_obs(:, agent) = vel_obs(:, agent) * (p_swarm.g_shill_base + self.drones(agent).emotion.frust * p_swarm.g_shill_range);
                vel_obs(:, agent) = vel_obs(:, agent) * 1;
            end
        end

        % Compute cylindric effect (vel_obs 2)
        if (p_swarm.is_active_cyl == true)

            for obs = 1:p_swarm.n_cyl
                % Get obstacle center and radius
                c_obs = p_swarm.cylinders(1:2, obs);
                r_obs = p_swarm.cylinders(3, obs);

                % Compute distance agent(a)-obstacle(b)
                 dist_ab = sqrt(sum((pos(1:2, agent) - c_obs).^2)) - r_obs;
                 nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);
                 
                 if dist_ab < 1
                     cylinder_n = (pos(:, agent) - c_obs)./norm(pos(:, agent) - c_obs);
                     %position calibration
                     coll_calibration(1:2, agent) = (r_obs + r_agent).*(1.01).*cylinder_n + c_obs;
                     %velocity calibration
                     rel_vel = vel(:, agent) - p_swarm.cylinder_vel(:, obs);
                     coll_calibration(3:4, agent) = rel_vel - 2.*cylinder_n.*dot(rel_vel, cylinder_n) + p_swarm.cylinder_vel(:, obs);
                     self.drones(agent).coll_cali = true;
                 end
    
                % Set the virtual speed of the obstacle direction out of
                % the obstacle
                v_obs_virtual = (pos(1:2, agent) - c_obs) / (sqrt(sum((pos(1:2, agent) - c_obs).^2))) * p_swarm.v_shill;

                % Compute relative velocity agent-obstacle
                vel_ab = sqrt(sum((vel(1:2, agent) - v_obs_virtual).^2));

                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end
                
                % r0_shill = p_swarm.r0_shill_base + self.drones(agent).emotion.fear * p_swarm.r0_shill_range;
                r0_shill = p_swarm.r0_shill_base;
                v_obs_max = get_v_max(0, dist_ab - r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                if vel_ab > v_obs_max
                    vel_obs(1:2, agent) = vel_obs(1:2, agent) + (vel_ab - v_obs_max) * (v_obs_virtual - vel(1:2, agent)) ./ vel_ab;
                end
                vel_obs(:, agent) = vel_obs(:, agent) * (p_swarm.g_shill_base + self.drones(agent).emotion.fear * p_swarm.g_shill_range);
                 %vel_obs(:, agent) = vel_obs(:, agent) * 2;
            end
        end
        
        %% Compute destination or direction contribution (vel_goal)

        if p_swarm.is_active_migration == true 
            vel_goal(:, agent) = p_swarm.v_ref * p_swarm.u_ref;
        elseif p_swarm.is_active_goal == true
            x_goal_rel = p_swarm.x_goal - pos(:, agent);
            if norm(x_goal_rel) < 100
                u_goal = x_goal_rel / 100;
            else
                u_goal = x_goal_rel / norm(x_goal_rel);
            end
            vel_goal(:, agent) = p_swarm.v_ref * u_goal;
        else
            vel_goal(:, agent) = [0; 0];
        end

        %% Calculate emotion intensity and effects
        neigfear_num = 0;
        for neig = 1 : nb_neig
            if self.drones(neig_list(neig)).emotion.status == EmotionEnum.Fear
                neigfear_num = neigfear_num + 1;
            end
        end

        emotion = self.drones(agent).emotion;
        %emotion = emotion.calc_frust(self.drones(agent).vel_xyz_history);
        emotion = emotion.calc_fear(neigfear_num, nb_neig, min_dist_obs);
        self.drones(agent).emotion = emotion.update_emotion();
        %self.drones(agent).color(3) = 10*emotion.frust;
        %self.drones(agent).color(2) = 0.1;
        self.drones(agent).color(1) = emotion.fear;
        %self.drones(agent).color(1) = 10*emotion.frust;
        
        prep(agent) = p_swarm.g_shill_base;% + self.drones(agent).emotion.frust * p_swarm.g_shill_range;
        rrep(agent) = 8 - self.drones(agent).emotion.fear * 3;
        

        %% Sum agent-agent and obstacle contributions

        vel_command(:, agent) = vel_rep(:, agent) + vel_fric(:, agent) + vel_obs(:, agent) + vel_wall(:, agent) + vel_goal(:, agent);

        
    end
    
    
    %% Compute collisions and bound velocities and accelerations

    % Total number of collisions per time step
    %nb_agent_collisions = nb_agent_collisions / 2; % reciprocal
    collisions = [nb_agent_collisions nb_obs_collisions min_dist_obs];

    % Add random effect on velocities
    if isfield(p_swarm, 'c_r')
        vel_command = vel_command + p_swarm.c_r * randn(2, nb_agents);
    end

    % Bound velocities and acceleration
    if ~isempty(p_swarm.max_v)
        vel_cmd_norm = sqrt(sum((vel_command.^2), 1));
        v_norm = sqrt(sum((vel.^2), 1));
        
        idx_to_bound = (vel_cmd_norm > p_swarm.max_v);
        if sum(idx_to_bound) > 0
            vel_command(:, idx_to_bound) = p_swarm.max_v * ...
                vel_command(:, idx_to_bound) ./ repmat(vel_cmd_norm(idx_to_bound), 2, 1);
        end
    end
    if ~isempty(p_swarm.max_a)
        accel_cmd = (vel_command-vel)./dt;
        accel_cmd_norm = sqrt(sum(accel_cmd.^2, 1));
        idx_to_bound = ( accel_cmd_norm > p_swarm.max_a | accel_cmd_norm < - p_swarm.max_a);
        if sum(idx_to_bound) > 0
            vel_command(:, idx_to_bound) = vel(:, idx_to_bound) + ...
                dt*p_swarm.max_a * accel_cmd(:, idx_to_bound) ./ ...
                repmat(accel_cmd_norm(idx_to_bound), 2, 1);
        end
    end


end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate V fric max
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ v_fricmax ] = get_v_max(v_fric, r, a, p)

    if r < 0
        v_fricmax = 0;
    elseif r * p > 0 && r * p < a / p
        v_fricmax = r * p;
    else
        v_fricmax = sqrt(2 * a * r - a^2 / p^2);
    end

    if v_fricmax < v_fric
        v_fricmax = v_fric;
    end
end
