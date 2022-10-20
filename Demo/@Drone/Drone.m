classdef Drone < handle
    % DRONE : This class is meant to create and manage drones.
    % Every drone is either a fixed-wing or a quadcopter (drone_type).
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Drone general properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % pos_ned:
    %           vect3, position coordinates in the inertial frame
    %           (North, East and Down) [m]
    % vel_xyz:
    %           vect3, velocity in body frame (vx, vy, vz) [m/s]
    % rates:
    %           vect3, angular rates w.r.t. phi, theta and psi  [rad/s]
    % pos_ned_history: matrix of size n, 3: each row is vect of previous
    %                  pos_ned, position coordinates in the inertial frame 
    %                  (North, East and Down) [m]
    %
    % vel_xyz_history: matrix of size n, 3: each row is vect of previous
    %                  vel_xyz, velocities in the body frame 
    %                  (North, East and Down) [m/s]
    % prev_state:
    %           vect12, contains the previous state.
    %           state = (pos_ned, vel_xyz, attitude, rates)
    % path_len:
    %           path lenght of the drone, from the beginning of the
    %           simulation
    % command:
    %           vect4, commmand input. The variables contained depend on
    %           the autopilot version
    % full_command:
    %           vect19, full command state vector, used in the function
    %           plot_uav_state_variable
    % color:
    %           color associated to the drone for scatterplots, offline
    %           trajectory plots, ecc.
    %


    properties

        % Parameters
        p_sim
        
        map % FIXME: why does the drone need a map? That's not logical

        % State = [pos_ned; vel_xyz; attitude; rates]
        pos_ned     % pn, pe, pd
        vel_xyz     % vx, vy, vz
        
        % State history
        pos_ned_history
        vel_xyz_history

        % Auxiliary variables
        path_len
        nb_neig
        
        % Command variables
        command
        full_command

        % Emotion variables
        emotion
        
        % Plot variables
        color
        
        %Collision calibrate
        coll_cali
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods

        function Drone = Drone(p_sim, p_emo, map)

            % Create a drone: assign parameters and initialize state to 0
            Drone.p_sim = p_sim;
            Drone.map = map;
            Drone.pos_ned = zeros(2, 1);
            Drone.vel_xyz = zeros(2, 1);
            Drone.pos_ned_history = [];
            Drone.vel_xyz_history = [];   
            Drone.path_len = 0;
            Drone.nb_neig = 0;
            Drone.command = zeros(4, 1);
            Drone.full_command = zeros(19, 1);
            Drone.emotion = Emotion(p_emo);
        
            % cmap = jet(16);
            Drone.color = [0, 0, 0]';
            Drone.coll_cali = false;
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos(self, position_ned)
            % SET_POS: Set the position of the drone in the NED frame
            self.pos_ned = position_ned;
            % Update pos_ned_history
            if isempty(self.pos_ned_history)
                self.pos_ned_history = self.pos_ned';
            else
                self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
            end
            %self.z_hat = true_states([self.get_state(); self.airdata; NaN]);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel(self, velocity_xyz)
            % SET_VEL: Set the velocity of the drone
            self.vel_xyz = velocity_xyz;
            if isempty(self.vel_xyz_history)
                self.vel_xyz_history = self.vel_xyz';
            else
                self.vel_xyz_history = [self.vel_xyz_history; self.vel_xyz'];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function state = get_state(self)
            % GET_STATE: Get the state of the drone, vect12
            state = [self.pos_ned; self.vel_xyz];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_state(self, pos_ned, vel_xyz, attitude, rates)
            % SET_STATE: Set the drone state to the one passed in argument
            self.set_pos(self, pos_ned)
            self.set_vel(vel_xyz);
            self.attitude = attitude;
            self.rates = rates;

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_state(self, time)            

                % Computes the new drone position with Euler forward method.
                % This method does not take the attitude into account.
                % We suppose that the attitude is always (0,0,0), so the
                % velocity in the body frame correponds to the velocity in the
                % inertial frame. Only usable with the velocity controller.
                
                if self.coll_cali == false
                    self.vel_xyz = self.command(2:3);
                else
                    self.coll_cali = false;
                end    
                self.pos_ned = self.pos_ned + self.vel_xyz * self.p_sim.dt;
                %%self.attitude(3) = self.command(1); % to plot drone psi angle
                
                % Update pos_ned_history
                if isempty(self.pos_ned_history)
                    self.pos_ned_history = self.pos_ned';
                else
                    self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
                end
                
                % Update vel_xyz_history
                if isempty(self.vel_xyz_history)
                    self.vel_xyz_history = self.vel_xyz';
                else
                    self.vel_xyz_history = [self.vel_xyz_history; self.vel_xyz'];
                end
        end
    end
end
