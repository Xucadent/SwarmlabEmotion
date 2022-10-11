classdef SwarmViewer < handle
    % SWARMVIEWER: class to visualize the swarm in the ENU (East-North-Up)
    % 3D-space
    
    properties
        figure_handle
        cylinders_handle
        plot_initialized
        output_rate
        time_of_last_frame
        center_view_on_swarm
        viewer_type SwarmViewerType
        scatter_handle
        circle_handle
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = SwarmViewer(output_rate, center_view_on_swarm)
            self.plot_initialized = 0;
            self.output_rate = output_rate;
            self.time_of_last_frame = 0;
            self.center_view_on_swarm = center_view_on_swarm;
            self.scatter_handle = [];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = update(self, time, swarm, map)
            
            if (time-self.time_of_last_frame) >= self.output_rate - 1e-6
                self.time_of_last_frame = time;
                
                %----------------------------------------------------------
                if self.plot_initialized == 0 % Init plot
                    x0 = 66; 
                    y0 = 1; 
                    width = 700;
                    height=700;
                    self.figure_handle = figure('Name','Swarm viewer', ...
                        'NumberTitle','off', ...
                        'position', [x0, y0, width, height]); clf;
                    grid on;
                    
                    if map.ACTIVE_ENVIRONMENT
                        if strcmp(map.building_shape, 'parallelepiped')
                            draw_buildings(map);
                        elseif strcmp(map.building_shape, 'cylinder')
                            [self.figure_handle,  self.cylinders_handle]= draw_cylinders(self.figure_handle, map);
                        end
                    end
                   
                    size_agents = repmat(100, swarm.nb_agents, 1);
                    colors = swarm.get_colors();
                    pos_ned = swarm.get_pos_ned();
                    pn = pos_ned(1,:);
                    pe = pos_ned(2,:);

                    % self.circle_handle = scatter(pe, pn, size_agents, 'Marker', 'o');

                        self.scatter_handle = scatter(pe, pn, size_agents, '.');


                    
                    % // TODO: Draw paths
                    
                    if self.center_view_on_swarm || ~map.ACTIVE_ENVIRONMENT
                        axis_lim = define_axis_lim(self, swarm);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -1, 0];
                        axis(axis_min_inverted(:));
                       view(self.figure_handle.CurrentAxes,0,90);
                    else
                        axis([min(map.buildings_east)-50, max(map.buildings_east)+50,...
                            min(map.buildings_north)-50, max(map.buildings_north)+50, -1, 0])
                        view(self.figure_handle.CurrentAxes,0,90);
                    end
                    axis square;
                    drawnow
                    
                    xlabel('Y position [m]');
                    ylabel('X position [m]');
                    zlabel('Z position [m]');
                    grid on;
                    self.plot_initialized = 1;
                    
                    set(self.figure_handle,'Units','pixels');
                    %                     axis_lim = self.define_axis_lim(swarm);
                    %                     xlim(axis_lim(1, 1:2));
                    %                     ylim(axis_lim(2, 1:2));
                    
                    %----------------------------------------------------------
                else % At every other time step, redraw 
                     if map.ACTIVE_ENVIRONMENT
                        if strcmp(map.building_shape, 'parallelepiped')
                            draw_buildings(map);    %矩形暂未改好
                        elseif strcmp(map.building_shape, 'cylinder')
                              NB_EDGES  = 50;
                              [X,Y,~] = cylinder(map.building_width/2,NB_EDGES);
                              nb_buildings = length(map.buildings_north);
                            for i = 1:nb_buildings
                                Xtrasl = X + map.buildings_north(i);
                                Ytrasl = Y + map.buildings_east(i);
                                set(self.cylinders_handle(i), 'Xdata', Ytrasl, 'Ydata', Xtrasl);
                            end
                        end
                     end
                    
                    size_agents = repmat(100, swarm.nb_agents, 1);
                    colors = swarm.get_colors();

                    pos_ned = swarm.get_pos_ned();
                    pn = pos_ned(1,:);
                    pe = pos_ned(2,:);
                    set(self.scatter_handle, 'Xdata', pe, 'Ydata', pn, ...
                         'Marker', '.', 'SizeData', ...
                        size_agents, 'CData', colors');
%                     set(self.circle_handle, 'Xdata', pe, 'Ydata', pn, ...
%                          'Marker', 'o', 'SizeData', size_agents);
                    
                    if self.center_view_on_swarm || ~map.ACTIVE_ENVIRONMENT
                        axis_lim = define_axis_lim(self, swarm);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -1, 0];
                        axis(axis_min_inverted(:));
                        view(self.figure_handle.CurrentAxes,0,90);
                    end
                    drawnow;
                end
                title(self.figure_handle.CurrentAxes, sprintf('Simulation time: %.1f seconds', time));
                
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function axis_lim = define_axis_lim(self, swarm)
            % define_axis_lim - Returns the limits of the plot in the form
            % of a 3x2 matrix as follows:
            %
            % x_min x_max
            % y_min y_max
            % z_min z_max
            
            margin = 40;
            positions = swarm.get_pos_ned();
            min_pos = min(positions,[],2);
            max_pos = max(positions,[],2);
            edge = max(max_pos-min_pos);
            center = (min_pos + max_pos)/2;
            axis_lim = repmat(center,1,2) + ...
                repmat([-edge/2-margin +edge/2+margin], 2, 1);
        end
        
    end
end