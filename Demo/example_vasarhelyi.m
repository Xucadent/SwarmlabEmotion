%% Clear console and workspace and add project root to path

close all
clearvars -except adj_param curr_scenerio

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename));
addpath(genpath(project_root));

%% Simulation options

ACTIVE_ENVIRONMENT = true;
DEBUG = false;
VIDEO = true;
CENTER_VIEW_ON_SWARM = true;

if DEBUG || VIDEO
    results_dirname = strcat('results');
    date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
    if ~exist(results_dirname, 'dir')
        mkdir(results_dirname)
    end
end

fontsize = 12;

SWARM_VIEWER_TYPE = "agent";


%% Call parameters files

run('param_sim');
run('param_map'); 
run('param_swarm');
run('param_vasarhelyi');

%% Init Swarm object, Statistics, Viewer and other variables

% Init swarm and set positions
swarm = Swarm();

for i = 1 : p_swarm.nb_agents
    swarm.add_drone(p_sim, map);
end
swarm.set_pos(p_swarm.Pos0);

% Init obstacle velocity
%velocity = Adjust_Param(adj_param);
velocity = 12;
p_swarm.cylinder_vel = zeros(2, p_swarm.n_cyl); %速度大小，取＋
for i = 1 : p_swarm.n_cyl
    if mod(i,2)
        p_swarm.cylinder_vel(:, i) = [0; velocity];
    else
        p_swarm.cylinder_vel(:, i) = [0; -velocity];
    end
end

% Init swarm statistics
swarm_statistics = Swarm_Statistics(p_swarm, p_sim);

% Init variables for history
x0 = [p_swarm.Pos0; zeros(2,p_swarm.nb_agents)];
x_history(1,:) = x0(:);
coll_count = zeros(1,2);

% Init video
if VIDEO    
    video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
    video_filepath = strcat(results_dirname, '/', video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
end

% Init viewer
 %swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
swarm_viewer = SwarmViewer(p_sim.dt, CENTER_VIEW_ON_SWARM);
swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
states_handle = [];

%% Clear Unused Variables
clear date_string project_root video_filename video_filepath


%% Main simulation loop
%disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time
    % Move obstacles
    [map,p_swarm] = move_obstacles(map, p_swarm, p_sim);
    
    % Compute velocity commands from swarming algorithm
    [vel_c, collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    
    % Get swarm statistics
    swarm_statistics = swarm_statistics.calc_statistics(swarm, collisions, time);
    %swarm_statistics.stat_collisions
    
    % Update swarm states and plot the drones
    swarm.update_state(time);
   
    % Plot state variables for debugging
    if DEBUG
        swarm.plot_state(time - swarm_statistics.stat_time(3), p_sim.end_time - swarm_statistics.stat_time(3), ...
            1, p_sim.dt_plot, collisions, p_swarm.r_coll/2);
    end
    
    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map);
        video.update(time, swarm_viewer.figure_handle);  
    end
    
    %Start and Reach Time  
    if swarm_statistics.end_flag
        break;
    end
end

if VIDEO
    video.close(); 
end

% Close all plots
close all;

% disp('Simulation completed successfully');
