function plot_state(self, time, end_time, iter, period, ...
    collisions, r_agent)

% plot_state - This function allows to plot the evolution of the
% state variables of the swarm in time.
%
% Inputs:
%   collisions :
%               collisions(1) = n_agent_collisions
%               collisions(2) = n_obs_collisions
%               collisions(3) = min_dist_obs
%
% Outputs:
%               graphical outputs
%               fig_handle: the updated handle to the figure
%

plot_distance = false;
plot_velocity = false;
plot_collisions = true;
plot_M_swarm = false;
plot_avrneig = false;
plot_avremotion = false;
plot_density = false;


if mod(time, period) == 0

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State variables

    pos = self.get_pos_ned();
    vel = self.get_vel_ned();
    Dist = pos2dist(pos);
    N = self.nb_agents;

    if plot_distance
        D_tilde = Dist;
        D_tilde(logical(eye(N, N))) = [];
        D_tilde = reshape(D_tilde, N - 1, N);
        D_min = min(D_tilde);
        d_min = min(D_min);
        d_max = max(D_min);
        d_mean = mean(D_min);
    end

    if plot_velocity
        v_module = sqrt(sum((vel.^2), 1));
        v_mean = mean(v_module);
        v_sd = std(v_module);
    end

    if ~isempty(collisions) && plot_collisions
        n_agent_collisions = collisions(1);
        n_obs_collisions = collisions(2);
        min_dist_obs = collisions(3);
    end

    %计算集群序参量M。
    if plot_M_swarm
        v_average=0;
        v_machine=[0;0];
        v_machine_mean=0;
        M=0;
        for i=1:N
            v_average=sqrt(vel(1,i)^2+vel(2,i)^2)+v_average;
            v_machine=v_machine+vel(:,i);
        end
        v_machine_mean=sqrt(v_machine(1,1)^2+v_machine(2,1)^2);
        M=v_machine_mean/v_average;
    end
    
    %计算平均邻居数
    if plot_avrneig
        nei_average=0;
    end

    %计算情感强度
    if plot_avremotion
        avr_emotion = 0;
        for i = 1 : N
            avr_emotion = avr_emotion + self.drones(i).emotion.frust;
        end
        avr_emotion = avr_emotion / N;
    end
    
    %计算平均集群密度 density_swarm
    if plot_density
        sum_neig = 0;
        for i=1:N
%             sum_neig = sum_neig + self.ag
        end
        density=sum_neig/(N*pi*100);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define persistent variables
    
    persistent fig_handle

    persistent d_mm_handle
    persistent dist_obs_handle

    persistent v_msd_handle

    persistent ag_col_handle
    persistent obs_col_handle

    persistent ag_thr_handle
    persistent obs_thr_handle

    persistent M_swarm_handle
    persistent emotion_handle
    persistent density_handle
    persistent az_handle

    persistent tg
    persistent tabs
    
    
    if time == 0.5
        fig_handle = [];
        d_mm_handle = [];
        dist_obs_handle = [];
        v_msd_handle = [];
        ag_col_handle = [];
        obs_col_handle = [];
        ag_thr_handle = [];
        obs_thr_handle = [];
        tg = [];
        tabs = [];
        M_swarm_handle=[];
        emotion_handle = [];
        az_handle = [];
    end

    % First time function is called: initialize figures and plots
    if isempty(fig_handle)
        x0 = 960; 
        y0 = 1; 
        width = 960;
        height = 973;
        fig_handle = figure('Name', 'Swarm state viewer', ...
            'NumberTitle', 'off', 'position', [x0, y0, width, height]); clf
        tg = uitabgroup; % tabgroup

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if N > 1 && plot_distance
            tabs(1) = uitab(tg, 'title', 'Distance');
            axes('Parent', tabs(1));

            % axes_h = get(fig_handle,'CurrentAxes');
            subplot(2, 1, 1);
            hold on
            d_mm_handle = graph_mean_min_max(iter, time, d_mean, d_min, d_max, 'd_{agents} [m]', []);
            ag_thr_handle = graph_thr(iter, time, 2 * r_agent, []);

            subplot(2, 1, 2)
            hold on
            dist_obs_handle = graph_y(iter, time, min_dist_obs, 'd_{obstacles} [m]', []);
            obs_thr_handle = graph_thr(iter, time, r_agent, []);

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if plot_velocity
            tabs(2) = uitab(tg, 'title', 'Velocity');
            axes('Parent', tabs(2));

            % fig_handle(2) = figure('Name','Velocity','NumberTitle','off');
            subplot(1, 1, 1);
            hold on
            v_msd_handle = graph_mean_sd(iter, time, v_mean, v_sd, 'velocity [m/s]', []);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if plot_collisions
            tabs(3) = uitab(tg, 'title', 'Collisions');
            axes('Parent', tabs(3));
            
            % fig_handle(3) = figure('Name','Collisions','NumberTitle','off');
            subplot(2, 1, 1)
            hold on
            ag_col_handle = graph_y(iter, time, n_agent_collisions, 'c_{agents}', []);

            subplot(2, 1, 2)
            hold on
            obs_col_handle = graph_y(iter, time, n_obs_collisions, 'c_{obstacles}', []);
        end
        
        %集群序参量M的表示
        if plot_M_swarm
            tabs(4) = uitab(tg,'title','M_swarm');
            axes('Parent', tabs(4));
            
            subplot(1,1,1);
            hold on;
            M_swarm_handle = graph_y(iter, time, M, 'M_{param}', []);
        end

        if plot_avremotion
            tabs(5) = uitab(tg,'title','Emotion');
            axes('Parent', tabs(5));

            subplot(1,1,1);
            hold on;
            emotion_handle = graph_y(iter, time, avr_emotion, 'Frust_{avr}', []);
        end
        
        if plot_density
            tabs(6) = uitab(tg,'title','Density');
            axes('Parent', tabs(6));

            subplot(1,1,1);
            hold on;
            density_handle = graph_y(iter, time, density, 'density_{swarm}', []);
        end

        tabs(7) = uitab(tg,'title','test');
            axes('Parent', tabs(7));
            
            subplot(1,1,1);
            hold on;
        az_handle = graph_y(iter, time, n_obs_collisions, 'c_{obstacles}', []);
        % At every other time step, redraw state variables
    else
        if N > 1 && plot_distance
            graph_mean_min_max(iter, time, d_mean, d_min, d_max, 'd_{agents} [m]', d_mm_handle);
            graph_thr(iter, time, 2 * r_agent, ag_thr_handle);

            graph_y(iter, time, min_dist_obs, 'd_{obstacles} [m]', dist_obs_handle);
            graph_thr(iter, time, r_agent, obs_thr_handle);
        end

        if plot_velocity
            graph_mean_sd(iter, time, v_mean, v_sd, 'velocity [m/s]', v_msd_handle);
        end
        
        if plot_collisions
            graph_y(iter, time, n_agent_collisions, 'c_{agents}', ag_col_handle);
            graph_y(iter, time, n_obs_collisions, 'c_{obstacles}', obs_col_handle);
        end

        if plot_M_swarm
            graph_y(iter, time, M, 'M_{swarm}', M_swarm_handle);
        end

        if plot_avremotion
            graph_y(iter, time, avr_emotion, 'Frust_{avr}', emotion_handle);
        end
        
        if plot_density
            graph_y(iter,time,density,'density_{swarm}',density_handle);
        end
        
    end
    
    % Save figures
    if time == end_time
        % path = strcat(dirname, '/', 'distances.fig');
        % savefig(fig_handle(1), path);
        % path = strcat(dirname, '/', 'velocity.fig');
        % savefig(fig_handle(2), path);
        % path = strcat(dirname, '/', 'collisions.fig');
        % savefig(fig_handle(4), path);
        path = strcat('E:/', 'stat1.fig');
        savefig(fig_handle, path);
    end

end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y with lable mylabel
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_y(i, t, y, lab, handle)

if isempty(handle) || i == 0
    handle = plot(t, y, 'b');
    ylabel(lab);
    % set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle, 'Xdata', [get(handle, 'Xdata'), t]);
    set(handle, 'Ydata', [get(handle, 'Ydata'), y]);
    drawnow
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add threshold to graph
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_thr(i, t, y_thr, handle)

if isempty(handle) || i == 0
    handle = plot(t, y_thr, 'r--');
    % set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle, 'Xdata', [get(handle, 'Xdata'), t]);
    set(handle, 'Ydata', [get(handle, 'Ydata'), y_thr]);
    drawnow
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y with mean and standard deviation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_mean_sd(i, t, mean, sd, lab, handle)

if isempty(handle) || i == 0
    handle(1) = plot(t, mean, 'b', 'LineWidth', 2);
    handle(2) = plot(t, mean + sd, 'b', 'LineWidth', 1.0);

    if (mean - sd) > 0
        handle(3) = plot(t, mean - sd, 'b', 'LineWidth', 1.0);
    else
        handle(3) = plot(t, 0, 'r--');
    end

    ylabel(lab);
    legend('avg', 'avg + sd', 'avg- sd');
    % Horizontal label for the axis
    % set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1), 'Xdata', [get(handle(1), 'Xdata'), t]);
    set(handle(1), 'Ydata', [get(handle(1), 'Ydata'), mean]);
    set(handle(2), 'Xdata', [get(handle(2), 'Xdata'), t]);
    set(handle(2), 'Ydata', [get(handle(2), 'Ydata'), mean + sd]);
    set(handle(3), 'Xdata', [get(handle(3), 'Xdata'), t]);

    if (mean - sd) > 0
        set(handle(3), 'Ydata', [get(handle(3), 'Ydata'), mean - sd]);
    else
        set(handle(3), 'Ydata', [get(handle(3), 'Ydata'), 0]);
    end

    drawnow
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y with mean and standard deviation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_mean_sd_min_max(i, t, mean, sd, min, max, lab, handle)

if isempty(handle) || i == 0
    handle(1) = plot(t, mean, 'b');
    handle(2) = plot(t, mean + 3 * sd, 'r--');

    if (mean - 3 * sd) > 0
        handle(3) = plot(t, mean - 3 * sd, 'r--');
    else
        handle(3) = plot(t, 0, 'r--');
    end

    handle(4) = plot(t, min, 'g');
    handle(5) = plot(t, max, 'g');
    ylabel(lab);
    legend('avg', 'avg + 3sd', 'avg- 3sd', 'min', 'max');
    % Horizontal label for the axis
    % set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1), 'Xdata', [get(handle(1), 'Xdata'), t]);
    set(handle(1), 'Ydata', [get(handle(1), 'Ydata'), mean]);
    set(handle(2), 'Xdata', [get(handle(2), 'Xdata'), t]);
    set(handle(2), 'Ydata', [get(handle(2), 'Ydata'), mean + 3 * sd]);
    set(handle(3), 'Xdata', [get(handle(3), 'Xdata'), t]);

    if mean - 3 * sd > 0
        set(handle(3), 'Ydata', [get(handle(3), 'Ydata'), mean - 3 * sd]);
    else
        set(handle(3), 'Ydata', [get(handle(3), 'Ydata'), 0]);
    end

    set(handle(4), 'Xdata', [get(handle(4), 'Xdata'), t]);
    set(handle(4), 'Ydata', [get(handle(4), 'Ydata'), min]);
    set(handle(5), 'Xdata', [get(handle(5), 'Xdata'), t]);
    set(handle(5), 'Ydata', [get(handle(5), 'Ydata'), max]);
    drawnow
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y with mean, min and max
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_mean_min_max(i, t, mean, min, max, lab, handle)

if isempty(handle) || i == 0
    handle(1) = plot(t, mean, 'b', 'LineWidth', 2);
    handle(2) = plot(t, min, 'b', 'LineWidth', 1.0);
    handle(3) = plot(t, max, 'b', 'LineWidth', 1.0);
    ylabel(lab);
    legend('avg', 'min', 'max', 'AutoUpdate', 'off');
    % Horizontal label for the axis
    % set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1), 'Xdata', [get(handle(1), 'Xdata'), t]);
    set(handle(1), 'Ydata', [get(handle(1), 'Ydata'), mean]);
    set(handle(2), 'Xdata', [get(handle(2), 'Xdata'), t]);
    set(handle(2), 'Ydata', [get(handle(2), 'Ydata'), min]);
    set(handle(3), 'Xdata', [get(handle(3), 'Xdata'), t]);
    set(handle(3), 'Ydata', [get(handle(3), 'Ydata'), max]);
    drawnow
end

end
