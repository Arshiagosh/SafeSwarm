classdef SwarmVisualizer < handle
    %% SwarmVisualizer — real-time 2D visualisation of the swarm
    %
    % Opens a figure with a dark background and animates the swarm. Graphics
    % handles (trails, agent markers, obstacles) are created once in the
    % constructor; update() then only sets their data each step. This keeps
    % redraws fast even for hundreds of agents and long runs.
    %
    % Example:
    %   viz            = SwarmVisualizer(swarm, env);
    %   sim.visualizer = viz;
    %   sim.run();

    properties
        fig
        ax
        trail_length = 40;  % number of past positions shown per agent
        N                   % number of agents (fixed at construction)
        h_trail             % 1×N animatedline handles (position trails)
        h_agent             % 1×N marker handles (agent bodies)
        h_title             % title text handle
    end

    methods
        function obj = SwarmVisualizer(swarm, env)
            % SwarmVisualizer(swarm, env)
            %   swarm : Swarm object (sets N and initial positions)
            %   env   : Environment object (sets axis limits and obstacles)
            obj.N = swarm.N;

            obj.fig = figure('Name', 'SwarmSim', 'Color', 'k');
            obj.ax  = axes('Parent', obj.fig, 'Color', 'k', ...
                           'XColor', 'w', 'YColor', 'w');
            axis(obj.ax, [env.x_lim(1) env.x_lim(2) env.y_lim(1) env.y_lim(2)]);
            axis(obj.ax, 'equal');
            hold(obj.ax, 'on');
            grid(obj.ax, 'on');
            obj.ax.GridColor = [0.3 0.3 0.3];
            xlabel(obj.ax, 'x (m)', 'Color', 'w');
            ylabel(obj.ax, 'y (m)', 'Color', 'w');

            % Static elements drawn once
            obj.draw_obstacles(env);
            if ~isempty(env.goal)
                plot(obj.ax, env.goal(1), env.goal(2), 'g*', ...
                     'MarkerSize', 15, 'LineWidth', 2);
            end

            % Persistent per-agent handles
            obj.h_trail = gobjects(1, obj.N);
            obj.h_agent = gobjects(1, obj.N);
            for i = 1:obj.N
                c = swarm.agents{i}.color;
                p = swarm.agents{i}.position;
                obj.h_trail(i) = animatedline(obj.ax, 'Color', c, 'LineWidth', 1, ...
                                              'MaximumNumPoints', obj.trail_length);
                addpoints(obj.h_trail(i), p(1), p(2));
                obj.h_agent(i) = plot(obj.ax, p(1), p(2), 'o', ...
                                      'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', ...
                                      'MarkerSize', 8);
            end

            obj.h_title = title(obj.ax, '', 'Color', 'w');
        end

        function update(obj, swarm, ~, t)
            % update(swarm, env, t) — push new positions to existing handles
            n = min(obj.N, swarm.N);
            for i = 1:n
                p = swarm.agents{i}.position;
                addpoints(obj.h_trail(i), p(1), p(2));
                set(obj.h_agent(i), 'XData', p(1), 'YData', p(2));
            end
            set(obj.h_title, 'String', sprintf('t = %.2f s  |  N = %d', t, swarm.N));
        end
    end

    methods (Access = private)
        function draw_obstacles(obj, env)
            % Render all obstacles once as filled patches.
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    th = linspace(0, 2*pi, 60);
                    fill(obj.ax, obs.center(1) + obs.radius*cos(th), ...
                                 obs.center(2) + obs.radius*sin(th), ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                elseif strcmp(obs.type, 'rect')
                    xr = obs.x_range; yr = obs.y_range;
                    fill(obj.ax, [xr(1) xr(2) xr(2) xr(1)], ...
                                 [yr(1) yr(1) yr(2) yr(2)], ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                end
            end
        end
    end
end
