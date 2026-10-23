classdef SwarmVisualizer < handle
    %% SwarmVisualizer — 2D animation of the swarm, with optional GIF export
    %
    % Graphics handles (trails, agent markers, obstacles) are created once in
    % the constructor; update() then only sets their data each step. This
    % keeps redraws fast even for hundreds of agents and long runs.
    %
    % Options (name-value pairs):
    %   'Theme'    — 'dark' (default, as in SwarmSim) or 'light'
    %   'Visible'  — 'on' (default) or 'off' for batch runs
    %   'GifFile'  — write an animated GIF to this file while running
    %   'GifEvery' — keep one frame in this many updates (default 4)
    %   'GifDelay' — seconds per GIF frame (default 0.05)
    %   'Title'    — text shown before the time stamp
    %
    % When swarm.agent_radius > 0 each agent is drawn as a disc of that
    % radius, so contacts are visible in the animation.
    %
    % Example:
    %   viz            = SwarmVisualizer(swarm, env, 'GifFile', 'figures/run.gif');
    %   sim.visualizer = viz;
    %   sim.run();
    %   viz.close();

    properties
        fig
        ax
        trail_length = 40;  % number of past positions shown per agent
        N                   % number of agents (fixed at construction)
        h_trail             % 1×N animatedline handles (position trails)
        h_agent             % 1×N handles (markers, or disc patches)
        h_obstacle = {};    % handles of drawn obstacles (redrawn when obstacles move)
        h_title             % title text handle
        radius   = 0;       % disc radius drawn per agent (from swarm.agent_radius)
        gif_file = '';
        gif_every = 4;
        gif_delay = 0.05;
        gif_count = 0;
        label    = '';
        fg       = 'w';     % text and axis colour
    end

    properties (Constant, Access = private)
        DISC = [cos(linspace(0, 2*pi, 24)); sin(linspace(0, 2*pi, 24))];
    end

    methods
        function obj = SwarmVisualizer(swarm, env, varargin)
            % SwarmVisualizer(swarm, env, Name, Value, ...)
            %   swarm : Swarm object (sets N and initial positions)
            %   env   : Environment object (sets axis limits and obstacles)
            p = inputParser;
            p.addParameter('Theme',    'dark');
            p.addParameter('Visible',  'on');
            p.addParameter('GifFile',  '');
            p.addParameter('GifEvery', 4);
            p.addParameter('GifDelay', 0.05);
            p.addParameter('Title',    '');
            p.parse(varargin{:});
            o = p.Results;

            obj.N         = swarm.N;
            obj.radius    = swarm.agent_radius;
            obj.gif_file  = o.GifFile;
            obj.gif_every = o.GifEvery;
            obj.gif_delay = o.GifDelay;
            obj.label     = o.Title;

            if strcmp(o.Theme, 'light')
                bg = 'w'; obj.fg = 'k'; grid_c = [0.85 0.85 0.85];
            else
                bg = 'k'; obj.fg = 'w'; grid_c = [0.3 0.3 0.3];
            end

            obj.fig = figure('Name', 'SafeSwarm', 'Color', bg, 'Visible', o.Visible, ...
                             'Position', [100 100 640 640]);
            obj.ax  = axes('Parent', obj.fig, 'Color', bg, ...
                           'XColor', obj.fg, 'YColor', obj.fg);
            axis(obj.ax, 'equal');
            axis(obj.ax, [env.x_lim(1) env.x_lim(2) env.y_lim(1) env.y_lim(2)]);
            hold(obj.ax, 'on');
            grid(obj.ax, 'on');
            obj.ax.GridColor = grid_c;
            xlabel(obj.ax, 'x (m)', 'Color', obj.fg, 'Interpreter', 'none');
            ylabel(obj.ax, 'y (m)', 'Color', obj.fg, 'Interpreter', 'none');

            obj.draw_obstacles(env);
            if ~isempty(env.goal)
                plot(obj.ax, env.goal(1), env.goal(2), 'g*', ...
                     'MarkerSize', 15, 'LineWidth', 2);
            end

            obj.h_trail = gobjects(1, obj.N);
            obj.h_agent = gobjects(1, obj.N);
            for i = 1:obj.N
                c = swarm.agents{i}.color;
                q = swarm.agents{i}.position;
                obj.h_trail(i) = animatedline(obj.ax, 'Color', c, 'LineWidth', 1, ...
                                              'MaximumNumPoints', obj.trail_length);
                addpoints(obj.h_trail(i), q(1), q(2));
                if obj.radius > 0
                    obj.h_agent(i) = patch(obj.ax, q(1) + obj.radius*obj.DISC(1,:), ...
                                           q(2) + obj.radius*obj.DISC(2,:), c, ...
                                           'EdgeColor', obj.fg, 'LineWidth', 0.5);
                else
                    obj.h_agent(i) = plot(obj.ax, q(1), q(2), 'o', ...
                                          'MarkerFaceColor', c, 'MarkerEdgeColor', obj.fg, ...
                                          'MarkerSize', 8);
                end
            end

            obj.h_title = title(obj.ax, '', 'Color', obj.fg, 'Interpreter', 'none');
        end

        function update(obj, swarm, env, t)
            % update(swarm, env, t) — push new positions to existing handles
            n = min(obj.N, swarm.N);
            for i = 1:n
                q = swarm.agents{i}.position;
                addpoints(obj.h_trail(i), q(1), q(2));
                if obj.radius > 0
                    set(obj.h_agent(i), 'XData', q(1) + obj.radius*obj.DISC(1,:), ...
                                        'YData', q(2) + obj.radius*obj.DISC(2,:));
                else
                    set(obj.h_agent(i), 'XData', q(1), 'YData', q(2));
                end
            end
            if nargin >= 3 && ~isempty(env) && ~isempty(env.dynamic_obstacles)
                obj.draw_obstacles(env);
            end
            txt = sprintf('t = %.2f s  |  N = %d', t, swarm.N);
            if ~isempty(obj.label), txt = [obj.label '  |  ' txt]; end
            set(obj.h_title, 'String', txt);
            drawnow limitrate;
            obj.capture_frame(false);
        end

        function capture_frame(obj, force)
            % capture_frame(force) — append the current figure to the GIF
            %   Frames are kept every gif_every calls unless force is true.
            if isempty(obj.gif_file), return; end
            obj.gif_count = obj.gif_count + 1;
            if ~force && obj.gif_count > 1 && mod(obj.gif_count - 1, obj.gif_every) ~= 0
                return;
            end
            drawnow;
            frame = getframe(obj.fig);
            [img, cmap] = rgb2ind(frame2im(frame), 128);
            if obj.gif_count == 1
                imwrite(img, cmap, obj.gif_file, 'gif', 'LoopCount', inf, ...
                        'DelayTime', obj.gif_delay);
            else
                imwrite(img, cmap, obj.gif_file, 'gif', 'WriteMode', 'append', ...
                        'DelayTime', obj.gif_delay);
            end
        end

        function close(obj)
            % close() — close the figure
            if isgraphics(obj.fig), close(obj.fig); end
        end
    end

    methods (Access = private)
        function draw_obstacles(obj, env)
            % Render all obstacles as filled patches (replacing earlier ones).
            for k = 1:numel(obj.h_obstacle)
                if isgraphics(obj.h_obstacle{k}), delete(obj.h_obstacle{k}); end
            end
            obj.h_obstacle = {};
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    th = linspace(0, 2*pi, 60);
                    obj.h_obstacle{end+1} = fill(obj.ax, ...
                        obs.center(1) + obs.radius*cos(th), ...
                        obs.center(2) + obs.radius*sin(th), ...
                        [0.6 0.2 0.2], 'EdgeColor', 'none');
                elseif strcmp(obs.type, 'rect')
                    xr = obs.x_range; yr = obs.y_range;
                    obj.h_obstacle{end+1} = fill(obj.ax, [xr(1) xr(2) xr(2) xr(1)], ...
                        [yr(1) yr(1) yr(2) yr(2)], [0.6 0.2 0.2], 'EdgeColor', 'none');
                end
            end
        end
    end
end
