classdef Environment < handle
    %% Environment — 2D world with rectangular boundaries and obstacles
    %
    % Obstacles are stored as structs and queried by in_collision() and
    % in_bounds(). The planner classes (PotentialField, RRT, AStar) and
    % SimEngine both use this class to check feasibility.
    %
    % Example:
    %   env = Environment([-50, 50], [-50, 50]);
    %   env.add_circular_obstacle([10, 0], 5);
    %   env.add_rectangular_obstacle([20, 30], [-10, 10]);
    %   env.goal = [40; 40];

    properties
        x_lim       % [xmin, xmax] world boundary in x
        y_lim       % [ymin, ymax] world boundary in y
        obstacles   % cell array of obstacle structs
        goal        % optional [x; y] goal position
    end

    methods
        function obj = Environment(x_lim, y_lim)
            % Environment(x_lim, y_lim)
            %   x_lim : [xmin, xmax]
            %   y_lim : [ymin, ymax]
            obj.x_lim     = x_lim;
            obj.y_lim     = y_lim;
            obj.obstacles = {};
            obj.goal      = [];
        end

        function add_circular_obstacle(obj, center, radius)
            % add_circular_obstacle(center, radius)
            %   center : [x, y] obstacle center
            %   radius : scalar radius in metres
            obs.type   = 'circle';
            obs.center = center;
            obs.radius = radius;
            obj.obstacles{end+1} = obs;
        end

        function add_rectangular_obstacle(obj, x_range, y_range)
            % add_rectangular_obstacle(x_range, y_range)
            %   x_range : [xmin, xmax]
            %   y_range : [ymin, ymax]
            obs.type    = 'rect';
            obs.x_range = x_range;
            obs.y_range = y_range;
            obj.obstacles{end+1} = obs;
        end

        function result = in_collision(obj, position)
            % in_collision(position) — true if position is inside any obstacle
            %   position : [x; y] column vector
            result = false;
            for k = 1:length(obj.obstacles)
                obs = obj.obstacles{k};
                if strcmp(obs.type, 'circle')
                    if norm(position(:) - obs.center(:)) <= obs.radius
                        result = true; return;
                    end
                elseif strcmp(obs.type, 'rect')
                    if position(1) >= obs.x_range(1) && position(1) <= obs.x_range(2) && ...
                       position(2) >= obs.y_range(1) && position(2) <= obs.y_range(2)
                        result = true; return;
                    end
                end
            end
        end

        function result = in_bounds(obj, position)
            % in_bounds(position) — true if position is within world boundaries
            %   position : [x; y] column vector
            result = position(1) >= obj.x_lim(1) && position(1) <= obj.x_lim(2) && ...
                     position(2) >= obj.y_lim(1) && position(2) <= obj.y_lim(2);
        end
    end
end
