classdef AStar < handle
    %% AStar — A* grid-based path planner
    %
    % Discretises the environment into a grid and finds the shortest
    % collision-free path using A* with Euclidean heuristic. Supports
    % 8-directional movement (including diagonals).
    %
    % Optimal within grid resolution. Better suited than RRT when a short,
    % clean path is needed; slower to initialise in large environments.
    %
    % Example:
    %   planner = AStar(env, 1.0);    % 1 m grid resolution
    %   path    = planner.plan(start, goal);
    %   behav   = PathFollowing(path);

    properties
        env                         % Environment reference
        grid_resolution = 1.0;      % cell size in metres
        grid_size                   % [nx, ny] grid dimensions
        obstacles_grid              % nx×ny logical obstacle mask
    end

    methods
        function obj = AStar(environment, resolution)
            % AStar(environment, resolution)
            %   environment : Environment object
            %   resolution  : grid cell size in metres (default 1.0)
            obj.env = environment;
            if nargin > 1, obj.grid_resolution = resolution; end
            obj.build_grid();
        end

        function path = plan(obj, start_pos, goal_pos)
            % plan(start_pos, goal_pos) — find shortest collision-free grid path
            %   start_pos : [x; y] start position
            %   goal_pos  : [x; y] goal position
            %   Returns   : 2×M waypoint matrix, or [] if no path exists
            start_idx = obj.pos_to_grid(start_pos);
            goal_idx  = obj.pos_to_grid(goal_pos);

            if obj.obstacles_grid(start_idx(1), start_idx(2)) || ...
               obj.obstacles_grid(goal_idx(1),  goal_idx(2))
                path = [];
                warning('AStar:StartGoalInObstacle', 'Start or goal lies inside an obstacle.');
                return;
            end

            open_set   = [start_idx, 0, obj.heuristic(start_idx, goal_idx)];
            closed_set = false(obj.grid_size);
            came_from  = zeros([obj.grid_size, 2]);
            g_score    = inf(obj.grid_size);
            g_score(start_idx(1), start_idx(2)) = 0;

            while size(open_set, 1) > 0
                [~, min_idx] = min(open_set(:,4));
                current = open_set(min_idx, 1:2);

                if all(current == goal_idx)
                    path = obj.reconstruct_path(came_from, current);
                    return;
                end

                open_set(min_idx, :) = [];
                closed_set(current(1), current(2)) = true;

                nbrs = obj.get_neighbors(current);
                for ni = 1:size(nbrs, 1)
                    neighbor = nbrs(ni, :);
                    if closed_set(neighbor(1), neighbor(2)), continue; end

                    tentative_g = g_score(current(1), current(2)) + norm(neighbor - current);

                    if tentative_g < g_score(neighbor(1), neighbor(2))
                        came_from(neighbor(1), neighbor(2), :) = current;
                        g_score(neighbor(1), neighbor(2))      = tentative_g;
                        f_score = tentative_g + obj.heuristic(neighbor, goal_idx);

                        in_open = false;
                        for m = 1:size(open_set, 1)
                            if all(open_set(m, 1:2) == neighbor)
                                open_set(m, 3:4) = [tentative_g, f_score];
                                in_open = true;
                                break;
                            end
                        end
                        if ~in_open
                            open_set = [open_set; neighbor, tentative_g, f_score];
                        end
                    end
                end
            end

            path = [];
            warning('AStar:NoPath', 'A* found no path from start to goal.');
        end
    end

    methods (Access = private)
        function build_grid(obj)
            nx = ceil((obj.env.x_lim(2) - obj.env.x_lim(1)) / obj.grid_resolution);
            ny = ceil((obj.env.y_lim(2) - obj.env.y_lim(1)) / obj.grid_resolution);
            obj.grid_size      = [nx, ny];
            obj.obstacles_grid = false(nx, ny);

            for i = 1:nx
                for j = 1:ny
                    x = obj.env.x_lim(1) + (i-0.5) * obj.grid_resolution;
                    y = obj.env.y_lim(1) + (j-0.5) * obj.grid_resolution;
                    if obj.env.in_collision([x; y])
                        obj.obstacles_grid(i,j) = true;
                    end
                end
            end
        end

        function idx = pos_to_grid(obj, pos)
            i   = round((pos(1) - obj.env.x_lim(1)) / obj.grid_resolution) + 1;
            j   = round((pos(2) - obj.env.y_lim(1)) / obj.grid_resolution) + 1;
            idx = [max(1, min(i, obj.grid_size(1))), max(1, min(j, obj.grid_size(2)))];
        end

        function pos = grid_to_pos(obj, idx)
            x = obj.env.x_lim(1) + (idx(1)-0.5) * obj.grid_resolution;
            y = obj.env.y_lim(1) + (idx(2)-0.5) * obj.grid_resolution;
            pos = [x; y];
        end

        function h = heuristic(~, idx1, idx2)
            h = norm(idx1 - idx2);
        end

        function neighbors = get_neighbors(obj, idx)
            directions = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
            neighbors  = [];
            for k = 1:size(directions, 1)
                n = idx + directions(k,:);
                if n(1) >= 1 && n(1) <= obj.grid_size(1) && ...
                   n(2) >= 1 && n(2) <= obj.grid_size(2) && ...
                   ~obj.obstacles_grid(n(1), n(2))
                    neighbors = [neighbors; n];
                end
            end
        end

        function path = reconstruct_path(obj, came_from, current)
            path_idx = current;
            while any(came_from(current(1), current(2), :))
                current  = squeeze(came_from(current(1), current(2), :))';
                path_idx = [current; path_idx];
            end
            path = zeros(2, size(path_idx, 1));
            for k = 1:size(path_idx, 1)
                path(:,k) = obj.grid_to_pos(path_idx(k,:));
            end
        end
    end
end
