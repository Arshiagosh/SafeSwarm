classdef RRT < handle
    %% RRT — Rapidly-exploring Random Tree path planner
    %
    % Builds a tree by sampling random points in free space and extending
    % the nearest tree vertex toward each sample. Probabilistically complete:
    % given enough iterations, will find a path if one exists.
    %
    % Example:
    %   planner = RRT(env, 5000, 2.0);
    %   path    = planner.plan(start, goal, 2.0);
    %   behav   = PathFollowing(path);

    properties
        env                         % Environment reference
        max_iter         = 5000;    % iteration budget
        step_size        = 2.0;     % maximum extension step (metres)
        goal_sample_rate = 0.1;     % probability of sampling the goal directly
    end

    methods
        function obj = RRT(environment, max_iter, step_size)
            % RRT(environment, max_iter, step_size)
            %   environment : Environment object
            %   max_iter    : iteration budget (default 5000)
            %   step_size   : maximum tree extension per step (default 2.0 m)
            obj.env = environment;
            if nargin > 1, obj.max_iter  = max_iter;  end
            if nargin > 2, obj.step_size = step_size; end
        end

        function [path, tree] = plan(obj, start_pos, goal_pos, goal_radius)
            % plan(start_pos, goal_pos, goal_radius) — compute a collision-free path
            %   start_pos   : [x; y] start position
            %   goal_pos    : [x; y] goal position
            %   goal_radius : distance threshold to declare goal reached (default 2.0)
            %   Returns     : path — 2×M waypoint matrix; tree — struct with vertices/parents
            if nargin < 4, goal_radius = 2.0; end

            tree.vertices = start_pos(:);
            tree.parents  = 0;

            for iter = 1:obj.max_iter
                if rand < obj.goal_sample_rate
                    rand_point = goal_pos(:);
                else
                    rand_point = obj.sample_free();
                end

                nearest_idx   = obj.nearest(tree, rand_point);
                nearest_point = tree.vertices(:, nearest_idx);
                new_point     = obj.steer(nearest_point, rand_point);

                if ~obj.env.in_collision(new_point) && ...
                   ~obj.path_has_collision(nearest_point, new_point)
                    tree.vertices = [tree.vertices, new_point];
                    tree.parents  = [tree.parents, nearest_idx];

                    if norm(new_point - goal_pos(:)) < goal_radius
                        path = obj.extract_path(tree, length(tree.parents));
                        return;
                    end
                end
            end

            path = [];
            warning('RRT:MaxIterReached', 'Max iterations reached without finding a path.');
        end
    end

    methods (Access = private)
        function point = sample_free(obj)
            x = obj.env.x_lim(1) + rand * (obj.env.x_lim(2) - obj.env.x_lim(1));
            y = obj.env.y_lim(1) + rand * (obj.env.y_lim(2) - obj.env.y_lim(1));
            point = [x; y];
        end

        function idx = nearest(~, tree, point)
            dists = vecnorm(tree.vertices - point);
            [~, idx] = min(dists);
        end

        function new_point = steer(obj, from_point, to_point)
            direction = to_point - from_point;
            d = norm(direction);
            if d < obj.step_size
                new_point = to_point;
            else
                new_point = from_point + (direction/d) * obj.step_size;
            end
        end

        function result = path_has_collision(obj, p1, p2)
            % Returns true if the straight-line segment p1→p2 passes through an obstacle.
            n_checks = ceil(norm(p2 - p1) / 0.5);
            result   = false;
            for k = 1:n_checks
                alpha = k / n_checks;
                if obj.env.in_collision(p1 + alpha * (p2 - p1))
                    result = true; return;
                end
            end
        end

        function path = extract_path(~, tree, goal_idx)
            path        = tree.vertices(:, goal_idx);
            current_idx = goal_idx;
            while tree.parents(current_idx) ~= 0
                current_idx = tree.parents(current_idx);
                path = [tree.vertices(:, current_idx), path];
            end
        end
    end
end
