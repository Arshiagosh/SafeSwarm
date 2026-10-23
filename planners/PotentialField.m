classdef PotentialField < handle
    %% PotentialField — artificial potential field path planner
    %
    % Uses gradient descent on a scalar potential: attractive force pulls
    % toward the goal, repulsive forces push away from obstacles. Handles
    % local minima with random perturbations.
    %
    % Supports both 'circle' and 'rect' obstacle types from Environment.
    %
    % Example:
    %   planner = PotentialField(env, 1.0, 100.0, 5.0);
    %   path    = planner.plan([-20;-20], [20;20], 5000, 0.1);
    %   behav   = PathFollowing(path);

    properties
        env                     % Environment reference
        k_att      = 1.0;       % attractive gain
        k_rep      = 100.0;     % repulsive gain
        d0         = 5.0;       % obstacle influence distance (metres)
        max_vel    = 5.0;       % maximum velocity magnitude during planning (m/s)
        inside_rep = 500.0;     % repulsion gain when agent is inside an obstacle
    end

    methods
        function obj = PotentialField(environment, k_att, k_rep, d0)
            % PotentialField(environment, k_att, k_rep, d0)
            %   environment : Environment object
            %   k_att       : attractive gain (default 1.0)
            %   k_rep       : repulsive gain (default 100.0)
            %   d0          : obstacle influence radius in metres (default 5.0)
            obj.env = environment;
            if nargin > 1 && ~isempty(k_att), obj.k_att = k_att; end
            if nargin > 2 && ~isempty(k_rep), obj.k_rep = k_rep; end
            if nargin > 3 && ~isempty(d0),    obj.d0    = d0;    end
        end

        function path = plan(obj, start_pos, goal_pos, max_steps, dt)
            % plan(start_pos, goal_pos, max_steps, dt) — compute a waypoint path
            %   start_pos : [x; y] start position
            %   goal_pos  : [x; y] goal position
            %   max_steps : iteration budget (default 2000)
            %   dt        : integration step size (default 0.1)
            %   Returns   : 2×M waypoint matrix
            if nargin < 4 || isempty(max_steps), max_steps = 2000; end
            if nargin < 5 || isempty(dt),        dt        = 0.1;  end

            pos  = start_pos(:);
            goal = goal_pos(:);

            % Preallocate path buffer; trim to actual length at the end
            path_buf = zeros(2, max_steps + 2);
            n_pts    = 1;
            path_buf(:, 1) = pos;

            stuck_count = 0;
            prev_pos    = pos;

            for step = 1:max_steps
                force = obj.compute_force(pos, goal);
                vel   = force;
                if norm(vel) > obj.max_vel
                    vel = vel / norm(vel) * obj.max_vel;
                end

                new_pos = pos + vel * dt;

                if obj.env.in_collision(new_pos)
                    for scale = [0.5, 0.25, 0.1]
                        test_pos = pos + vel * dt * scale;
                        if ~obj.env.in_collision(test_pos)
                            new_pos = test_pos;
                            break;
                        end
                    end
                    if obj.env.in_collision(new_pos)
                        f_rep = obj.repulsive_force(pos);
                        if norm(f_rep) > 1e-6
                            new_pos = pos + (f_rep/norm(f_rep)) * obj.max_vel * dt * 0.5;
                        end
                    end
                end

                new_pos(1) = max(obj.env.x_lim(1), min(new_pos(1), obj.env.x_lim(2)));
                new_pos(2) = max(obj.env.y_lim(1), min(new_pos(2), obj.env.y_lim(2)));

                pos          = new_pos;
                n_pts        = n_pts + 1;
                path_buf(:, n_pts) = pos;

                if norm(pos - goal) < 1.0, break; end

                if norm(pos - prev_pos) < 1e-4
                    stuck_count = stuck_count + 1;
                    if stuck_count > 50
                        perturb  = randn(2,1) * 2.0;
                        test_pos = pos + perturb;
                        if ~obj.env.in_collision(test_pos)
                            pos   = test_pos;
                            n_pts = n_pts + 1;
                            path_buf(:, n_pts) = pos;
                        end
                        stuck_count = 0;
                    end
                else
                    stuck_count = 0;
                end
                prev_pos = pos;
            end

            path = path_buf(:, 1:n_pts);

            if norm(pos - goal) >= 1.0
                warning('PotentialField:GoalNotReached', ...
                    'Planning stopped without reaching goal (dist=%.2f)', ...
                    norm(pos - goal));
            end
        end

        function force = compute_force(obj, pos, goal)
            % compute_force(pos, goal) — total gradient force at pos
            force = obj.attractive_force(pos, goal) + obj.repulsive_force(pos);
        end
    end

    methods (Access = private)
        function f = attractive_force(obj, pos, goal)
            diff = goal(:) - pos(:);
            if norm(diff) > 0
                f = obj.k_att * diff;
            else
                f = zeros(2,1);
            end
        end

        function f = repulsive_force(obj, pos)
            f   = zeros(2,1);
            pos = pos(:);

            for k = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{k};

                if strcmp(obs.type, 'circle')
                    center = obs.center(:);
                    diff   = pos - center;
                    d_c    = norm(diff);
                    d      = d_c - obs.radius;

                    if d <= 0
                        direction = (d_c > 1e-6) * diff/max(d_c,1e-6) + (d_c <= 1e-6) * [1;0];
                        f = f + obj.inside_rep * direction;
                    elseif d < obj.d0
                        f = f + obj.k_rep * (1/d - 1/obj.d0) / d^2 * (diff/d_c);
                    end

                elseif strcmp(obs.type, 'rect')
                    cx = max(obs.x_range(1), min(pos(1), obs.x_range(2)));
                    cy = max(obs.y_range(1), min(pos(2), obs.y_range(2)));
                    diff = pos - [cx; cy];
                    d    = norm(diff);
                    inside = pos(1) >= obs.x_range(1) && pos(1) <= obs.x_range(2) && ...
                             pos(2) >= obs.y_range(1) && pos(2) <= obs.y_range(2);

                    if inside
                        dists = [pos(1)-obs.x_range(1), obs.x_range(2)-pos(1), ...
                                 pos(2)-obs.y_range(1), obs.y_range(2)-pos(2)];
                        [~, e] = min(dists);
                        dirs   = {[-1;0],[1;0],[0;-1],[0;1]};
                        f = f + obj.inside_rep * dirs{e};
                    elseif d < obj.d0 && d > 1e-6
                        f = f + obj.k_rep * (1/d - 1/obj.d0) / d^2 * (diff/d);
                    end
                end
            end
        end
    end
end
