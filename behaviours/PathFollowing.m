classdef PathFollowing < handle
    %% PathFollowing — track a pre-planned waypoint path
    %
    % Each agent independently follows the same path using a lookahead point
    % strategy: find the furthest path waypoint at least lookahead_dist ahead,
    % then steer toward it. Works with point-mass and Unicycle dynamics.
    %
    % Typically used after a planner (PotentialField, RRT, AStar) generates
    % the path. All agents share the same path but start from different
    % initial positions, converging onto it at their own pace.
    %
    % Works with: SingleIntegrator, DoubleIntegrator, Unicycle
    %
    % Example:
    %   path  = PotentialField(env).plan(start, goal);
    %   behav = PathFollowing(path, 3.0);
    %   sim   = SimEngine(swarm, env, behav);

    properties
        path                    % 2×M waypoint matrix
        lookahead_dist = 3.0;   % lookahead distance (metres)
        k_v            = 1.0;   % linear speed gain
        k_omega        = 2.0;   % angular gain (Unicycle only)
        max_speed      = 3.0;   % maximum speed for point-mass robots (m/s)
        max_omega      = pi;    % maximum angular velocity for Unicycle (rad/s)
    end

    methods
        function obj = PathFollowing(path, lookahead)
            % PathFollowing(path, lookahead)
            %   path      : 2×M waypoint matrix from a planner
            %   lookahead : lookahead distance in metres (default 3.0)
            obj.path = path;
            if nargin > 1, obj.lookahead_dist = lookahead; end
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns control_dim×N control matrix
            if swarm.N > 0 && isprop(swarm.agents{1}.dynamics, 'control_dim')
                ctrl_dim = swarm.agents{1}.dynamics.control_dim;
            else
                ctrl_dim = 2;
            end
            u = zeros(ctrl_dim, swarm.N);

            for i = 1:swarm.N
                agent  = swarm.agents{i};
                pos    = agent.state(1:2);
                target = obj.get_lookahead_point(pos);

                if isa(agent.dynamics, 'Unicycle')
                    theta         = agent.state(3);
                    to_target     = target - pos;
                    desired_angle = atan2(to_target(2), to_target(1));
                    angle_error   = atan2(sin(desired_angle - theta), cos(desired_angle - theta));

                    v     = max(-obj.max_speed, min(obj.k_v * norm(to_target),  obj.max_speed));
                    omega = max(-obj.max_omega, min(obj.k_omega * angle_error, obj.max_omega));
                    u(:,i) = [v; omega];
                else
                    to_target = target - pos;
                    control   = obj.k_v * to_target;
                    if norm(control) > obj.max_speed
                        control = control / norm(control) * obj.max_speed;
                    end
                    u(:,i) = control;
                end
            end
        end

        function target = get_lookahead_point(obj, pos)
            % get_lookahead_point(pos) — returns the next waypoint to steer toward
            if isempty(obj.path)
                target = pos;
                return;
            end

            dists = vecnorm(obj.path - pos);
            [~, closest_idx] = min(dists);

            for k = closest_idx:size(obj.path, 2)
                if norm(obj.path(:,k) - pos) >= obj.lookahead_dist
                    target = obj.path(:,k);
                    return;
                end
            end

            target = obj.path(:, end);
        end
    end
end
