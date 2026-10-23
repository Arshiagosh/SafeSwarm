classdef FormationWithObstacles < handle
    %% FormationWithObstacles — formation behaviour wrapper with obstacle avoidance
    %
    % Wraps any formation behaviour and adds a potential-field repulsive force
    % from nearby Environment obstacles. Supports both 'circle' and 'rect'
    % obstacle types as defined by Environment.add_*_obstacle().
    %
    % Works with: any formation behaviour (LeaderFollower, VirtualStructure, etc.)
    % Use case:   navigate rigid or hierarchical formations through cluttered spaces
    %
    % Example:
    %   base  = LeaderFollower(1, offsets);
    %   behav = FormationWithObstacles(base, 50.0, 3.0);
    %   sim   = SimEngine(swarm, env, behav);

    properties
        formation_controller        % any behaviour implementing compute_control
        k_obstacle = 50.0;         % obstacle repulsion gain
        d_safe     = 3.0;          % obstacle influence distance (metres)
        max_accel  = 3.0;          % control saturation (m/s²)
    end

    methods
        function obj = FormationWithObstacles(formation_ctrl, k_obs, d_safe)
            % FormationWithObstacles(formation_ctrl, k_obs, d_safe)
            %   formation_ctrl : behaviour object
            %   k_obs          : repulsion gain (default 50.0)
            %   d_safe         : influence distance in metres (default 3.0)
            obj.formation_controller = formation_ctrl;
            if nargin > 1, obj.k_obstacle = k_obs;  end
            if nargin > 2, obj.d_safe     = d_safe; end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — formation control plus obstacle repulsion
            u = obj.formation_controller.compute_control(swarm, env);

            for i = 1:swarm.N
                pos      = swarm.agents{i}.state(1:2);
                f_obs    = obj.obstacle_force(pos, env);
                u(:,i)   = u(:,i) + f_obs;
                if norm(u(:,i)) > obj.max_accel
                    u(:,i) = u(:,i) / norm(u(:,i)) * obj.max_accel;
                end
            end
        end
    end

    methods (Access = private)
        function f = obstacle_force(obj, pos, env)
            % Potential-field repulsion from all environment obstacles.
            f = zeros(2, 1);
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    diff = pos(:) - obs.center(:);
                    d    = norm(diff) - obs.radius;
                    if d < obj.d_safe && d > 0
                        f = f + obj.k_obstacle * (1/d - 1/obj.d_safe) * (diff/norm(diff));
                    end

                elseif strcmp(obs.type, 'rect')
                    % Clamp to nearest point on rectangle surface
                    closest = [max(obs.x_range(1), min(pos(1), obs.x_range(2)));
                               max(obs.y_range(1), min(pos(2), obs.y_range(2)))];
                    diff = pos(:) - closest;
                    d    = norm(diff);
                    if d < obj.d_safe && d > 0.1
                        f = f + obj.k_obstacle * (1/d - 1/obj.d_safe) * (diff/d);
                    end
                end
            end
        end
    end
end
