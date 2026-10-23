classdef CollisionAvoidance < handle
    %% CollisionAvoidance — behaviour wrapper adding inter-agent collision avoidance
    %
    % Wraps any base behaviour and adds a potential-field repulsive force
    % between agents that come within d_safe of each other. The combined
    % control is clamped to max_u to prevent runaway accelerations.
    %
    % Example:
    %   base = Flocking();
    %   safe = CollisionAvoidance(base, 2.0, 30.0);
    %   sim  = SimEngine(swarm, env, safe);

    properties
        base_behavior   % any behaviour implementing compute_control(swarm, env)
        d_safe = 2.0;   % minimum safe inter-agent distance (metres)
        k_rep  = 30.0;  % repulsion gain
        max_u  = 5.0;   % maximum control magnitude (m/s or m/s²)
    end

    methods
        function obj = CollisionAvoidance(base_behavior, d_safe, k_rep)
            % CollisionAvoidance(base_behavior, d_safe, k_rep)
            %   base_behavior : behaviour object
            %   d_safe        : safe distance in metres (default 2.0)
            %   k_rep         : repulsion gain (default 30.0)
            obj.base_behavior = base_behavior;
            if nargin > 1, obj.d_safe = d_safe; end
            if nargin > 2, obj.k_rep  = k_rep;  end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — base control plus inter-agent repulsion
            u = obj.base_behavior.compute_control(swarm, env);

            P = swarm.get_positions();      % 2×N
            % Pairwise displacements: DX(i,j) = x_i - x_j
            DX = P(1,:).' - P(1,:);
            DY = P(2,:).' - P(2,:);
            D  = sqrt(DX.^2 + DY.^2);

            mask = (D < obj.d_safe) & (D > 1e-3);
            coef = zeros(size(D));
            coef(mask) = obj.k_rep * (1 ./ D(mask) - 1/obj.d_safe) ./ D(mask);

            % Add summed repulsion to the (spatial) control of every agent
            u(1,:) = u(1,:) + sum(coef .* DX, 2).';
            u(2,:) = u(2,:) + sum(coef .* DY, 2).';

            % Clamp control magnitude per agent
            nrm  = vecnorm(u, 2, 1);
            over = nrm > obj.max_u;
            u(:, over) = u(:, over) ./ nrm(over) * obj.max_u;
        end
    end
end
