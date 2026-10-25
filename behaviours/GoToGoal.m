classdef GoToGoal < handle
    %% GoToGoal — every agent drives straight to its own goal
    %
    % The nominal (unsafe) controller of the safety benchmarks:
    %   SingleIntegrator: u_i = k_p (g_i - p_i)
    %   DoubleIntegrator: u_i = k_p (g_i - p_i) - k_d v_i
    %   Unicycle:         v_i = k_p ||g_i - p_i|| cos(e_i),  omega_i = k_w e_i,
    %                     e_i = heading error to the goal
    % Agents ignore each other, so collisions are expected without a filter.
    %
    % Works with: SingleIntegrator, DoubleIntegrator, Unicycle
    %
    % Example:
    %   behav = GoToGoal(goals, 1.0);
    %   safe  = SafetyFilter(behav);

    properties
        goals           % 2×N goal positions
        k_p = 1.0;      % position gain
        k_d = 2.0;      % velocity damping (DoubleIntegrator)
        k_w = 2.0;      % heading gain (Unicycle)
    end

    methods
        function obj = GoToGoal(goals, k_p)
            % GoToGoal(goals, k_p)
            %   goals : 2×N matrix
            %   k_p   : position gain (default 1.0)
            obj.goals = goals;
            if nargin > 1, obj.k_p = k_p; end
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            S = swarm.get_states();
            E = obj.goals - S(1:2, :);
            switch size(S, 1)
                case 2
                    u = obj.k_p * E;
                case 4
                    u = obj.k_p * E - obj.k_d * S(3:4, :);
                case 3
                    e = atan2(E(2,:), E(1,:)) - S(3,:);
                    e = atan2(sin(e), cos(e));
                    u = [obj.k_p * vecnorm(E) .* max(cos(e), 0); obj.k_w * e];
            end
        end

        function r = reached(obj, swarm, tol)
            % reached(swarm, tol) — 1×N logical, agent within tol of its goal
            r = vecnorm(swarm.get_positions() - obj.goals) <= tol;
        end
    end
end
