classdef Aggregation < handle
    %% Aggregation — all agents move toward the swarm centroid
    %
    % Implements a simple consensus protocol: each agent computes a force
    % proportional to the displacement from the global centroid. Converges
    % to a single point; useful for gathering before a task begins.
    %
    % Works with: SingleIntegrator, DoubleIntegrator
    % Topology:   any (centroid is computed over all agents, ignoring comm graph)
    %
    % Example:
    %   behav = Aggregation(1.5);
    %   sim   = SimEngine(swarm, env, behav);

    properties
        gain = 1.5;     % proportional gain toward centroid
    end

    methods
        function obj = Aggregation(gain)
            % Aggregation(gain)
            %   gain : proportional gain (default 1.5)
            if nargin > 0, obj.gain = gain; end
        end

        function u_all = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            positions = swarm.get_positions();
            centroid  = mean(positions, 2);
            u_all     = obj.gain * (centroid - positions);   % vectorized over all agents
        end
    end
end
