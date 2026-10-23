classdef Dispersion < handle
    %% Dispersion — agents repel each other to maximize area coverage
    %
    % Each agent computes a repulsive force from every other agent that is
    % closer than min_dist. Force magnitude is proportional to the gap
    % (min_dist - distance), so repulsion weakens as agents spread out.
    %
    % Works with: SingleIntegrator, DoubleIntegrator
    % Use case:   area exploration, uniform coverage, sensor deployment
    %
    % Example:
    %   behav = Dispersion(1.5, 5.0);   % gain=1.5, desired spacing=5 m
    %   sim   = SimEngine(swarm, env, behav);

    properties
        gain     = 1.5;     % repulsion gain
        min_dist = 5.0;     % desired minimum inter-agent distance (metres)
    end

    methods
        function obj = Dispersion(gain, min_dist)
            % Dispersion(gain, min_dist)
            %   gain     : repulsion gain (default 1.5)
            %   min_dist : desired separation in metres (default 5.0)
            if nargin > 0, obj.gain     = gain;     end
            if nargin > 1, obj.min_dist = min_dist; end
        end

        function u_all = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            P = swarm.get_positions();
            % Pairwise displacements: DX(i,j) = x_i - x_j, DY(i,j) = y_i - y_j
            DX = P(1,:).' - P(1,:);
            DY = P(2,:).' - P(2,:);
            D  = sqrt(DX.^2 + DY.^2);

            % Repulsion coefficient for neighbours closer than min_dist
            mask = (D < obj.min_dist) & (D > 1e-6);
            coef = zeros(size(D));
            coef(mask) = obj.gain * (obj.min_dist - D(mask)) ./ D(mask);

            % Sum forces from all other agents (sum over columns j)
            u_all = [sum(coef .* DX, 2).'; sum(coef .* DY, 2).'];
        end
    end
end
