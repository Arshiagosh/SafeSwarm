classdef DisplacementFormation < handle
    %% DisplacementFormation — distributed displacement-based formation control
    %
    % Each agent only uses the relative positions (and velocities) of its
    % neighbours. The desired shape is given by target positions p*_i; the
    % formation is reached up to a common translation. With graph weights
    % a_ij this is the consensus-type law of Oh, Park and Ahn,
    % "A survey of multi-agent formation control", Automatica 53, 2015:
    %
    %   SingleIntegrator: u_i = k_p sum_j a_ij ((p_j - p_i) - (p*_j - p*_i)) + v_ref
    %   DoubleIntegrator: u_i = k_p sum_j a_ij ((p_j - p_i) - (p*_j - p*_i))
    %                         + k_v sum_j a_ij (v_j - v_i) - k_d (v_i - v_ref)
    %
    % The graph is the swarm's communication graph unless a fixed adjacency
    % matrix is given. A connected graph is needed for convergence.
    %
    % Works with: SingleIntegrator, DoubleIntegrator
    %
    % Example:
    %   shape = 4 * [cos(2*pi*(1:6)/6); sin(2*pi*(1:6)/6)];   % hexagon
    %   behav = DisplacementFormation(shape);
    %   sim   = SimEngine(swarm, env, behav);

    properties
        target_positions        % 2×N desired positions p*_i (any translation)
        adjacency  = [];        % fixed N×N weights; empty = use swarm.get_adjacency()
        k_p   = 1.0;            % position consensus gain
        k_v   = 1.0;            % velocity consensus gain (DoubleIntegrator)
        k_d   = 1.0;            % velocity damping gain (DoubleIntegrator)
        v_ref = [0; 0];         % common reference velocity for the formation
    end

    methods
        function obj = DisplacementFormation(target_positions, adjacency, k_p)
            % DisplacementFormation(target_positions, adjacency, k_p)
            %   target_positions : 2×N matrix
            %   adjacency        : N×N fixed graph (default: communication graph)
            %   k_p              : position gain (default 1.0)
            obj.target_positions = target_positions;
            if nargin > 1, obj.adjacency = adjacency; end
            if nargin > 2, obj.k_p       = k_p;       end
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            S = swarm.get_states();
            P = S(1:2, :);
            A = obj.graph(swarm);
            L = diag(sum(A, 2)) - A;
            % sum_j a_ij ((p_j - p_i) - (p*_j - p*_i)) = -(P - P*) L' for symmetric A
            e = (P - obj.target_positions) * L.';
            u = -obj.k_p * e;
            if size(S, 1) == 4
                V = S(3:4, :);
                u = u - obj.k_v * V * L.' - obj.k_d * (V - obj.v_ref(:));
            elseif size(S, 1) == 2
                u = u + obj.v_ref(:);
            else
                error('DisplacementFormation:Dynamics', ...
                      'Only SingleIntegrator and DoubleIntegrator agents are supported.');
            end
        end

        function e = formation_error(obj, swarm)
            % formation_error(swarm) — mean distance to the shape, translation removed
            P  = swarm.get_positions();
            Ps = obj.target_positions;
            D  = (P - mean(P, 2)) - (Ps - mean(Ps, 2));
            e  = mean(vecnorm(D));
        end
    end

    methods (Access = private)
        function A = graph(obj, swarm)
            if isempty(obj.adjacency)
                A = swarm.get_adjacency();
            else
                A = obj.adjacency;
            end
        end
    end
end
