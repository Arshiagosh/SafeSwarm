classdef Flocking < handle
    %% Flocking — Reynolds boids: separation + alignment + cohesion
    %
    % Implements the classic Reynolds (1987) flocking model. Each agent
    % combines three weighted forces from its local communication neighbours:
    %
    %   Separation — repel from agents closer than d_sep (avoid collisions)
    %   Alignment  — match the average velocity of neighbours
    %   Cohesion   — move toward the local neighbourhood centroid
    %
    % Works with: DoubleIntegrator (velocity state needed for alignment)
    % Topology:   'metric' or 'knn' recommended (comm range defines neighbourhood)
    %
    % Example:
    %   behav = Flocking(2.0, 1.0, 1.0, 3.0);   % sep, ali, coh weights; sep dist
    %   sim   = SimEngine(swarm, env, behav);

    properties
        w_sep = 2.0;    % separation weight
        w_ali = 1.0;    % alignment weight
        w_coh = 1.0;    % cohesion weight
        d_sep = 3.0;    % separation distance (metres); repulsion activates below this
    end

    methods
        function obj = Flocking(w_sep, w_ali, w_coh, d_sep)
            % Flocking(w_sep, w_ali, w_coh, d_sep)
            %   w_sep : separation weight (default 2.0)
            %   w_ali : alignment weight  (default 1.0)
            %   w_coh : cohesion weight   (default 1.0)
            %   d_sep : separation distance in metres (default 3.0)
            if nargin > 0, obj.w_sep = w_sep; end
            if nargin > 1, obj.w_ali = w_ali; end
            if nargin > 2, obj.w_coh = w_coh; end
            if nargin > 3, obj.d_sep = d_sep; end
        end

        function u_all = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            P = swarm.get_positions();      % 2×N
            V = swarm.get_velocities();     % 2×N
            A = swarm.get_adjacency();      % N×N (symmetric)

            deg     = sum(A, 2).';          % 1×N neighbour counts
            has     = deg > 0;
            degsafe = deg; degsafe(~has) = 1;   % avoid divide-by-zero

            % Cohesion: mean neighbour position minus own position
            f_coh = (P * A) ./ degsafe - P;
            % Alignment: mean neighbour velocity minus own velocity
            f_ali = (V * A) ./ degsafe - V;

            % Separation: repel from neighbours closer than d_sep
            DX = P(1,:).' - P(1,:);         % DX(i,j) = x_i - x_j
            DY = P(2,:).' - P(2,:);
            D  = sqrt(DX.^2 + DY.^2);
            mask = (A > 0) & (D < obj.d_sep) & (D > 1e-6);
            coef = zeros(size(D));
            coef(mask) = (obj.d_sep - D(mask)) / obj.d_sep ./ D(mask);
            f_sep = [sum(coef .* DX, 2).'; sum(coef .* DY, 2).'];

            u_all = obj.w_sep * f_sep + obj.w_ali * f_ali + obj.w_coh * f_coh;
            u_all(:, ~has) = 0;             % agents with no neighbours hold still
        end
    end
end
