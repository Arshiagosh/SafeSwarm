classdef DataLogger < handle
    %% DataLogger — records per-step swarm states and performance metrics
    %
    % Attach to SimEngine via sim.logger = DataLogger() before calling run().
    % After the simulation, call plot_metrics() or export_csv() to analyse
    % results. MetricsAnalyzer provides higher-level derived statistics.
    %
    % Metrics logged each step:
    %   centroid     — swarm centre of mass [x; y]
    %   spread       — mean distance of agents from centroid (metres)
    %   connectivity — Fiedler value of graph Laplacian (> 0 means connected)
    %   energy       — total kinetic energy (0.5 * sum(||v||^2))
    %
    % Example:
    %   logger     = DataLogger();
    %   sim.logger = logger;
    %   sim.run();
    %   logger.plot_metrics();
    %   logger.export_csv('results.csv');

    properties
        time_log   = [];    % 1×T vector of timestamps
        state_log  = {};    % {t} → [state_dim × N] matrix
        metric_log = struct('centroid', {}, 'spread', {}, ...
                            'connectivity', {}, 'energy', {});
        log_count  = 0;
    end

    methods
        function log(obj, t, swarm)
            % log(t, swarm) — record state and metrics at time t
            obj.log_count      = obj.log_count + 1;
            obj.time_log(end+1) = t;

            states = cell2mat(cellfun(@(a) a.state, ...
                              swarm.agents, 'UniformOutput', false));
            obj.state_log{end+1} = states;

            % Spread: mean distance from centroid
            positions = states(1:2, :);
            centroid  = mean(positions, 2);
            spread    = mean(vecnorm(positions - centroid));

            % Connectivity: Fiedler eigenvalue of the graph Laplacian
            A    = swarm.get_adjacency();
            D    = diag(sum(A, 2));
            L    = D - A;
            evs  = sort(real(eig(L)));
            connectivity = evs(min(2, length(evs)));  % 2nd smallest eigenvalue

            % Kinetic energy: only meaningful for states with velocity (dim >= 4)
            state_dim = size(states, 1);
            if state_dim >= 4
                velocities = states(3:4, :);
                energy = 0.5 * sum(vecnorm(velocities).^2);
            else
                energy = 0;
            end

            obj.metric_log(end+1) = struct( ...
                'centroid',     centroid, ...
                'spread',       spread, ...
                'connectivity', connectivity, ...
                'energy',       energy);
        end

        function plot_metrics(obj)
            % plot_metrics() — plot spread, connectivity, and energy over time
            t            = obj.time_log;
            spread       = [obj.metric_log.spread];
            connectivity = [obj.metric_log.connectivity];
            energy       = [obj.metric_log.energy];

            figure('Name', 'Swarm Metrics', 'NumberTitle', 'off');

            subplot(3,1,1);
            plot(t, spread, 'b-', 'LineWidth', 1.5);
            ylabel('Spread (m)'); grid on;
            title('Swarm Performance Metrics');

            subplot(3,1,2);
            plot(t, connectivity, 'r-', 'LineWidth', 1.5);
            ylabel('Fiedler Value'); grid on;

            subplot(3,1,3);
            plot(t, energy, 'g-', 'LineWidth', 1.5);
            ylabel('Kinetic Energy'); xlabel('Time (s)'); grid on;
        end

        function plot_analysis(obj, name)
            % plot_analysis(name) — comprehensive, clean analysis dashboard.
            %   Plots every important swarm metric over time in a labelled
            %   2×3 grid, ready for thesis/analysis use:
            %     1. Spread               — how dispersed the swarm is
            %     2. Algebraic connectivity (Fiedler λ₂) — graph cohesion
            %     3. Kinetic energy       — control effort / activity
            %     4. Group speed          — centroid translation speed
            %     5. Polarization φ       — velocity alignment order [0,1]
            %     6. Closest approach     — minimum inter-agent distance (safety)
            if nargin < 2, name = 'Swarm Analysis'; end

            t      = obj.time_log;
            spread = [obj.metric_log.spread];
            conn   = [obj.metric_log.connectivity];
            energy = [obj.metric_log.energy];
            [pol, dmin, gspeed] = obj.derived_metrics();

            figure('Name', name, 'NumberTitle', 'off', 'Color', 'w');
            % Titles/labels use Interpreter 'none' so arbitrary names render
            % literally regardless of the session's default interpreter; the
            % two math labels (λ₂, φ) opt into LaTeX explicitly.

            subplot(2,3,1);
            plot(t, spread); grid on;
            title('Swarm Spread', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Mean distance to centroid (m)', 'Interpreter', 'none');

            subplot(2,3,2);
            plot(t, conn); grid on; hold on;
            yline(0, ':', 'Color', [0.5 0.5 0.5]);
            title('Algebraic Connectivity', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Fiedler value $\lambda_2$', 'Interpreter', 'latex');

            subplot(2,3,3);
            plot(t, energy); grid on;
            title('Kinetic Energy', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Total energy (J)', 'Interpreter', 'none');

            subplot(2,3,4);
            plot(t, gspeed); grid on;
            title('Group Speed', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Centroid speed (m/s)', 'Interpreter', 'none');

            subplot(2,3,5);
            plot(t, pol); grid on; ylim([0 1]);
            title('Polarization', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Alignment order $\phi$', 'Interpreter', 'latex');

            subplot(2,3,6);
            plot(t, dmin); grid on;
            title('Closest Approach', 'Interpreter', 'none');
            xlabel('Time (s)', 'Interpreter', 'none');
            ylabel('Min inter-agent distance (m)', 'Interpreter', 'none');

            sgtitle(name, 'Interpreter', 'none');
        end

        function export_csv(obj, filename)
            % export_csv(filename) — write all time-series metrics to a CSV file
            spread       = [obj.metric_log.spread]';
            connectivity = [obj.metric_log.connectivity]';
            energy       = [obj.metric_log.energy]';
            centroid     = [obj.metric_log.centroid]';          % T×2
            [pol, dmin, gspeed] = obj.derived_metrics();

            T = table(obj.time_log', spread, connectivity, energy, ...
                      centroid(:,1), centroid(:,2), pol', dmin', gspeed', ...
                      'VariableNames', {'time','spread','connectivity','energy', ...
                                        'centroid_x','centroid_y','polarization', ...
                                        'min_distance','group_speed'});
            writetable(T, filename);
            fprintf('Logged %d steps to %s\n', obj.log_count, filename);
        end
    end

    methods (Access = private)
        function [pol, dmin, gspeed] = derived_metrics(obj)
            % Compute derived analysis metrics from the recorded state history.
            %   pol    — Vicsek polarization (velocity alignment order) in [0,1]
            %   dmin   — minimum pairwise inter-agent distance (metres)
            %   gspeed — centroid translation speed (m/s)
            T      = obj.log_count;
            pol    = zeros(1, T);
            dmin   = nan(1, T);
            gspeed = zeros(1, T);

            for k = 1:T
                S = obj.state_log{k};
                P = S(1:2, :);
                n = size(P, 2);

                % Polarization: ||mean of unit velocity vectors||
                if size(S, 1) >= 4
                    V  = S(3:4, :);
                    sp = vecnorm(V);
                    moving = sp > 1e-9;
                    if any(moving)
                        pol(k) = norm(mean(V(:, moving) ./ sp(moving), 2));
                    end
                end

                % Minimum pairwise distance (collision-safety metric)
                if n >= 2
                    DX = P(1,:).' - P(1,:);
                    DY = P(2,:).' - P(2,:);
                    D  = sqrt(DX.^2 + DY.^2);
                    D(1:n+1:end) = inf;
                    dmin(k) = min(D(:));
                end
            end

            % Group speed: numerical derivative of the centroid path
            C = [obj.metric_log.centroid];   % 2×T
            if T >= 2
                tt = obj.time_log;
                dC = [zeros(2,1), diff(C, 1, 2)];
                dt = [1, diff(tt)];
                dt(dt == 0) = eps;
                gspeed = vecnorm(dC) ./ dt;
                gspeed(1) = gspeed(min(2, T));   % avoid start-of-run artifact
            end
        end
    end
end
