classdef DataLogger < handle
    %% DataLogger — records per-step swarm states, controls and metrics
    %
    % Attach to SimEngine via sim.logger = DataLogger() before calling run().
    % After the simulation, call plot_analysis(), summary() or export_csv().
    % MetricsAnalyzer provides higher-level derived statistics.
    %
    % Metrics logged each step:
    %   centroid      — swarm centre of mass [x; y]
    %   spread        — mean distance of agents from centroid (metres)
    %   connectivity  — Fiedler value of graph Laplacian (> 0 means connected)
    %   energy        — total kinetic energy (0.5 * sum(||v||^2))
    %   min_distance  — smallest centre-to-centre distance between two agents
    %   min_clearance — smallest distance from an agent's surface to an obstacle
    %   effort        — sum over agents of ||u_i||^2 for the applied command
    %
    % A collision is counted when two agents are closer than 2*agent_radius,
    % or an agent is closer than agent_radius to an obstacle surface.
    %
    % Example:
    %   logger     = DataLogger();
    %   sim.logger = logger;
    %   sim.run();
    %   logger.plot_analysis();
    %   s = logger.summary();

    properties
        time_log    = [];   % 1×T vector of timestamps
        state_log   = {};   % {t} → [state_dim × N] matrix
        control_log = {};   % {t} → [control_dim × N] command that produced state_log{t}
        metric_log  = struct('centroid', {}, 'spread', {}, 'connectivity', {}, ...
                             'energy', {}, 'min_distance', {}, 'min_clearance', {}, ...
                             'effort', {});
        log_count   = 0;
        agent_radius = 0;   % copied from the swarm at the first log() call
    end

    methods
        function log(obj, t, swarm, env, u)
            % log(t, swarm, env, u) — record state and metrics at time t
            %   env and u are optional; without env no clearance is logged,
            %   without u no effort is logged.
            if nargin < 4, env = []; end
            if nargin < 5, u   = []; end
            obj.log_count       = obj.log_count + 1;
            obj.time_log(end+1) = t;
            obj.agent_radius    = swarm.agent_radius;

            states = swarm.get_states();
            obj.state_log{end+1}   = states;
            obj.control_log{end+1} = u;

            P        = states(1:2, :);
            n        = size(P, 2);
            centroid = mean(P, 2);
            spread   = mean(vecnorm(P - centroid));

            % Connectivity: Fiedler eigenvalue of the graph Laplacian
            A   = swarm.get_adjacency();
            L   = diag(sum(A, 2)) - A;
            evs = sort(real(eig(L)));
            connectivity = evs(min(2, length(evs)));

            % Kinetic energy: only meaningful for states with velocity (dim >= 4)
            if size(states, 1) >= 4
                energy = 0.5 * sum(vecnorm(states(3:4, :)).^2);
            else
                energy = 0;
            end

            % Closest pair of agents
            dmin = NaN;
            if n >= 2
                D = sqrt((P(1,:).' - P(1,:)).^2 + (P(2,:).' - P(2,:)).^2);
                D(1:n+1:end) = inf;
                dmin = min(D(:));
            end

            % Closest agent surface to an obstacle surface
            clr = NaN;
            if ~isempty(env) && ~isempty(env.obstacles)
                clr = min(env.obstacle_distances(P), [], 'all') - swarm.agent_radius;
            end

            effort = NaN;
            if ~isempty(u), effort = sum(u(:).^2); end

            obj.metric_log(end+1) = struct( ...
                'centroid',      centroid, ...
                'spread',        spread, ...
                'connectivity',  connectivity, ...
                'energy',        energy, ...
                'min_distance',  dmin, ...
                'min_clearance', clr, ...
                'effort',        effort);
        end

        function s = summary(obj)
            % summary() — scalar safety and performance numbers for the whole run
            %   min_distance       : smallest agent-agent distance (centre to centre)
            %   min_clearance      : smallest agent-obstacle clearance (surface to surface)
            %   agent_collisions   : logged steps with at least one agent-agent collision
            %   obstacle_collisions: logged steps with at least one agent-obstacle collision
            %   control_effort     : time integral of sum ||u_i||^2
            dmin = [obj.metric_log.min_distance];
            clr  = [obj.metric_log.min_clearance];
            eff  = [obj.metric_log.effort];
            t    = obj.time_log;
            ok   = ~isnan(eff);

            s.duration            = t(end) - t(1);
            s.min_distance        = min(dmin);
            s.min_clearance       = min(clr);
            s.agent_collisions    = sum(dmin < 2 * obj.agent_radius);
            s.obstacle_collisions = sum(clr < 0);
            if sum(ok) >= 2
                s.control_effort = trapz(t(ok), eff(ok));
            else
                s.control_effort = NaN;
            end
        end

        function plot_metrics(obj)
            % plot_metrics() — plot spread, connectivity, and energy over time
            t = obj.time_log;
            figure('Name', 'Swarm Metrics', 'NumberTitle', 'off');
            subplot(3,1,1);
            plot(t, [obj.metric_log.spread], 'b-', 'LineWidth', 1.5);
            ylabel('Spread (m)'); grid on;
            title('Swarm Performance Metrics');
            subplot(3,1,2);
            plot(t, [obj.metric_log.connectivity], 'r-', 'LineWidth', 1.5);
            ylabel('Fiedler Value'); grid on;
            subplot(3,1,3);
            plot(t, [obj.metric_log.energy], 'g-', 'LineWidth', 1.5);
            ylabel('Kinetic Energy'); xlabel('Time (s)'); grid on;
        end

        function fig = plot_analysis(obj, name, visible)
            % plot_analysis(name, visible) — 2×3 analysis dashboard
            %     1. Spread               — how dispersed the swarm is
            %     2. Algebraic connectivity (Fiedler λ₂) — graph cohesion
            %     3. Kinetic energy       — control effort / activity
            %     4. Group speed          — centroid translation speed
            %     5. Polarization φ       — velocity alignment order [0,1]
            %     6. Closest approach     — minimum inter-agent distance (safety)
            %   visible : 'on' (default) or 'off' for batch figure export
            if nargin < 2, name = 'Swarm Analysis'; end
            if nargin < 3, visible = 'on'; end

            t      = obj.time_log;
            [pol, gspeed] = obj.derived_metrics();
            dmin   = [obj.metric_log.min_distance];

            fig = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', ...
                         'Visible', visible, 'Position', [100 100 1100 620]);
            panels = {
                [obj.metric_log.spread],       'Swarm Spread',           'Mean distance to centroid (m)'
                [obj.metric_log.connectivity], 'Algebraic Connectivity', 'Fiedler value'
                [obj.metric_log.energy],       'Kinetic Energy',         'Total energy (J)'
                gspeed,                        'Group Speed',            'Centroid speed (m/s)'
                pol,                           'Polarization',           'Alignment order'
                dmin,                          'Closest Approach',       'Min inter-agent distance (m)'};
            for k = 1:6
                subplot(2, 3, k);
                plot(t, panels{k, 1}); grid on;
                title(panels{k, 2}, 'Interpreter', 'none');
                xlabel('Time (s)', 'Interpreter', 'none');
                ylabel(panels{k, 3}, 'Interpreter', 'none');
            end
            subplot(2, 3, 5); ylim([0 1]);
            if obj.agent_radius > 0
                subplot(2, 3, 6); hold on;
                yline(2 * obj.agent_radius, 'r--', 'collision');
            end
            sgtitle(name, 'Interpreter', 'none');
        end

        function export_csv(obj, filename)
            % export_csv(filename) — write all time-series metrics to a CSV file
            centroid = [obj.metric_log.centroid]';          % T×2
            [pol, gspeed] = obj.derived_metrics();

            T = table(obj.time_log', [obj.metric_log.spread]', ...
                      [obj.metric_log.connectivity]', [obj.metric_log.energy]', ...
                      centroid(:,1), centroid(:,2), pol', ...
                      [obj.metric_log.min_distance]', [obj.metric_log.min_clearance]', ...
                      [obj.metric_log.effort]', gspeed', ...
                      'VariableNames', {'time','spread','connectivity','energy', ...
                                        'centroid_x','centroid_y','polarization', ...
                                        'min_distance','min_clearance','effort','group_speed'});
            writetable(T, filename);
            fprintf('Logged %d steps to %s\n', obj.log_count, filename);
        end

        function P = positions(obj)
            % positions() — 2×N×T array of logged positions
            T = obj.log_count;
            n = size(obj.state_log{1}, 2);
            P = zeros(2, n, T);
            for k = 1:T
                P(:, :, k) = obj.state_log{k}(1:2, :);
            end
        end
    end

    methods (Access = private)
        function [pol, gspeed] = derived_metrics(obj)
            % pol    — Vicsek polarization (velocity alignment order) in [0,1]
            % gspeed — centroid translation speed (m/s)
            T   = obj.log_count;
            pol = zeros(1, T);
            for k = 1:T
                S = obj.state_log{k};
                if size(S, 1) >= 4
                    V  = S(3:4, :);
                    sp = vecnorm(V);
                    moving = sp > 1e-9;
                    if any(moving)
                        pol(k) = norm(mean(V(:, moving) ./ sp(moving), 2));
                    end
                end
            end

            gspeed = zeros(1, T);
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
