classdef PublicationPlot < handle
    %% PublicationPlot — publication-quality figure generation from a DataLogger
    %
    % All methods are static. Generates 16×14 cm (trajectory) and 16×12 cm
    % (metrics) figures at 300 DPI. Exports to both PDF (vector) and PNG.
    %
    % Example:
    %   PublicationPlot.trajectory_plot(logger, env, 'Flocking experiment');
    %   PublicationPlot.metrics_plot(logger, 'Flocking metrics');
    %   PublicationPlot.save_fig('figures/flocking');

    methods (Static)
        function trajectory_plot(logger, env, title_str)
            % trajectory_plot(logger, env, title_str) — agent paths with start/end markers
            %   title_str : figure title string
            figure('Units','centimeters','Position',[2 2 16 14]);
            ax = axes; hold on; axis equal; grid on;

            PublicationPlot.draw_env(env);

            N      = size(logger.state_log{1}, 2);
            colors = lines(N);

            for i = 1:N
                traj = cell2mat(cellfun(@(s) s(1:2,i), ...
                       logger.state_log, 'UniformOutput', false));
                plot(traj(1,:), traj(2,:), '-',  'Color', colors(i,:), 'LineWidth', 1.2);
                plot(traj(1,1), traj(2,1), 'o',  'Color', colors(i,:), ...
                     'MarkerFaceColor', colors(i,:), 'MarkerSize', 5);
                plot(traj(1,end), traj(2,end), 's', 'Color', colors(i,:), ...
                     'MarkerFaceColor', colors(i,:), 'MarkerSize', 5);
            end

            xlabel('x (m)'); ylabel('y (m)');
            title(title_str, 'FontSize', 11);
            set(ax, 'FontSize', 10, 'LineWidth', 1);
            box on;
        end

        function metrics_plot(logger, title_str)
            % metrics_plot(logger, title_str) — 3-panel spread / Fiedler / energy plot
            t      = logger.time_log;
            spread = [logger.metric_log.spread];
            conn   = [logger.metric_log.connectivity];
            energy = [logger.metric_log.energy];

            figure('Units','centimeters','Position',[2 2 16 12]);

            subplot(3,1,1);
            plot(t, spread, 'b-', 'LineWidth', 1.5); grid on;
            ylabel('Spread (m)'); title(title_str, 'FontSize', 11);
            set(gca, 'FontSize', 10);

            subplot(3,1,2);
            plot(t, conn, 'r-', 'LineWidth', 1.5); grid on;
            ylabel('\lambda_2 (Fiedler)');
            set(gca, 'FontSize', 10);

            subplot(3,1,3);
            plot(t, energy, 'g-', 'LineWidth', 1.5); grid on;
            ylabel('Energy (J)'); xlabel('Time (s)');
            set(gca, 'FontSize', 10);
        end

        function draw_env(env)
            % draw_env(env) — render all obstacles onto the current axes
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    th = linspace(0, 2*pi, 60);
                    fill(obs.center(1) + obs.radius*cos(th), ...
                         obs.center(2) + obs.radius*sin(th), ...
                         [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1);
                elseif strcmp(obs.type, 'rect')
                    xr = obs.x_range; yr = obs.y_range;
                    fill([xr(1) xr(2) xr(2) xr(1)], [yr(1) yr(1) yr(2) yr(2)], ...
                         [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1);
                end
            end
        end

        function save_fig(filename)
            % save_fig(filename) — export current figure as PDF (vector) and PNG (300 DPI)
            %   filename : path without extension, e.g. 'figures/flocking'
            exportgraphics(gcf, [filename '.pdf'], 'ContentType', 'vector');
            exportgraphics(gcf, [filename '.png'], 'Resolution', 300);
            fprintf('Saved: %s.pdf / .png\n', filename);
        end
    end
end
