% e1_benchmarks — safety filters against the nominal and potential-field controllers
%
% Three scenes (swap, crossing, corridor) x two dynamics (single and double
% integrator) x five controllers:
%   nominal            go-to-goal, no collision avoidance
%   potential          SwarmSim's repulsive potential field (agents and obstacles)
%   cbf_central        centralised CBF-QP filter
%   cbf_decentral      decentralised CBF-QP filter (reciprocal half split)
%   cbf_decentral_rhb  the same with the right-hand deadlock heuristic
% Writes results.json (key "e1"), figures/e1_*.png and figures/swap_cbf.gif.
%
% Run from anywhere: run('experiments/e1_benchmarks.m'). Takes a few minutes.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);
fig_dir = fullfile(root, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

scenes      = {'swap', 'crossing', 'corridor'};
kinds       = {'si', 'di'};
controllers = {'nominal', 'potential', 'cbf_central', 'cbf_decentral', 'cbf_decentral_rhb'};
t_max = 40;
dt    = 0.05;

results = {};
for a = 1:numel(scenes)
    for b = 1:numel(kinds)
        f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 330]);
        tl = tiledlayout(f, 1, numel(controllers), 'TileSpacing', 'compact');
        for c = 1:numel(controllers)
            sc = make_scene(scenes{a}, kinds{b}, 1);
            [r, sim] = run_case(sc, controllers{c}, t_max, dt);
            results{end+1} = r; %#ok<SAGROW>
            fprintf('%-9s %-3s %-18s dmin %.3f  clr %6.3f  coll %4d/%4d  reached %d/%d  t %5.1f  infeas %4d  solve %.1f ms\n', ...
                    r.scene, r.dynamics, r.controller, r.min_distance, r.min_clearance, ...
                    r.agent_collision_steps, r.obstacle_collision_steps, r.reached, r.n_agents, ...
                    r.time_all_reached, r.qp_infeasible, r.mean_solve_ms);
            plot_paths(nexttile(tl), sc, sim.logger, r);
        end
        title(tl, sprintf('%s, %s', scenes{a}, upper(kinds{b})), 'Interpreter', 'none');
        print(f, fullfile(fig_dir, sprintf('e1_%s_%s.png', scenes{a}, kinds{b})), '-dpng', '-r130');
        close(f);
    end
end
save_results('e1', [results{:}]);

% Hero GIF: the swap with the centralised filter
sc  = make_scene('swap', 'si', 1);
viz = SwarmVisualizer(sc.swarm, sc.env, 'Theme', 'light', 'Visible', 'off', ...
                      'GifFile', fullfile(fig_dir, 'swap_cbf.gif'), 'GifEvery', 3, ...
                      'Title', 'Antipodal swap, centralised CBF-QP');
run_case(sc, 'cbf_central', 15, dt, [], viz);
viz.capture_frame(true);
viz.close();

function plot_paths(ax, sc, logger, r)
    % Trajectories of one run, with obstacles, start and goal markers.
    P = logger.positions();
    hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
    for k = 1:numel(sc.env.obstacles)
        o = sc.env.obstacles{k};
        if strcmp(o.type, 'rect')
            fill(ax, o.x_range([1 2 2 1]), o.y_range([1 1 2 2]), [0.75 0.75 0.75], 'EdgeColor', 'none');
        end
    end
    for i = 1:size(P, 2)
        c = sc.swarm.agents{i}.color;
        plot(ax, squeeze(P(1, i, :)), squeeze(P(2, i, :)), '-', 'Color', c, 'LineWidth', 1.2);
        plot(ax, P(1, i, 1), P(2, i, 1), 'o', 'Color', c, 'MarkerSize', 4);
        plot(ax, sc.goals(1, i), sc.goals(2, i), 'x', 'Color', c, 'MarkerSize', 6);
    end
    axis(ax, [sc.env.x_lim sc.env.y_lim]);
    title(ax, {strrep(r.controller, '_', ' '), ...
               sprintf('d_{min} %.2f m, reached %d/%d', r.min_distance, r.reached, r.n_agents)}, ...
          'FontWeight', 'normal', 'FontSize', 10);
end
