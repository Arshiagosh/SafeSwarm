% e4a_mpc_vs_cbf — distributed NMPC against the CBF-QP filters
%
% Swap, crossing and corridor, single and double integrator, 5 seeds each.
% Every controller runs at the same rate (control_period 0.2 s, the MPC step),
% with the dynamics integrated at 0.05 s:
%   cbf_central        go-to-goal + centralised CBF-QP filter
%   cbf_decentral_rhb  go-to-goal + decentralised filter with right-hand bias
%   mpc                distributed NMPC with plan sharing (horizon 10 x 0.2 s)
%   mpc_cbf            the same NMPC with the decentralised filter on top
% Writes results.json (key "e4_mpc") and figures/e4_mpc.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

scenes = {'swap', 'crossing', 'corridor'};
kinds  = {'si', 'di'};
ctrls  = {'cbf_central', 'cbf_decentral_rhb', 'mpc', 'mpc_cbf'};
seeds  = 1:5;

rows = {};
for a = 1:numel(scenes)
    for b = 1:numel(kinds)
        for c = 1:numel(ctrls)
            runs = MonteCarloRunner.run(@(seed) run_case(make_scene(scenes{a}, kinds{b}, seed), ...
                                          ctrls{c}, 40, 0.05, 0.2), seeds);
            row = summarise_runs(runs, struct('scene', scenes{a}, 'dynamics', kinds{b}, ...
                                              'controller', ctrls{c}));
            t = [runs.time_all_reached];
            row.time_all_reached_mean = mean(t(~isnan(t)));
            row.mean_solve_ms = mean([runs.mean_solve_ms]);
            row.mpc_soft_mean = mean([runs.mpc_soft]);
            rows{end+1} = row; %#ok<SAGROW>
            print_row(row);
        end
    end
end
rows = [rows{:}];
save_results('e4_mpc', rows);

% One panel per dynamics: closest approach (median, min-max bar) per scene and controller
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1150 420]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
for b = 1:numel(kinds)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    Y = nan(numel(scenes), numel(ctrls)); L = Y; U = Y;
    for a = 1:numel(scenes)
        for c = 1:numel(ctrls)
            sel = strcmp({rows.scene}, scenes{a}) & strcmp({rows.dynamics}, kinds{b}) & ...
                  strcmp({rows.controller}, ctrls{c});
            Y(a, c) = rows(sel).min_distance_median;
            L(a, c) = rows(sel).min_distance_min;
            U(a, c) = rows(sel).min_distance_max;
        end
    end
    hb = bar(ax, categorical(scenes, scenes), Y);
    for c = 1:numel(ctrls)
        x = hb(c).XEndPoints;
        errorbar(ax, x, Y(:, c), Y(:, c) - L(:, c), U(:, c) - Y(:, c), 'k.', 'LineWidth', 1);
    end
    yline(ax, 0.6, 'r--', 'contact'); yline(ax, 0.65, 'k:', 'D');
    ylim(ax, [0 1.1]); ylabel(ax, 'Closest approach (m): median, min to max over 5 seeds');
    legend(ax, strrep(ctrls, '_', ' '), 'Location', 'southoutside', 'Orientation', 'horizontal');
    title(ax, sprintf('%s, all controllers at 5 Hz', upper(kinds{b})));
end
print(f, fullfile(root, 'figures', 'e4_mpc.png'), '-dpng', '-r150');
close(f);
