% e5_rl_shield — a learned policy with and without a CBF safety filter
%
% Each of the eight crossing robots (single integrator) learns its own
% tabular Q-learning policy alone (learning/QLearningPolicy.m, 1 m grid, 5000 episodes),
% so the policies know nothing about each other. The learned policies are then
% run together: on their own, and with the centralised or decentralised
% CBF-QP filter on top. 10 seeds (training and start offsets).
% Writes results.json (key "e5_rl"), figures/e5_rl_training.png and
% figures/e5_rl.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

ctrls = {'rl', 'rl_cbf_central', 'rl_cbf_decentral'};
seeds = 1:10;
episodes = 5000;

runs = cell(numel(ctrls), numel(seeds));
returns = zeros(numel(seeds), episodes);
for s = seeds
    rng(s);
    sc  = make_scene('crossing', 'si', s);
    pol = QLearningPolicy(sc.goals, sc.env);
    pol.train(sc.swarm, episodes);
    returns(s, :) = mean(pol.returns, 1);
    for c = 1:numel(ctrls)
        sc = make_scene('crossing', 'si', s);       % fresh robots, same starts
        r  = run_case(sc, ctrls{c}, 40, 0.05, [], [], struct('policy', pol));
        r.seed = s;
        runs{c, s} = r;
    end
    fprintf('seed %d trained and run\n', s);
end

rows = {};
for c = 1:numel(ctrls)
    rr = [runs{c, :}];
    row = summarise_runs(rr, struct('dynamics', 'si', 'controller', ctrls{c}));
    t = [rr.time_all_reached];
    row.time_all_reached_mean = mean(t(~isnan(t)));
    rows{end+1} = row; %#ok<SAGROW>
    print_row(row);
end
rows = [rows{:}];
save_results('e5_rl', rows);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 640 360]);
k = 50;
m = movmean(mean(returns, 1), k);
plot(1:episodes, m, 'LineWidth', 1.5); grid on;
xlabel('Episode'); ylabel(sprintf('Return (mean over robots and seeds, %d-episode window)', k));
title('Independent Q-learning, crossing goals');
print(f, fullfile(root, 'figures', 'e5_rl_training.png'), '-dpng', '-r150');
close(f);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 380]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl);
bar(ax, categorical(strrep(ctrls, '_', ' '), strrep(ctrls, '_', ' ')), [rows.collision_rate]);
ylim(ax, [0 1]); grid(ax, 'on'); ylabel(ax, 'Runs with contact (10 seeds)');
title(ax, 'Learned policies, crossing, SI');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
cats = categorical(strrep(ctrls, '_', ' '), strrep(ctrls, '_', ' '));
bar(ax, cats, [rows.min_distance_median]);
errorbar(ax, cats, [rows.min_distance_median], [rows.min_distance_median] - [rows.min_distance_min], ...
         [rows.min_distance_max] - [rows.min_distance_median], 'k.', 'LineWidth', 1);
yline(ax, 0.6, 'r--', 'contact'); yline(ax, 0.65, 'k:', 'D');
ylabel(ax, 'Closest approach (m): median, min to max');
title(ax, 'Closest approach');
print(f, fullfile(root, 'figures', 'e5_rl.png'), '-dpng', '-r150');
close(f);
