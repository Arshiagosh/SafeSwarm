% e3a_noise — how sensor noise degrades the safety filters (Monte Carlo)
%
% Swap scene, single and double integrator, centralised filter and the
% decentralised filter with the right-hand bias. Each robot's controller sees
% positions (and velocities) with zero-mean Gaussian noise of std sigma; the
% filter margin is not changed. 10 seeds per setting (start offsets and noise),
% 45 s per run.
% Writes results.json (key "e3_noise") and figures/e3_noise.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

sigmas = [0 0.02 0.05 0.1 0.2];
kinds  = {'si', 'di'};
ctrls  = {'cbf_central', 'cbf_decentral_rhb'};
seeds  = 1:10;

rows = {};
for b = 1:numel(kinds)
    for c = 1:numel(ctrls)
        for s = sigmas
            opts = struct('noise_p', s, 'noise_v', s);
            runs = MonteCarloRunner.run(@(seed) run_case(make_scene('swap', kinds{b}, seed), ...
                                          ctrls{c}, 45, 0.05, [], [], opts), seeds);
            rows{end+1} = summarise_runs(runs, struct('dynamics', kinds{b}, ...
                                         'controller', ctrls{c}, 'sigma', s)); %#ok<SAGROW>
            print_row(rows{end});
        end
    end
end
rows = [rows{:}];
save_results('e3_noise', rows);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 400]);
tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
plot_sweep(nexttile, rows, 'sigma', 'collision_rate', kinds, ctrls);
xlabel('Noise std \sigma (m, m/s)'); ylabel('Runs with contact'); ylim([0 1]);
title('Contact rate (10 seeds, 95% Wilson interval)');
plot_sweep(nexttile, rows, 'sigma', 'min_distance_median', kinds, ctrls);
yline(0.6, 'r--', 'contact'); yline(0.65, 'k:', 'D');
xlabel('Noise std \sigma (m, m/s)'); ylabel('Closest approach (m), median over seeds');
title('Closest approach');
print(f, fullfile(root, 'figures', 'e3_noise.png'), '-dpng', '-r150');
close(f);
