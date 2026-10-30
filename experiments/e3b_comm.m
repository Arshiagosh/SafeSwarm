% e3b_comm — packet loss and delay under the decentralised filter (Monte Carlo)
%
% Swap scene, single and double integrator, decentralised filter with the
% right-hand bias. Each robot builds its constraints from the last state it
% received from each neighbour; messages are dropped with probability p and
% delayed by 0 or 5 control steps (0.25 s). 10 seeds per setting, 45 s per run.
% The centralised filter assumes perfect information, so it is not in this sweep.
% Writes results.json (key "e3_comm") and figures/e3_comm.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

losses = [0 0.25 0.5 0.75 0.9];
delays = [0 5];
kinds  = {'si', 'di'};
seeds  = 1:10;

rows = {};
for b = 1:numel(kinds)
    for dl = delays
        for p = losses
            opts = struct('loss', p, 'delay', dl);
            runs = MonteCarloRunner.run(@(seed) run_case(make_scene('swap', kinds{b}, seed), ...
                                          'cbf_decentral_rhb', 45, 0.05, [], [], opts), seeds);
            rows{end+1} = summarise_runs(runs, struct('dynamics', kinds{b}, ...
                          'controller', sprintf('delay %d steps', dl), 'loss', p, 'delay', dl)); %#ok<SAGROW>
            print_row(rows{end});
        end
    end
end
rows = [rows{:}];
save_results('e3_comm', rows);

ctrls = {'delay 0 steps', 'delay 5 steps'};
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 400]);
tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
plot_sweep(nexttile, rows, 'loss', 'collision_rate', kinds, ctrls);
xlabel('Packet loss probability'); ylabel('Runs with contact'); ylim([0 1]);
title('Contact rate (10 seeds, 95% Wilson interval)');
plot_sweep(nexttile, rows, 'loss', 'min_distance_median', kinds, ctrls);
yline(0.6, 'r--', 'contact'); yline(0.65, 'k:', 'D');
xlabel('Packet loss probability'); ylabel('Closest approach (m), median over seeds');
title('Decentralised CBF-QP with lossy, delayed messages');
print(f, fullfile(root, 'figures', 'e3_comm.png'), '-dpng', '-r150');
close(f);
