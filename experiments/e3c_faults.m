% e3c_faults — one faulty robot among cooperative ones (Monte Carlo)
%
% Crossing scene, single and double integrator. From t = 3 s robot 1 either
% freezes, wanders at full speed in a random direction, or goes silent (keeps
% driving but sends no messages). A frozen or wandering robot ignores its
% safety filter. Centralised filter vs decentralised filter with the
% right-hand bias. 10 seeds per setting.
% Writes results.json (key "e3_faults") and figures/e3_faults.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

modes = {'none', 'freeze', 'random_walk', 'silent'};
kinds = {'si', 'di'};
ctrls = {'cbf_central', 'cbf_decentral_rhb'};
seeds = 1:10;

rows = {};
for b = 1:numel(kinds)
    for c = 1:numel(ctrls)
        for m = 1:numel(modes)
            if strcmp(modes{m}, 'silent') && strcmp(ctrls{c}, 'cbf_central')
                continue;   % no messages in the centralised filter
            end
            opts = struct();
            if ~strcmp(modes{m}, 'none')
                opts.faults = struct('agent', 1, 'mode', modes{m}, 't_start', 3);
            end
            runs = MonteCarloRunner.run(@(seed) run_case(make_scene('crossing', kinds{b}, seed), ...
                                          ctrls{c}, 30, 0.05, [], [], opts), seeds);
            rows{end+1} = summarise_runs(runs, struct('dynamics', kinds{b}, ...
                          'controller', ctrls{c}, 'fault', modes{m})); %#ok<SAGROW>
            print_row(rows{end});
        end
    end
end
rows = [rows{:}];
save_results('e3_faults', rows);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 400]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
for b = 1:numel(kinds)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    Y = nan(numel(modes), numel(ctrls));
    for m = 1:numel(modes)
        for c = 1:numel(ctrls)
            sel = strcmp({rows.dynamics}, kinds{b}) & strcmp({rows.controller}, ctrls{c}) & ...
                  strcmp({rows.fault}, modes{m});
            if any(sel), Y(m, c) = rows(sel).collision_rate; end
        end
    end
    bar(ax, categorical(strrep(modes, '_', ' '), strrep(modes, '_', ' ')), Y);
    ylim(ax, [0 1]); ylabel(ax, 'Runs with contact');
    legend(ax, strrep(ctrls, '_', ' '), 'Location', 'northwest');
    title(ax, sprintf('Crossing, %s, fault on robot 1 at t = 3 s', upper(kinds{b})));
end
print(f, fullfile(root, 'figures', 'e3_faults.png'), '-dpng', '-r150');
close(f);
