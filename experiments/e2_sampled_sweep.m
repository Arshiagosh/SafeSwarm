% e2_sampled_sweep — what happens to a continuous-time CBF filter under a zero-order hold
%
% The CBF condition hdot >= -alpha h is designed for a command that changes
% continuously. A digital controller computes the command every T seconds
% and holds it in between. This sweep runs the centralised filter on the
% swap and crossing scenes, single and double integrator, for a range of
% hold times T, with the dynamics integrated at 2 ms, and records the
% closest approach. Nothing in the filter accounts for T.
% Writes results.json (key "e2") and figures/e2_sampled_sweep.png.
%
% Run from anywhere: run('experiments/e2_sampled_sweep.m').

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);
fig_dir = fullfile(root, 'figures');

holds  = [0.002 0.05 0.1 0.2 0.3 0.4 0.5 0.75 1.0];
scenes = {'swap', 'crossing'};
kinds  = {'si', 'di'};
dt     = 0.002;
t_max  = 25;

rows = {};
for a = 1:numel(scenes)
    for b = 1:numel(kinds)
        for T = holds
            sc = make_scene(scenes{a}, kinds{b}, 1);
            r  = run_case(sc, 'cbf_central', t_max, dt, T);
            rows{end+1} = r; %#ok<SAGROW>
            fprintf('%-9s %-3s T = %5.3f s  dmin %.3f  (D = %.2f)  coll %d  infeas %d  reached %d/%d\n', ...
                    r.scene, r.dynamics, T, r.min_distance, r.safe_distance, ...
                    r.agent_collision_steps, r.qp_infeasible, r.reached, r.n_agents);
        end
    end
end
rows = [rows{:}];
save_results('e2', rows);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 760 420]);
hold on; grid on; box on;
styles = {'-o', '--s', '-^', '--d'};
k = 0; names = {};
for a = 1:numel(scenes)
    for b = 1:numel(kinds)
        k = k + 1;
        sel = strcmp({rows.scene}, scenes{a}) & strcmp({rows.dynamics}, kinds{b});
        plot([rows(sel).control_period], [rows(sel).min_distance], styles{k}, 'LineWidth', 1.4);
        names{end+1} = sprintf('%s, %s', scenes{a}, upper(kinds{b})); %#ok<SAGROW>
    end
end
yline(rows(1).safe_distance, 'k:', 'filter distance D');
yline(0.6, 'r--', 'contact (2 x radius)');
xlabel('Hold time T (s)'); ylabel('Closest approach (m)');
title('Centralised CBF-QP under a zero-order hold');
legend([names, {'', ''}], 'Location', 'southwest');
print(f, fullfile(fig_dir, 'e2_sampled_sweep.png'), '-dpng', '-r150');
close(f);
