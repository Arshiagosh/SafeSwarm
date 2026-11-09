% e4c_unicycle_mpc — the MPC planner and the filters on unicycle robots
%
% Swap and crossing with unicycles (1 m/s, 2 rad/s, starting faced at their
% goals), 5 seeds, every controller at 5 Hz (control_period 0.2 s):
%   cbf_central  go-to-goal + centralised CBF-QP filter (look-ahead point 0.2 m)
%   mpc          distributed NMPC, exact arc rollout, finite-difference gradients
%   mpc_cbf      the same NMPC with the decentralised filter on top
% Writes results.json (key "e4_unicycle").

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));

scenes = {'swap', 'crossing'};
ctrls  = {'cbf_central', 'mpc', 'mpc_cbf'};
seeds  = 1:5;

rows = {};
for a = 1:numel(scenes)
    for c = 1:numel(ctrls)
        runs = MonteCarloRunner.run(@(seed) run_case(make_scene(scenes{a}, 'uni', seed), ...
                                      ctrls{c}, 40, 0.05, 0.2), seeds);
        row = summarise_runs(runs, struct('scene', scenes{a}, 'dynamics', 'uni', 'controller', ctrls{c}));
        t = [runs.time_all_reached];
        row.time_all_reached_mean = mean(t(~isnan(t)));
        row.mean_solve_ms = mean([runs.mean_solve_ms]);
        rows{end+1} = row; %#ok<SAGROW>
        print_row(row);
    end
end
save_results('e4_unicycle', [rows{:}]);
