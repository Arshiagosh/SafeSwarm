function run_all_scenarios()
    % run_all_scenarios — smoke test: runs every scenario and reports PASS/FAIL.
    %
    % Resolves all paths relative to this file, so it works no matter what the
    % current folder is. Run it with:  run('tests/run_all_scenarios.m')

    here     = fileparts(mfilename('fullpath'));
    root     = fileparts(here);
    scen_dir = fullfile(root, 'scenarios');
    addpath(genpath(root));
    if ~exist(fullfile(root, 'out'), 'dir'); mkdir(fullfile(root, 'out')); end

    scs = { ...
        'scenario_aggregation', ...
        'scenario_dispersion', ...
        'scenario_flocking', ...
        'scenario_potential_field', ...
        'scenario_rrt', ...
        'scenario_astar', ...
        'scenario_leader_follower', ...
        'scenario_virtual_structure', ...
        'scenario_compare', ...
        'scenario_param_sweep', ...
        'scenario_full_experiment' ...
    };

    n_pass = 0; n_fail = 0;
    for k = 1:numel(scs)
        fprintf('\n========== RUN [%d/%d] %s ==========\n', k, numel(scs), scs{k});
        try
            close all force;
            t0 = tic;
            run_one(fullfile(scen_dir, [scs{k} '.m']));
            fprintf('---------- PASS %s (%.1f s) ----------\n', scs{k}, toc(t0));
            n_pass = n_pass + 1;
        catch e
            fprintf('---------- FAIL %s : %s ----------\n', scs{k}, e.message);
            for s = 1:min(4, numel(e.stack))
                fprintf('     at %s (line %d)\n', e.stack(s).name, e.stack(s).line);
            end
            n_fail = n_fail + 1;
        end
    end
    close all force;
    fprintf('\n========== SUMMARY: %d passed, %d failed ==========\n', n_pass, n_fail);
end

function run_one(scenario_path)
    % Separate scope so a scenario's own `clear` only wipes this workspace,
    % not the driver loop variables.
    run(scenario_path);
end
