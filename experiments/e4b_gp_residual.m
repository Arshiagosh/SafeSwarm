% e4b_gp_residual — an unknown wind field, and a GP that learns it for the filter
%
% Every robot is pushed by the same position-dependent disturbance d(p)
% (a swirl plus a steady push in x, at most about 0.5 in input units),
% which enters like the input. The centralised CBF-QP filter is run on the
% swap scene, single and double integrator, with four disturbance models:
%   none     the filter ignores d (nominal model only)
%   gp_mean  GP posterior mean added to the constraints (kappa = 0)
%   gp_2sd   GP mean plus a margin of 2 posterior standard deviations
%   oracle   the true d (reference)
% For each seed the GP is trained on 20 s of the same robots wandering to
% random waypoints in the field (400 samples per run, residual targets with
% added noise of std 0.05). 10 seeds per setting.
% Writes results.json (key "e4_gp") and figures/e4_gp.png.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

wind   = @(p) 0.25 * [-p(2); p(1)] / (1 + 0.08 * (p' * p)) + [0.1; 0];
kinds  = {'si', 'di'};
models = {'none', 'gp_mean', 'gp_2sd', 'oracle'};
seeds  = 1:10;

rows = {};
gp_err = {};
for b = 1:numel(kinds)
    for m = 1:numel(models)
        runs = MonteCarloRunner.run(@(seed) one_run(kinds{b}, models{m}, seed, wind), seeds);
        rows{end+1} = summarise_runs(runs, struct('dynamics', kinds{b}, ...
                                     'controller', models{m})); %#ok<SAGROW>
        if isfield(runs, 'gp_rmse')
            rows{end}.gp_rmse_mean = mean([runs.gp_rmse]);
        else
            rows{end}.gp_rmse_mean = NaN;
        end
        print_row(rows{end});
    end
end
rows = [rows{:}];
save_results('e4_gp', rows);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 400]);
tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
for k = 1:2
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    if k == 1, fld = 'collision_rate'; else, fld = 'min_distance_median'; end
    Y = zeros(numel(models), numel(kinds));
    for b = 1:numel(kinds)
        for m = 1:numel(models)
            sel = strcmp({rows.dynamics}, kinds{b}) & strcmp({rows.controller}, models{m});
            Y(m, b) = rows(sel).(fld);
        end
    end
    bar(ax, categorical(strrep(models, '_', ' '), strrep(models, '_', ' ')), Y);
    legend(ax, upper(kinds), 'Location', 'best', 'AutoUpdate', 'off');
    if k == 1
        ylim(ax, [0 1]); ylabel(ax, 'Runs with contact (10 seeds)');
        title(ax, 'Contact rate under an unknown wind field');
    else
        yline(ax, 0.6, 'r--', 'contact'); yline(ax, 0.65, 'k:', 'D');
        ylabel(ax, 'Closest approach (m), median over seeds');
        title(ax, 'Closest approach');
    end
end
print(f, fullfile(root, 'figures', 'e4_gp.png'), '-dpng', '-r150');
close(f);

function r = one_run(kind, model, seed, wind)
    % Train the GP (if needed) on exploration data, then run the filtered swap.
    opts = struct();
    switch model
        case 'oracle'
            opts.residual = FunctionResidual(wind); opts.kappa = 0;
        case {'gp_mean', 'gp_2sd'}
            [gp, rmse] = train_gp(kind, seed, wind);
            opts.residual = gp;
            opts.kappa = 2 * strcmp(model, 'gp_2sd');
    end
    sc = make_scene('swap', kind, seed);
    for i = 1:sc.swarm.N
        sc.swarm.agents{i}.dynamics.disturbance = wind;
    end
    r = run_case(sc, 'cbf_central', 40, 0.05, [], [], opts);
    if exist('rmse', 'var'), r.gp_rmse = rmse; else, r.gp_rmse = NaN; end
end

function [gp, rmse] = train_gp(kind, seed, wind)
    % 20 s of the eight robots driving to random waypoints, then fit the GP.
    sc = make_scene('swap', kind, 100 + seed);
    for i = 1:sc.swarm.N
        sc.swarm.agents{i}.dynamics.disturbance = wind;
        if strcmp(kind, 'di'), sc.swarm.agents{i}.dynamics.max_speed = Inf; end
    end
    targets = 12 * rand(2, sc.swarm.N) - 6;
    explore = FunctionBehaviour(@(sw, ~) wander(sw));
    sim = SimEngine(sc.swarm, sc.env, explore, 0.05, 20);
    sim.contact_mode = 'none';
    sim.logger = DataLogger();
    sim.run();
    [X, Y] = GPResidual.residual_samples(sim.logger, 0.05);
    pick = randperm(size(X, 2), 400);
    X = X(:, pick);
    Y = Y(:, pick) + 0.05 * randn(2, 400);
    gp = GPResidual(); gp.fit(X, Y);
    % Accuracy on a grid over the swap area
    [gx, gy] = meshgrid(-6:0.5:6);
    Pg = [gx(:)'; gy(:)'];
    mu = gp.predict(Pg);
    tru = cell2mat(arrayfun(@(k) wind(Pg(:, k)), 1:size(Pg, 2), 'UniformOutput', false));
    rmse = sqrt(mean(sum((mu - tru).^2, 1)));

    function u = wander(sw)
        % Go-to-waypoint commands, saturated here so the logged command is the applied one.
        S = sw.get_states();
        near = vecnorm(S(1:2, :) - targets) < 0.5;
        targets(:, near) = 12 * rand(2, nnz(near)) - 6;
        if size(S, 1) == 4
            u = 1.0 * (targets - S(1:2, :)) - 1.5 * S(3:4, :);
        else
            u = 0.5 * (targets - S(1:2, :));
        end
        for i = 1:sw.N
            u(:, i) = sw.agents{i}.dynamics.saturate(u(:, i));
        end
    end
end
