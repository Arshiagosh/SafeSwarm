function row = summarise_runs(runs, labels)
    % summarise_runs(runs, labels) — one summary row for a batch of seeded runs
    %   runs   : struct array from MonteCarloRunner.run over run_case
    %   labels : struct of fields copied into the row (setting being swept)
    row = labels;
    row.n_runs = numel(runs);
    [p, lo, hi] = MonteCarloRunner.rate(runs, @(r) r.agent_collision_steps > 0);
    row.collision_rate    = p;
    row.collision_rate_lo = lo;
    row.collision_rate_hi = hi;
    d = MonteCarloRunner.summarise(runs, 'min_distance');
    row.min_distance_median = d.median;
    row.min_distance_min    = d.min;
    row.min_distance_max    = d.max;
    row.reached_all_rate = mean([runs.reached] == [runs.n_agents]);
    row.reached_mean     = mean([runs.reached]);   % robots at their goal, mean over seeds
    row.infeasible_steps_mean = mean([runs.qp_infeasible]);
end
