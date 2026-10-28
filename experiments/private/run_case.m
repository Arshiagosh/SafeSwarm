function [r, sim] = run_case(sc, controller, t_max, dt, control_period, viz)
    % run_case(sc, controller, t_max, dt, control_period, viz) — run one benchmark case
    %   sc         : scene from make_scene
    %   controller : 'nominal' | 'potential' | 'cbf_central' | 'cbf_decentral'
    %                | 'cbf_decentral_rhb' (decentralised + right-hand bias)
    %   control_period : zero-order-hold period (empty = every step)
    %   viz        : optional SwarmVisualizer
    % Returns a struct of scalar metrics and the finished SimEngine.
    if nargin < 5, control_period = []; end
    if nargin < 6, viz = []; end

    nominal = GoToGoal(sc.goals, 1.0);
    nominal.via = sc.via;
    nominal.via_tol = 0.8;
    filt    = [];
    switch controller
        case 'nominal'
            behav = nominal;
        case 'potential'
            % SwarmSim's pairwise repulsion plus the same repulsion from obstacles
            behav = CollisionAvoidance(nominal, 1.2, 1.0);
            behav.max_u = Inf;
            behav = FormationWithObstacles(behav, 1.0, 1.0);
            behav.max_accel = Inf;
        case 'cbf_central'
            filt = SafetyFilter(nominal, 'Mode', 'centralised');
        case 'cbf_decentral'
            filt = SafetyFilter(nominal, 'Mode', 'decentralised');
        case 'cbf_decentral_rhb'
            filt = SafetyFilter(nominal, 'Mode', 'decentralised', 'RightHandBias', 1.0);
        otherwise
            error('run_case:Controller', 'Unknown controller %s', controller);
    end
    if ~isempty(filt), behav = filt; end

    sim = SimEngine(sc.swarm, sc.env, behav, dt, t_max);
    sim.contact_mode   = 'none';
    sim.control_period = control_period;
    sim.logger         = DataLogger();
    sim.visualizer     = viz;
    sim.run();

    s   = sim.logger.summary();
    P   = sim.logger.positions();                         % 2×N×T
    err = squeeze(vecnorm(P - sc.goals, 2, 1));           % N×T
    all_in = all(err <= 0.2, 1);
    t   = sim.logger.time_log;
    k   = find(~all_in, 1, 'last');                       % last step someone was out
    if isempty(k), t_reach = t(1);
    elseif k == numel(t), t_reach = NaN;
    else, t_reach = t(k + 1);
    end

    r = struct();
    r.scene      = sc.name;
    r.dynamics   = sc.kind;
    r.controller = controller;
    r.control_period = control_period;
    r.min_distance        = s.min_distance;
    r.min_clearance       = s.min_clearance;
    r.agent_collision_steps    = s.agent_collisions;
    r.obstacle_collision_steps = s.obstacle_collisions;
    r.reached             = sum(err(:, end) <= 0.2);
    r.n_agents            = sc.swarm.N;
    r.time_all_reached    = t_reach;
    r.control_effort      = s.control_effort;
    if isempty(filt)
        r.qp_calls = 0; r.qp_infeasible = 0; r.max_slack = 0; r.mean_solve_ms = NaN;
        r.safe_distance = NaN;
    else
        r.qp_calls      = filt.n_calls;
        r.qp_infeasible = filt.n_infeasible;
        r.max_slack     = max([filt.log_slack 0]);
        r.mean_solve_ms = 1000 * mean(filt.log_solve);
        r.safe_distance = filt.safe_distance(sc.swarm);
    end
end
