classdef NMPCPlanner < handle
    %% NMPCPlanner — distributed nonlinear MPC with plan sharing
    %
    % Every control call, each robot solves its own finite-horizon problem
    % with fmincon (SQP):
    %
    %   min_U  sum_k  q ||p_k - g||^2 + r ||u_k||^2   (+ q_v ||v_N||^2 for DI)
    %   s.t.   exact discrete dynamics with step tau (horizon H steps)
    %          ||u_k|| <= u_max            (inner 16-sided polygon, linear)
    %          ||v_k|| <= v_max            (DI, inner polygon, linear)
    %          ||p_k - p^j_k||^2 >= D^2    for neighbours j within sensing_radius
    %          clearance to every obstacle >= D/2
    %
    % where p^j_k is robot j's plan from the previous call, shifted by one
    % step (robots exchange plans, in the spirit of distributed MPC for
    % multi-robot motion planning, e.g. Luis, Vukosavljev and Schoellig,
    % IEEE RA-L 5(2), 2020). Before the first plans exist, neighbours are
    % predicted with constant velocity. Constraints hold at the H sample
    % points only, and the plans of neighbours are one step old, so this is
    % not a safety guarantee; the experiments measure how close it gets.
    % The collision rows are softened by one slack variable with a large
    % penalty, so a robot that is already too close still gets a useful plan
    % (counted in n_soft). If fmincon fails anyway, the robot keeps its
    % previous plan shifted by one step (counted in n_failed).
    %
    % Use with SimEngine.control_period = tau so the first input of each
    % plan is held for exactly one MPC step.
    % Works with: SingleIntegrator, DoubleIntegrator, Unicycle. For the
    % unicycle the plan is an exact arc rollout of (v, omega) per step with box
    % bounds, and fmincon uses finite-difference gradients (slower).
    % Requires the Optimization Toolbox (fmincon).
    %
    % Example:
    %   mpc = NMPCPlanner(goals, 'Horizon', 10, 'Tau', 0.2);
    %   sim = SimEngine(swarm, env, mpc, 0.05, 30);
    %   sim.control_period = 0.2;

    properties
        goals                   % 2×N goal positions
        horizon = 10;           % H, number of steps
        tau     = 0.2;          % MPC step (s)
        q       = 1.0;          % position weight
        r       = 0.1;          % input weight
        q_v     = 1.0;          % terminal velocity weight (DI)
        d_safe  = [];           % centre distance; empty = 2*agent_radius + margin
        margin  = 0.05;
        sensing_radius = 4.0;   % neighbours farther than this are ignored (m)
        polygon_sides  = 16;

        via     = [];           % optional 2×N via points (as in GoToGoal); empty = none
        via_tol = 0.8;          % distance at which a via point counts as passed (m)
        passed  = [];           % 1×N logical
        plans   = [];           % 2×(H+1)×N planned positions from the last call
        U_prev  = {};           % warm starts
        n_calls = 0;
        n_failed = 0;           % solver failures (previous plan reused)
        n_soft   = 0;           % solves that needed the slack (plan closer than D)
        slack_weight = 1e3;     % weight of the collision slack (linear and quadratic)
        log_solve = [];         % wall-clock time per call, all robots (s)
    end

    properties (Access = private)
        opts
        opts_fd     % unicycle: finite-difference gradients
    end

    methods
        function obj = NMPCPlanner(goals, varargin)
            % NMPCPlanner(goals, Name, Value, ...)
            %   Names: 'Horizon', 'Tau', 'Q', 'R', 'QV', 'DSafe', 'Margin', 'SensingRadius'
            if ~(exist('fmincon', 'file') == 2)
                error('NMPCPlanner:NoToolbox', 'NMPCPlanner needs fmincon from the Optimization Toolbox.');
            end
            p = inputParser;
            p.addParameter('Horizon', obj.horizon);
            p.addParameter('Tau', obj.tau);
            p.addParameter('Q', obj.q);
            p.addParameter('R', obj.r);
            p.addParameter('QV', obj.q_v);
            p.addParameter('DSafe', obj.d_safe);
            p.addParameter('Margin', obj.margin);
            p.addParameter('SensingRadius', obj.sensing_radius);
            p.parse(varargin{:});
            o = p.Results;
            obj.goals = goals;
            obj.horizon = o.Horizon; obj.tau = o.Tau; obj.q = o.Q; obj.r = o.R; obj.q_v = o.QV;
            obj.d_safe = o.DSafe; obj.margin = o.Margin; obj.sensing_radius = o.SensingRadius;
            obj.opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
                'SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient', true, ...
                'MaxIterations', 100, 'OptimalityTolerance', 1e-4, 'ConstraintTolerance', 1e-4);
            obj.opts_fd = optimoptions(obj.opts, 'SpecifyObjectiveGradient', false, ...
                'SpecifyConstraintGradient', false);
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — first input of each robot's plan
            t0 = tic;
            S  = swarm.get_states();
            n  = swarm.N;
            H  = obj.horizon;
            di = size(S, 1) == 4;
            if isempty(obj.d_safe), D = 2 * swarm.agent_radius + obj.margin; else, D = obj.d_safe; end

            % Neighbour predictions: last plans shifted by one step, else constant velocity
            pred = zeros(2, H, n);
            for j = 1:n
                if ~isempty(obj.plans)
                    pj = obj.plans(:, 3:end, j);
                    pred(:, :, j) = [pj, repmat(pj(:, end), 1, H - size(pj, 2))];
                else
                    vj = [0; 0];                % unicycle and SI: assumed at rest
                    if di, vj = S(3:4, j); end
                    pred(:, :, j) = S(1:2, j) + vj * obj.tau * (1:H);
                end
            end

            goals = obj.goals;
            if ~isempty(obj.via)
                if isempty(obj.passed), obj.passed = isnan(obj.via(1, :)); end
                obj.passed = obj.passed | vecnorm(S(1:2, :) - obj.via) <= obj.via_tol;
                goals(:, ~obj.passed) = obj.via(:, ~obj.passed);
            end

            u = zeros(2, n);
            new_plans = zeros(2, H + 1, n);
            if numel(obj.U_prev) ~= n, obj.U_prev = repmat({zeros(2*H, 1)}, 1, n); end
            uni = size(S, 1) == 3;
            for i = 1:n
                nb = [];
                for j = [1:i-1, i+1:n]
                    if norm(S(1:2, j) - S(1:2, i)) <= obj.sensing_radius, nb(end+1) = j; end %#ok<AGROW>
                end
                dyn = swarm.agents{i}.dynamics;
                U0  = [obj.U_prev{i}(3:end); obj.U_prev{i}(end-1:end)];
                if uni
                    [U, Pi, flag, sl] = obj.solve_unicycle(S(:, i), dyn, goals(:, i), ...
                                                           pred(:, :, nb), D, env, U0);
                else
                    [G, p0, Gv, v0] = obj.prediction(S(:, i), di);
                    [A, b] = obj.linear_bounds(dyn, di, Gv, v0);
                    % Decision vector z = [U; s]; s >= 0 softens the collision rows.
                    fun = @(z) obj.cost(z, G, p0, Gv, v0, goals(:, i), di);
                    con = @(z) obj.collision(z, G, p0, pred(:, :, nb), D, env);
                    lb  = [-inf(2*H, 1); 0];
                    [z, ~, flag] = fmincon(fun, [U0; 0], [A, zeros(size(A, 1), 1)], b, ...
                                           [], [], lb, [], con, obj.opts);
                    U  = z(1:end-1);
                    sl = z(end);
                end
                if flag <= 0
                    % Solver failure: keep the previous plan, shifted, and count it.
                    obj.n_failed = obj.n_failed + 1;
                    U = U0;
                elseif sl > 1e-3
                    obj.n_soft = obj.n_soft + 1;
                end
                obj.U_prev{i} = U;
                u(:, i) = U(1:2);
                if uni
                    Pi = obj.rollout_unicycle(S(:, i), U);
                    new_plans(:, :, i) = [S(1:2, i), Pi];
                else
                    new_plans(:, :, i) = [S(1:2, i), reshape(p0 + G * U, 2, H)];
                end
            end
            obj.plans = new_plans;
            obj.n_calls = obj.n_calls + 1;
            obj.log_solve(end+1) = toc(t0);
        end
    end

    methods (Access = private)
        function [U, P, flag, sl] = solve_unicycle(obj, x, dyn, g, pred, D, env, U0)
            % Unicycle: U = [v_1; w_1; ...; v_H; w_H] with box bounds, exact arc
            % rollout, gradients by finite differences (no analytic Jacobian).
            H  = obj.horizon;
            lb = [repmat([-dyn.max_speed; -dyn.max_omega], H, 1); 0];
            ub = [repmat([ dyn.max_speed;  dyn.max_omega], H, 1); Inf];
            fun = @(z) obj.cost_unicycle(z, x, g);
            con = @(z) obj.collision_unicycle(z, x, pred, D, env);
            [z, ~, flag] = fmincon(fun, [min(max(U0, lb(1:end-1)), ub(1:end-1)); 0], ...
                                   [], [], [], [], lb, ub, con, obj.opts_fd);
            U  = z(1:end-1);
            sl = z(end);
            P  = obj.rollout_unicycle(x, U);
        end

        function P = rollout_unicycle(obj, x, U)
            % Positions after each of the H steps under constant (v, w) per step.
            H = obj.horizon; t = obj.tau;
            P = zeros(2, H);
            for k = 1:H
                v = U(2*k-1); w = U(2*k); th = x(3);
                if abs(w) < 1e-9
                    x(1:2) = x(1:2) + v * t * [cos(th); sin(th)];
                else
                    x(1:2) = x(1:2) + (v / w) * [sin(th + w*t) - sin(th); cos(th) - cos(th + w*t)];
                end
                x(3) = th + w * t;
                P(:, k) = x(1:2);
            end
        end

        function J = cost_unicycle(obj, z, x, g)
            U = z(1:end-1); sl = z(end);
            P = obj.rollout_unicycle(x, U);
            E = P - g;
            J = obj.q * sum(E(:).^2) + obj.r * (U' * U) + obj.slack_weight * (sl + sl^2);
        end

        function [c, ceq] = collision_unicycle(obj, z, x, pred, D, env)
            P = obj.rollout_unicycle(x, z(1:end-1));
            [c, ceq] = obj.position_constraints(P, pred, D, env);
            c = c - z(end);
        end

        function [c, ceq] = position_constraints(obj, P, pred, D, env)
            % Same rows as collision(), from planned positions directly.
            H = obj.horizon;
            c = zeros(0, 1);
            for jj = 1:size(pred, 3)
                for k = 1:H
                    d = P(:, k) - pred(:, k, jj);
                    c(end+1, 1) = D^2 - d' * d; %#ok<AGROW>
                end
            end
            if ~isempty(env)
                for o = 1:numel(env.obstacles)
                    obs = env.obstacles{o};
                    for k = 1:H
                        p = P(:, k);
                        if strcmp(obs.type, 'circle')
                            qc = obs.center(:); R = obs.radius + D/2;
                        else
                            qc = [min(max(p(1), obs.x_range(1)), obs.x_range(2));
                                  min(max(p(2), obs.y_range(1)), obs.y_range(2))];
                            R = D/2;
                        end
                        d = p - qc;
                        c(end+1, 1) = R^2 - d' * d; %#ok<AGROW>
                    end
                end
            end
            ceq = [];
        end

        function [G, p0, Gv, v0] = prediction(obj, x, di)
            % Stacked positions P = p0 + G U (and velocities V = v0 + Gv U) over k = 1..H.
            H = obj.horizon; t = obj.tau;
            G = zeros(2*H); Gv = zeros(2*H);
            p0 = zeros(2*H, 1); v0 = zeros(2*H, 1);
            for k = 1:H
                rk = 2*k-1:2*k;
                for m = 1:k
                    cm = 2*m-1:2*m;
                    if di
                        G(rk, cm)  = ((k - m) * t^2 + t^2 / 2) * eye(2);
                        Gv(rk, cm) = t * eye(2);
                    else
                        G(rk, cm)  = t * eye(2);
                    end
                end
                if di
                    p0(rk) = x(1:2) + k * t * x(3:4);
                    v0(rk) = x(3:4);
                else
                    p0(rk) = x(1:2);
                end
            end
        end

        function [A, b] = linear_bounds(obj, dyn, di, Gv, v0)
            % Input polygon at every step, and the speed polygon for DI.
            H   = obj.horizon;
            m   = obj.polygon_sides;
            phi = 2*pi*(0:m-1)'/m;
            E   = [cos(phi) sin(phi)];
            if di, umax = dyn.max_accel; else, umax = dyn.max_speed; end
            A = kron(eye(H), E);
            b = umax * cos(pi/m) * ones(m*H, 1);
            if di && isfinite(dyn.max_speed)
                Av = kron(eye(H), E);
                A  = [A; Av * Gv];
                b  = [b; dyn.max_speed * cos(pi/m) * ones(m*H, 1) - Av * v0];
            end
        end

        function [J, dJ] = cost(obj, z, G, p0, Gv, v0, g, di)
            U  = z(1:end-1);
            sl = z(end);
            H  = obj.horizon;
            e  = p0 + G * U - repmat(g, H, 1);
            J  = obj.q * (e' * e) + obj.r * (U' * U);
            dJ = 2 * obj.q * G' * e + 2 * obj.r * U;
            if di
                vN = v0(end-1:end) + Gv(end-1:end, :) * U;
                J  = J + obj.q_v * (vN' * vN);
                dJ = dJ + 2 * obj.q_v * Gv(end-1:end, :)' * vN;
            end
            J  = J + obj.slack_weight * (sl + sl^2);
            dJ = [dJ; obj.slack_weight * (1 + 2*sl)];
        end

        function [c, ceq, dc, dceq] = collision(obj, z, G, p0, pred, D, env)
            % c <= 0 form; one row per neighbour per step, and per obstacle per step,
            % each relaxed by the slack s = z(end).
            U  = z(1:end-1);
            H  = obj.horizon;
            P  = reshape(p0 + G * U, 2, H);
            c  = zeros(0, 1); dc = zeros(numel(U), 0);
            for jj = 1:size(pred, 3)
                for k = 1:H
                    d = P(:, k) - pred(:, k, jj);
                    c(end+1, 1) = D^2 - d' * d;                          %#ok<AGROW>
                    dc(:, end+1) = -2 * G(2*k-1:2*k, :)' * d;            %#ok<AGROW>
                end
            end
            if ~isempty(env)
                for o = 1:numel(env.obstacles)
                    obs = env.obstacles{o};
                    for k = 1:H
                        p = P(:, k);
                        if strcmp(obs.type, 'circle')
                            qc = obs.center(:); R = obs.radius + D/2;
                        else
                            qc = [min(max(p(1), obs.x_range(1)), obs.x_range(2));
                                  min(max(p(2), obs.y_range(1)), obs.y_range(2))];
                            R = D/2;
                        end
                        d = p - qc;   % every obstacle row is kept: fmincon needs a fixed count
                        c(end+1, 1) = R^2 - d' * d;                      %#ok<AGROW>
                        dc(:, end+1) = -2 * G(2*k-1:2*k, :)' * d;        %#ok<AGROW>
                    end
                end
            end
            dc  = [dc; -ones(1, numel(c))];
            c   = c - z(end);
            ceq = []; dceq = [];
        end
    end
end
