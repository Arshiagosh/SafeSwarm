classdef SafetyFilter < handle
    %% SafetyFilter — CBF-QP safety filter that wraps any behaviour
    %
    % Each step the nominal behaviour proposes u_hat; the filter returns the
    % closest command (least squares) that satisfies control barrier function
    % (CBF) constraints for every close pair of agents and every obstacle,
    % together with the input bounds:
    %
    %   min_u  sum_i ||u_i - u_hat_i||^2
    %   s.t.   CBF rows    A u <= b
    %          input rows  ||u_i|| <= u_max  (inner polygon), or a box for Unicycle
    %
    % Barriers (D = d_safe, the required centre-to-centre distance):
    %   SingleIntegrator  h_ij = ||p_i - p_j||^2 - D^2, relative degree 1,
    %                     hdot >= -alpha h.
    %   DoubleIntegrator  same h, relative degree 2, handled as a high-order
    %                     CBF: psi = hdot + alpha h, psidot >= -alpha2 psi
    %                     (Xiao and Belta, CDC 2019). A speed barrier
    %                     vmax^2 - ||v||^2 >= 0 is added when max_speed is finite.
    %   Unicycle          the CBF of the single integrator is applied to a
    %                     look-ahead point at distance 'lookahead' in front of
    %                     each robot, with D enlarged by 2*lookahead.
    % Obstacles get the same barriers with the obstacle surface as the other
    % "agent" (moving circles use their velocity).
    %
    % Modes:
    %   'centralised'   one QP over all agents' inputs.
    %   'decentralised' one small QP per agent. Each agent takes half of
    %                   every pairwise constraint (the reciprocal split of
    %                   Wang, Ames and Egerstedt, IEEE T-RO 33(3), 2017), so
    %                   the pair constraint holds if both halves hold, and
    %                   needs only neighbours' positions and velocities.
    %
    % Deadlock: in symmetric scenes (the antipodal swap) the filtered robots
    % can stop short of their goals with every constraint satisfied. This is
    % a known property of CBF-QP filters, not a bug. 'RightHandBias' > 0
    % turns on a simple heuristic: when another agent is within 2*d_safe, the
    % nominal command of a point-mass agent gets an extra component of
    % RightHandBias*||u_hat_i|| to its right. It changes the nominal command
    % only, so the safety constraints are untouched. Off by default.
    %
    % Input bounds can make the CBF constraints infeasible. Then the QP is
    % solved again with a slack on the CBF rows (input bounds stay hard), the
    % step is counted in n_infeasible and the largest slack is logged. The
    % filter never falls back to the nominal command silently.
    %
    % Requires the Optimization Toolbox (quadprog).
    %
    % Example:
    %   nominal = GoToGoal(goals);
    %   safe    = SafetyFilter(nominal, 'Mode', 'decentralised', 'Margin', 0.05);
    %   sim     = SimEngine(swarm, env, safe, 0.02, 20);

    properties
        nominal                 % wrapped behaviour
        mode      = 'decentralised';  % 'centralised' | 'decentralised'
        d_safe    = [];         % centre distance to keep; empty = 2*agent_radius + margin
        margin    = 0.05;       % added to 2*agent_radius when d_safe is empty (m)
        alpha     = 2.0;        % class-K gain (linear)
        alpha2    = 2.0;        % second gain for relative-degree-2 barriers
        alpha_v   = 2.0;        % gain of the speed barrier
        sensing_radius = Inf;   % pairs farther apart than this are ignored
        lookahead = 0.2;        % look-ahead distance for Unicycle agents (m)
        polygon_sides = 16;     % sides of the polygon inside the input disc
        avoid_obstacles = true;
        slack_weight = 1e4;     % weight of the slack when the QP is infeasible
        right_hand_bias = 0;    % deadlock heuristic gain (0 = off), point-mass agents only

        % Statistics, appended once per call
        n_calls      = 0;
        n_infeasible = 0;       % calls in which some QP needed slack
        log_time     = [];      % swarm time of each call
        log_slack    = [];      % largest slack in that call (0 = feasible)
        log_solve    = [];      % wall-clock solve time of that call (s)
        log_change   = [];      % ||u - u_hat||_F in that call
    end

    properties (Access = private)
        qp_opts
    end

    methods
        function obj = SafetyFilter(nominal, varargin)
            % SafetyFilter(nominal, Name, Value, ...)
            %   Names: 'Mode', 'DSafe', 'Margin', 'Alpha', 'Alpha2', 'AlphaV',
            %          'SensingRadius', 'Lookahead', 'AvoidObstacles'
            if ~(exist('quadprog', 'file') == 2)
                error('SafetyFilter:NoToolbox', ...
                      'SafetyFilter needs quadprog from the Optimization Toolbox.');
            end
            p = inputParser;
            p.addParameter('Mode',   obj.mode);
            p.addParameter('DSafe',  obj.d_safe);
            p.addParameter('Margin', obj.margin);
            p.addParameter('Alpha',  obj.alpha);
            p.addParameter('Alpha2', obj.alpha2);
            p.addParameter('AlphaV', obj.alpha_v);
            p.addParameter('SensingRadius', obj.sensing_radius);
            p.addParameter('Lookahead', obj.lookahead);
            p.addParameter('AvoidObstacles', obj.avoid_obstacles);
            p.addParameter('RightHandBias', obj.right_hand_bias);
            p.parse(varargin{:});
            o = p.Results;
            obj.nominal   = nominal;
            obj.mode      = validatestring(o.Mode, {'centralised', 'decentralised'});
            obj.d_safe    = o.DSafe;
            obj.margin    = o.Margin;
            obj.alpha     = o.Alpha;
            obj.alpha2    = o.Alpha2;
            obj.alpha_v   = o.AlphaV;
            obj.sensing_radius  = o.SensingRadius;
            obj.lookahead       = o.Lookahead;
            obj.avoid_obstacles = o.AvoidObstacles;
            obj.right_hand_bias = o.RightHandBias;
            obj.qp_opts = optimoptions('quadprog', 'Display', 'off');
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — filtered control_dim×N command
            u_hat = obj.nominal.compute_control(swarm, env);
            if obj.right_hand_bias > 0
                u_hat = obj.add_right_hand_bias(u_hat, swarm);
            end
            t0    = tic;
            [rows, kind] = obj.constraints(swarm, env);
            n  = swarm.N;
            m  = size(u_hat, 1);
            [Ab, bb] = obj.input_bounds(swarm, kind);

            if strcmp(obj.mode, 'centralised')
                A = zeros(numel(rows.b), m*n);
                for r = 1:numel(rows.b)
                    A(r, m*(rows.i(r)-1) + (1:m)) = rows.ai(r, :);
                    if rows.j(r) > 0
                        A(r, m*(rows.j(r)-1) + (1:m)) = rows.aj(r, :);
                    end
                end
                [x, slack] = obj.solve(u_hat(:), A, rows.b, ...
                                       blkdiag_rep(Ab, n), repmat(bb, n, 1));
                u = reshape(x, m, n);
            else
                u = zeros(m, n);
                slack = 0;
                for i = 1:n
                    % Rows that involve agent i: its own share of each pair
                    % constraint (half of b) and the full obstacle rows.
                    own = rows.i == i;
                    oth = rows.j == i;
                    Ai  = [rows.ai(own, :); rows.aj(oth, :)];
                    bi  = [rows.b(own); rows.b(oth)];
                    pair = [rows.j(own) > 0; true(nnz(oth), 1)];
                    bi(pair) = bi(pair) / 2;
                    % The pair row a_i u_i + a_j u_j <= b is split into
                    % a_i u_i <= b/2 and a_j u_j <= b/2.
                    [ui, si] = obj.solve(u_hat(:, i), Ai, bi, Ab, bb);
                    u(:, i)  = ui;
                    slack    = max(slack, si);
                end
            end

            obj.n_calls = obj.n_calls + 1;
            obj.n_infeasible = obj.n_infeasible + (slack > 0);
            obj.log_time(end+1)   = swarm.time;
            obj.log_slack(end+1)  = slack;
            obj.log_solve(end+1)  = toc(t0);
            obj.log_change(end+1) = norm(u - u_hat, 'fro');
        end

        function D = safe_distance(obj, swarm)
            % safe_distance(swarm) — centre-to-centre distance the filter keeps
            if isempty(obj.d_safe)
                D = 2 * swarm.agent_radius + obj.margin;
            else
                D = obj.d_safe;
            end
        end

        function reset_stats(obj)
            % reset_stats() — clear the per-call statistics
            obj.n_calls = 0; obj.n_infeasible = 0;
            obj.log_time = []; obj.log_slack = []; obj.log_solve = []; obj.log_change = [];
        end
    end

    methods (Access = private)
        function [rows, kind] = constraints(obj, swarm, env)
            % Build all CBF rows. Row r reads ai*u_i + aj*u_j <= b, with j = 0
            % for rows that involve only agent i (obstacles, speed).
            S    = swarm.get_states();
            n    = swarm.N;
            kind = size(S, 1);              % 2 = SI, 3 = unicycle, 4 = DI
            D    = obj.safe_distance(swarm);
            R    = D / 2;                   % radius used against obstacles

            % Point that the barrier acts on, its velocity, and the map from
            % the input to its derivative (SI/unicycle) or second derivative (DI).
            P = S(1:2, :);
            V = zeros(2, n);
            M = repmat(eye(2), 1, 1, n);
            if kind == 4
                V = S(3:4, :);
            elseif kind == 3
                l  = obj.lookahead;
                th = S(3, :);
                P  = P + l * [cos(th); sin(th)];
                for i = 1:n
                    M(:, :, i) = [cos(th(i)) -l*sin(th(i)); sin(th(i)) l*cos(th(i))];
                end
                D = D + 2 * l;
                R = R + l;
            end

            ai = zeros(0, 2); aj = zeros(0, 2); b = zeros(0, 1);
            I  = zeros(0, 1); J  = zeros(0, 1);

            % Agent-agent rows
            for i = 1:n-1
                for j = i+1:n
                    dp = P(:, i) - P(:, j);
                    if norm(dp) > obj.sensing_radius, continue; end
                    dv = V(:, i) - V(:, j);
                    [g, bij] = obj.barrier_row(dp, dv, D, kind);
                    % g' (input_i - input_j) <= bij   (g = -2 dp)
                    ai(end+1, :) = g' * M(:, :, i);      %#ok<AGROW>
                    aj(end+1, :) = -g' * M(:, :, j);     %#ok<AGROW>
                    b(end+1, 1)  = bij;                  %#ok<AGROW>
                    I(end+1, 1)  = i;                    %#ok<AGROW>
                    J(end+1, 1)  = j;                    %#ok<AGROW>
                end
            end

            % Obstacle rows (the agent takes the whole constraint)
            if obj.avoid_obstacles && ~isempty(env) && ~isempty(env.obstacles)
                for k = 1:numel(env.obstacles)
                    obs = env.obstacles{k};
                    for i = 1:n
                        if strcmp(obs.type, 'circle')
                            q  = obs.center(:);
                            Dk = obs.radius + R;
                            vq = [0; 0];
                            if isfield(obs, 'velocity'), vq = obs.velocity(:); end
                        else
                            q  = [min(max(P(1,i), obs.x_range(1)), obs.x_range(2));
                                  min(max(P(2,i), obs.y_range(1)), obs.y_range(2))];
                            Dk = R;
                            vq = [0; 0];
                        end
                        dp = P(:, i) - q;
                        if norm(dp) - Dk > obj.sensing_radius, continue; end
                        [g, bik] = obj.barrier_row(dp, V(:, i) - vq, Dk, kind);
                        ai(end+1, :) = g' * M(:, :, i);  %#ok<AGROW>
                        aj(end+1, :) = [0 0];            %#ok<AGROW>
                        b(end+1, 1)  = bik;              %#ok<AGROW>
                        I(end+1, 1)  = i;                %#ok<AGROW>
                        J(end+1, 1)  = 0;                %#ok<AGROW>
                    end
                end
            end

            % Speed barrier for double integrators: 2 v' a <= alpha_v (vmax^2 - ||v||^2)
            if kind == 4
                for i = 1:n
                    vmax = swarm.agents{i}.dynamics.max_speed;
                    if isfinite(vmax)
                        v = V(:, i);
                        ai(end+1, :) = 2 * v';                          %#ok<AGROW>
                        aj(end+1, :) = [0 0];                           %#ok<AGROW>
                        b(end+1, 1)  = obj.alpha_v * (vmax^2 - v'*v);   %#ok<AGROW>
                        I(end+1, 1)  = i;                               %#ok<AGROW>
                        J(end+1, 1)  = 0;                               %#ok<AGROW>
                    end
                end
            end

            rows = struct('ai', ai, 'aj', aj, 'b', b, 'i', I, 'j', J);
        end

        function [g, b] = barrier_row(obj, dp, dv, D, kind)
            % One CBF row g' (w_i - w_j) <= b for h = ||dp||^2 - D^2, where w is
            % the velocity (relative degree 1) or the acceleration (degree 2).
            h  = dp' * dp - D^2;
            g  = -2 * dp;
            if kind == 4
                hd = 2 * dp' * dv;
                b  = 2 * (dv' * dv) + (obj.alpha + obj.alpha2) * hd + obj.alpha * obj.alpha2 * h;
            else
                % dv is nonzero only against a moving obstacle (minus its velocity)
                b  = obj.alpha * h + 2 * dp' * dv;
            end
        end

        function u_hat = add_right_hand_bias(obj, u_hat, swarm)
            % Rotate part of each nominal command to the right when crowded.
            S = swarm.get_states();
            if size(S, 1) == 3, return; end       % not defined for unicycles
            P  = S(1:2, :);
            D  = obj.safe_distance(swarm);
            Dm = sqrt((P(1,:)' - P(1,:)).^2 + (P(2,:)' - P(2,:)).^2);
            Dm(1:swarm.N+1:end) = Inf;
            crowded = any(Dm < 2 * D, 2)';
            right   = [u_hat(2, :); -u_hat(1, :)];   % u_hat rotated by -90 degrees
            u_hat(:, crowded) = u_hat(:, crowded) + obj.right_hand_bias * right(:, crowded);
        end

        function [Ab, bb] = input_bounds(obj, swarm, kind)
            % Linear input bounds shared by every agent.
            dyn = swarm.agents{1}.dynamics;
            if kind == 3
                Ab = [eye(2); -eye(2)];
                bb = [dyn.max_speed; dyn.max_omega; dyn.max_speed; dyn.max_omega];
            else
                if kind == 4, umax = dyn.max_accel; else, umax = dyn.max_speed; end
                k   = obj.polygon_sides;
                phi = 2*pi*(0:k-1)'/k;
                Ab  = [cos(phi) sin(phi)];
                bb  = umax * cos(pi/k) * ones(k, 1);
            end
        end

        function [x, slack] = solve(obj, x_hat, A, b, Ab, bb)
            % Least-squares projection onto {A x <= b, Ab x <= bb}; relaxes the
            % CBF rows with a slack when that set is empty.
            nx = numel(x_hat);
            H  = 2 * eye(nx);
            f  = -2 * x_hat;
            slack = 0;
            if isempty(A)
                [x, ~, flag] = quadprog(H, f, Ab, bb, [], [], [], [], [], obj.qp_opts);
            else
                [x, ~, flag] = quadprog(H, f, [A; Ab], [b; bb], [], [], [], [], [], obj.qp_opts);
            end
            if flag == 1 && all(A*x <= b + 1e-6) && all(Ab*x <= bb + 1e-6)
                return;
            end
            % Relaxed problem: variables [x; s], A x - s <= b, s >= 0.
            nr = numel(b);
            H2 = blkdiag(2 * eye(nx), 2 * obj.slack_weight * eye(nr));
            f2 = [f; zeros(nr, 1)];
            A2 = [A, -eye(nr); Ab, zeros(size(Ab, 1), nr); zeros(nr, nx), -eye(nr)];
            b2 = [b; bb; zeros(nr, 1)];
            z  = quadprog(H2, f2, A2, b2, [], [], [], [], [], obj.qp_opts);
            x  = z(1:nx);
            slack = max([z(nx+1:end); 0]);
            if slack <= 1e-9
                slack = 1e-9;   % the first solve failed, so still count the step
            end
        end
    end
end

function B = blkdiag_rep(A, n)
    % blkdiag_rep(A, n) — block-diagonal matrix with n copies of A
    c = repmat({A}, 1, n);
    B = blkdiag(c{:});
end
