classdef QLearningPolicy < handle
    %% QLearningPolicy — independent tabular Q-learning for single integrators
    %
    % Each robot learns its own Q-table over a grid of its own position and
    % five actions (full speed up, down, left, right, or stay), ignoring the
    % other robots, as in independent Q-learning (Tan, ICML 1993). Training
    % runs in the robot's own dynamics with each action held for tau seconds:
    %
    %   reward  -0.1 per step, +100 on reaching the goal (within goal_tol = final_radius),
    %           -10 and episode end for leaving the map,
    %   update  Q(s,a) += alpha (r + gamma max_a' Q(s',a') - Q(s,a)),
    %   epsilon-greedy exploration, epsilon decaying per episode.
    %
    % The learned policy knows nothing about the other robots, so it collides
    % unless something else prevents it: wrap it in a SafetyFilter to get a
    % learned policy with a safety layer on top (a "shield").
    % Within final_radius of its goal a robot drives straight to it, because a
    % grid policy cannot stop exactly on a point.
    %
    % Works with: SingleIntegrator.
    %
    % Example:
    %   pol = QLearningPolicy(goals, env);
    %   pol.train(swarm, 3000);
    %   safe = SafetyFilter(pol, 'Mode', 'decentralised');

    properties
        goals                   % 2×N goal positions
        x_lim                   % map bounds, from the environment
        y_lim
        cell     = 1.0;         % grid cell size (m)
        tau      = 0.25;        % action duration during training (s)
        alpha    = 0.2;
        gamma    = 0.95;
        epsilon0 = 0.3;
        epsilon_min = 0.02;
        goal_tol = 0.8;         % same as final_radius: inside it the robot drives straight in
        final_radius = 0.8;
        max_steps = 150;        % per training episode
        Q = {};                 % 1×N cell of nx×ny×5 tables
        returns = [];           % N×episodes training returns
    end

    properties (Constant)
        DIRS = [0 0 -1 1 0; 1 -1 0 0 0];   % up, down, left, right, stay
    end

    methods
        function obj = QLearningPolicy(goals, env)
            obj.goals = goals;
            obj.x_lim = env.x_lim;
            obj.y_lim = env.y_lim;
        end

        function train(obj, swarm, episodes)
            % train(swarm, episodes) — learn one Q-table per robot (robots trained alone)
            n  = swarm.N;
            nx = ceil(diff(obj.x_lim) / obj.cell);
            ny = ceil(diff(obj.y_lim) / obj.cell);
            obj.Q = repmat({zeros(nx, ny, 5)}, 1, n);
            obj.returns = zeros(n, episodes);
            for i = 1:n
                vmax = swarm.agents{i}.dynamics.max_speed;
                Qi = obj.Q{i};
                g = obj.goals(:, i);
                for ep = 1:episodes
                    eps_ = max(obj.epsilon_min, obj.epsilon0 * (1 - ep / (0.8 * episodes)));
                    p = [obj.x_lim(1) + diff(obj.x_lim) * rand; obj.y_lim(1) + diff(obj.y_lim) * rand];
                    [cx, cy] = obj.cell_of(p);
                    G = 0;
                    for k = 1:obj.max_steps
                        if rand < eps_
                            a = randi(5);
                        else
                            [~, a] = max(Qi(cx, cy, :));
                        end
                        p = p + vmax * obj.tau * obj.DIRS(:, a);   % exact SingleIntegrator step
                        if p(1) < obj.x_lim(1) || p(1) > obj.x_lim(2) || ...
                           p(2) < obj.y_lim(1) || p(2) > obj.y_lim(2)
                            r = -10; done = true; nxt = 0;
                        elseif norm(p - g) <= obj.goal_tol
                            r = 100; done = true; nxt = 0;
                        else
                            r = -0.1; done = false;
                            [nx_, ny_] = obj.cell_of(p);
                            nxt = max(Qi(nx_, ny_, :));
                        end
                        Qi(cx, cy, a) = Qi(cx, cy, a) + obj.alpha * (r + obj.gamma * nxt - Qi(cx, cy, a));
                        G = G + r;
                        if done, break; end
                        cx = nx_; cy = ny_;
                    end
                    obj.returns(i, ep) = G;
                end
                obj.Q{i} = Qi;
            end
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — greedy action of each robot's Q-table
            P = swarm.get_positions();
            u = zeros(2, swarm.N);
            for i = 1:swarm.N
                e = obj.goals(:, i) - P(:, i);
                vmax = swarm.agents{i}.dynamics.max_speed;
                if norm(e) <= obj.final_radius
                    u(:, i) = e;
                else
                    [cx, cy] = obj.cell_of(P(:, i));
                    [~, a] = max(obj.Q{i}(cx, cy, :));
                    u(:, i) = vmax * obj.DIRS(:, a);
                end
            end
        end
    end

    methods (Access = private)
        function [cx, cy] = cell_of(obj, p)
            nx = size(obj.Q{1}, 1); ny = size(obj.Q{1}, 2);
            cx = min(max(floor((p(1) - obj.x_lim(1)) / obj.cell) + 1, 1), nx);
            cy = min(max(floor((p(2) - obj.y_lim(1)) / obj.cell) + 1, 1), ny);
        end
    end
end
