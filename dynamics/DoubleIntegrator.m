classdef DoubleIntegrator < handle
    %% DoubleIntegrator — second-order (acceleration-controlled) point-mass dynamics
    %
    % State:   [x; y; vx; vy]   (2D position + velocity)
    % Control: [ax; ay]          (acceleration; Euclidean norm bounded by max_accel)
    %
    % Continuous model in control-affine form, xdot = f(x) + g(x) u, with
    % f(x) = [v; 0] and g(x) = [0; I]. step() is the exact zero-order-hold
    % solution as long as the speed limit is not reached. max_speed is a
    % state limit, not an input bound: when a step would exceed it, the
    % velocity is clipped (as in SwarmSim) and the clip is counted, so a
    % safety filter that should respect it can be checked. Use
    % max_speed = Inf to switch the clip off.
    %
    % disturbance: optional function handle d(p) -> 2×1, an unknown
    % acceleration added to the (saturated) command inside step(). Empty = none.
    %
    % Example:
    %   dyn   = DoubleIntegrator(3.0, 2.0);        % 3 m/s max, 2 m/s² max accel
    %   state = dyn.step([0;0;0;0], [1;0], 0.1);   % → [0.005; 0; 0.1; 0]

    properties
        max_speed   = 3.0;  % speed limit ||v|| <= max_speed (m/s), Inf disables
        max_accel   = 2.0;  % input bound ||u|| <= max_accel (m/s²)
        state_dim   = 4;    % [x; y; vx; vy]
        control_dim = 2;    % [ax; ay]
        speed_clips = 0;    % number of steps in which the velocity was clipped
        disturbance = [];   % optional @(p) -> 2×1 unknown acceleration added to u
    end

    methods
        function obj = DoubleIntegrator(max_speed, max_accel)
            % DoubleIntegrator(max_speed, max_accel)
            %   max_speed : scalar (default 3.0 m/s)
            %   max_accel : scalar (default 2.0 m/s²)
            if nargin > 0, obj.max_speed = max_speed; end
            if nargin > 1, obj.max_accel = max_accel; end
        end

        function [u, saturated] = saturate(obj, u)
            % saturate(u) — project u onto the input set ||u|| <= max_accel
            u = u(:);
            n = norm(u);
            saturated = n > obj.max_accel * (1 + 1e-9);
            if saturated
                u = u * (obj.max_accel / n);
            end
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds under constant u
            %   state : [x; y; vx; vy]
            %   u     : [ax; ay] acceleration command (saturated first)
            %   dt    : timestep in seconds
            u  = obj.saturate(u);
            if ~isempty(obj.disturbance)
                u = u + obj.disturbance(state(1:2));
            end
            p  = state(1:2);
            v  = state(3:4);
            v1 = v + u * dt;
            if norm(v1) > obj.max_speed
                % Speed limit reached: clip and use the mean velocity for position.
                v1 = v1 * (obj.max_speed / norm(v1));
                p1 = p + 0.5 * (v + v1) * dt;
                obj.speed_clips = obj.speed_clips + 1;
            else
                p1 = p + v * dt + 0.5 * u * dt^2;
            end
            new_state = [p1; v1];
        end

        function fx = f(~, x)
            % f(x) — drift term of xdot = f(x) + g(x) u
            fx = [x(3); x(4); 0; 0];
        end

        function gx = g(~, ~)
            % g(x) — input matrix of xdot = f(x) + g(x) u
            gx = [zeros(2); eye(2)];
        end
    end
end
