classdef SingleIntegrator < handle
    %% SingleIntegrator — first-order (velocity-controlled) point-mass dynamics
    %
    % State:   [x; y]      (2D position)
    % Control: [vx; vy]    (velocity; Euclidean norm bounded by max_speed)
    %
    % Continuous model in control-affine form, xdot = f(x) + g(x) u, with
    % f(x) = 0 and g(x) = I. Safety filters use f and g; the simulator uses
    % step(), which is the exact zero-order-hold solution.
    %
    % disturbance: optional function handle d(p) -> 2×1, added to the
    % (saturated) command inside step(). Controllers do not know it; it models
    % wind, slope or any unmodelled drift. Empty = none.
    %
    % Example:
    %   dyn   = SingleIntegrator(2.0);         % max 2 m/s
    %   state = dyn.step([0;0], [1;0.5], 0.1); % → [0.1; 0.05]

    properties
        max_speed   = 1.0;  % input bound ||u|| <= max_speed (m/s)
        state_dim   = 2;    % [x; y]
        control_dim = 2;    % [vx; vy]
        disturbance = [];   % optional @(p) -> 2×1 unknown drift added to u
    end

    methods
        function obj = SingleIntegrator(max_speed)
            % SingleIntegrator(max_speed)
            %   max_speed : scalar (default 1.0 m/s)
            if nargin > 0, obj.max_speed = max_speed; end
        end

        function [u, saturated] = saturate(obj, u)
            % saturate(u) — project u onto the input set ||u|| <= max_speed
            %   saturated is true when the projection changed u.
            u = u(:);
            n = norm(u);
            saturated = n > obj.max_speed * (1 + 1e-9);
            if saturated
                u = u * (obj.max_speed / n);
            end
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds under constant u
            %   state : [x; y]
            %   u     : [vx; vy] velocity command (saturated first)
            %   dt    : timestep in seconds
            u = obj.saturate(u);
            if ~isempty(obj.disturbance)
                u = u + obj.disturbance(state(1:2));
            end
            new_state = state(:) + u * dt;
        end

        function fx = f(~, x)
            % f(x) — drift term of xdot = f(x) + g(x) u
            fx = zeros(size(x(:)));
        end

        function gx = g(~, ~)
            % g(x) — input matrix of xdot = f(x) + g(x) u
            gx = eye(2);
        end

        function vel = get_velocity(~, ~, u)
            % get_velocity(state, u) — for SingleIntegrator, control IS velocity
            vel = u(:);
        end
    end
end
