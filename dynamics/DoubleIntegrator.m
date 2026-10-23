classdef DoubleIntegrator < handle
    %% DoubleIntegrator — second-order (acceleration-controlled) point-mass dynamics
    %
    % State:   [x; y; vx; vy]   (2D position + velocity)
    % Control: [ax; ay]          (acceleration; clamped to max_accel)
    %
    % More physically realistic than SingleIntegrator. Velocity builds up
    % over time (inertia), so behaviours must account for overshoot.
    % Used by most SwarmSim scenarios by default.
    %
    % Example:
    %   dyn   = DoubleIntegrator(3.0, 2.0);        % 3 m/s max, 2 m/s² max accel
    %   state = dyn.step([0;0;0;0], [1;0], 0.1);   % → [0; 0; 0.1; 0]

    properties
        max_speed = 3.0;    % maximum speed (m/s)
        max_accel = 2.0;    % maximum acceleration (m/s²)
        control_dim = 2;    % [ax; ay]
    end

    methods
        function obj = DoubleIntegrator(max_speed, max_accel)
            % DoubleIntegrator(max_speed, max_accel)
            %   max_speed : scalar (default 3.0 m/s)
            %   max_accel : scalar (default 2.0 m/s²)
            if nargin > 0, obj.max_speed = max_speed; end
            if nargin > 1, obj.max_accel = max_accel; end
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds
            %   state : [x; y; vx; vy]
            %   u     : [ax; ay] desired acceleration
            %   dt    : timestep in seconds
            %   Returns: new [x; y; vx; vy]
            u = u(:);
            if norm(u) > obj.max_accel
                u = u * (obj.max_accel / norm(u));
            end

            vel = state(3:4) + u * dt;
            if norm(vel) > obj.max_speed
                vel = vel * (obj.max_speed / norm(vel));
            end

            pos = state(1:2) + vel * dt;
            new_state = [pos; vel];
        end
    end
end
