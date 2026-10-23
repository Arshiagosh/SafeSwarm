classdef Unicycle < handle
    %% Unicycle — non-holonomic differential-drive robot dynamics
    %
    % State:   [x; y; theta]  (2D position + heading angle in radians)
    % Control: [v; omega]      (linear speed + angular velocity)
    %
    % Models a robot that can only move in the direction it faces (non-holonomic
    % constraint). Required for PathFollowing with heading-aware control.
    % Heading is always wrapped to [-π, π].
    %
    % Example:
    %   dyn   = Unicycle(2.0, pi);
    %   state = dyn.step([0;0;0], [1; pi/4], 0.1);  % move forward, turn

    properties
        max_speed   = 2.0;  % maximum linear speed (m/s)
        max_omega   = pi;   % maximum angular velocity (rad/s)
        control_dim = 2;    % [v; omega]
    end

    methods
        function obj = Unicycle(max_speed, max_omega)
            % Unicycle(max_speed, max_omega)
            %   max_speed : scalar (default 2.0 m/s)
            %   max_omega : scalar (default π rad/s)
            if nargin > 0, obj.max_speed = max_speed; end
            if nargin > 1, obj.max_omega = max_omega; end
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds
            %   state : [x; y; theta]
            %   u     : [v; omega] linear speed and angular velocity
            %   dt    : timestep in seconds
            %   Returns: new [x; y; theta] with theta in [-π, π]
            v     = max(-obj.max_speed, min(obj.max_speed, u(1)));
            omega = max(-obj.max_omega,  min(obj.max_omega,  u(2)));

            theta     = state(3);
            new_state = state(:) + dt * [v*cos(theta); v*sin(theta); omega];

            % wrap heading to [-pi, pi]
            new_state(3) = atan2(sin(new_state(3)), cos(new_state(3)));
        end
    end
end
