classdef SingleIntegrator < handle
    %% SingleIntegrator — first-order (velocity-controlled) point-mass dynamics
    %
    % State:   [x; y]      (2D position)
    % Control: [vx; vy]    (desired velocity; clamped to max_speed)
    %
    % Simplest dynamics model. The control input is treated directly as
    % velocity, making it ideal for algorithm prototyping where dynamics
    % are not the focus of study.
    %
    % Example:
    %   dyn   = SingleIntegrator(2.0);        % max 2 m/s
    %   state = dyn.step([0;0], [1;0.5], 0.1); % → [0.1; 0.05]

    properties
        max_speed   = 1.0;  % maximum speed (m/s)
        state_dim   = 2;    % [x; y]
        control_dim = 2;    % [vx; vy]
    end

    methods
        function obj = SingleIntegrator(max_speed)
            % SingleIntegrator(max_speed)
            %   max_speed : scalar (default 1.0 m/s)
            if nargin > 0, obj.max_speed = max_speed; end
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds
            %   state : [x; y]
            %   u     : [vx; vy] desired velocity
            %   dt    : timestep in seconds
            %   Returns: new [x; y]
            vel   = u(:);
            speed = norm(vel);
            if speed > obj.max_speed
                vel = vel / speed * obj.max_speed;
            end
            new_state = state(:) + vel * dt;
        end

        function vel = get_velocity(~, ~, u)
            % get_velocity(state, u) — for SingleIntegrator, control IS velocity
            vel = u(:);
        end
    end
end
