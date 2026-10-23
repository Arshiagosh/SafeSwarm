classdef Unicycle < handle
    %% Unicycle — non-holonomic differential-drive robot dynamics
    %
    % State:   [x; y; theta]  (2D position + heading angle in radians)
    % Control: [v; omega]      (linear speed + angular velocity, box-bounded)
    %
    % Continuous model in control-affine form, xdot = f(x) + g(x) u, with
    % f(x) = 0 and g(x) = [cos(theta) 0; sin(theta) 0; 0 1]. step() is the
    % exact zero-order-hold solution (an arc of a circle when omega ~= 0).
    % Heading is always wrapped to [-π, π].
    %
    % Example:
    %   dyn   = Unicycle(2.0, pi);
    %   state = dyn.step([0;0;0], [1; pi/4], 0.1);  % move forward, turn

    properties
        max_speed   = 2.0;  % |v| <= max_speed (m/s)
        max_omega   = pi;   % |omega| <= max_omega (rad/s)
        state_dim   = 3;    % [x; y; theta]
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

        function [u, saturated] = saturate(obj, u)
            % saturate(u) — clip u to the box |v| <= max_speed, |omega| <= max_omega
            u  = u(:);
            hi = [obj.max_speed; obj.max_omega];
            u1 = min(max(u, -hi), hi);
            saturated = any(abs(u1 - u) > 1e-9 * max(1, abs(u)));
            u = u1;
        end

        function new_state = step(obj, state, u, dt)
            % step(state, u, dt) — advance state by dt seconds under constant u
            %   state : [x; y; theta]
            %   u     : [v; omega] (saturated first)
            %   Returns: new [x; y; theta] with theta in [-π, π]
            u     = obj.saturate(u);
            v     = u(1);
            omega = u(2);
            th    = state(3);
            if abs(omega) < 1e-9
                dp = v * dt * [cos(th); sin(th)];
            else
                dp = (v / omega) * [sin(th + omega*dt) - sin(th);
                                    cos(th) - cos(th + omega*dt)];
            end
            th1       = th + omega * dt;
            new_state = [state(1:2) + dp; atan2(sin(th1), cos(th1))];
        end

        function fx = f(~, x)
            % f(x) — drift term of xdot = f(x) + g(x) u
            fx = zeros(size(x(:)));
        end

        function gx = g(~, x)
            % g(x) — input matrix of xdot = f(x) + g(x) u
            gx = [cos(x(3)) 0; sin(x(3)) 0; 0 1];
        end
    end
end
