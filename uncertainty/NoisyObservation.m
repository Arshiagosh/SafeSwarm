classdef NoisyObservation < handle
    %% NoisyObservation — the wrapped behaviour sees noisy states
    %
    % Before calling the inner behaviour, zero-mean Gaussian noise is added
    % to every agent's position (std sigma_p) and velocity (std sigma_v,
    % double integrators) or heading (std sigma_theta, unicycles). The true
    % states are restored right after, so the simulation itself is exact;
    % only the controller is fooled. New noise is drawn on every call.
    %
    % Wrap the whole control stack, e.g. NoisyObservation(SafetyFilter(...)),
    % so the nominal controller and the filter see the same measurements.
    %
    % Example:
    %   safe  = SafetyFilter(GoToGoal(goals), 'Mode', 'decentralised');
    %   behav = NoisyObservation(safe, 0.05, 0.05);
    %   sim   = SimEngine(swarm, env, behav);

    properties
        inner                % wrapped behaviour
        sigma_p     = 0;     % position noise std (m)
        sigma_v     = 0;     % velocity noise std (m/s), double integrators
        sigma_theta = 0;     % heading noise std (rad), unicycles
    end

    methods
        function obj = NoisyObservation(inner, sigma_p, sigma_v, sigma_theta)
            % NoisyObservation(inner, sigma_p, sigma_v, sigma_theta)
            obj.inner = inner;
            if nargin > 1, obj.sigma_p     = sigma_p;     end
            if nargin > 2, obj.sigma_v     = sigma_v;     end
            if nargin > 3, obj.sigma_theta = sigma_theta; end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — inner control computed on noisy states
            n    = swarm.N;
            keep = cell(1, n);
            for i = 1:n
                a = swarm.agents{i};
                keep{i} = a.state;
                x = a.state;
                x(1:2) = x(1:2) + obj.sigma_p * randn(2, 1);
                if numel(x) == 4
                    x(3:4) = x(3:4) + obj.sigma_v * randn(2, 1);
                elseif numel(x) == 3
                    x(3) = x(3) + obj.sigma_theta * randn;
                end
                a.state = x;
            end
            swarm.invalidate_cache();
            try
                u = obj.inner.compute_control(swarm, env);
            catch err
                restore(swarm, keep);
                rethrow(err);
            end
            restore(swarm, keep);
        end
    end
end

function restore(swarm, keep)
    for i = 1:swarm.N
        swarm.agents{i}.state = keep{i};
    end
    swarm.invalidate_cache();
end
