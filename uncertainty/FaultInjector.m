classdef FaultInjector < handle
    %% FaultInjector — make some agents misbehave from a given time on
    %
    % Wraps the control stack and overrides the commands of faulty agents
    % after the stack has run, so a faulty agent ignores its safety filter
    % and the others have to avoid it on their own. Fault modes:
    %
    %   'freeze'      the agent stops (single integrator and unicycle: u = 0;
    %                 double integrator: maximum braking, then rest).
    %   'random_walk' the agent wanders: full-speed command in a direction
    %                 that drifts randomly (heading step std 0.5 rad per call).
    %   'silent'      the agent keeps its normal command but stops sending
    %                 messages (needs the CommChannel used by the filter).
    %
    % faults is a struct array with fields agent, mode, t_start.
    %
    % Example:
    %   f = struct('agent', 3, 'mode', 'freeze', 't_start', 5);
    %   behav = FaultInjector(SafetyFilter(nominal), f);

    properties
        inner            % wrapped behaviour
        faults           % struct array: agent, mode, t_start
        comm = [];       % CommChannel for 'silent' faults
        heading = [];    % 1×N random-walk directions
    end

    methods
        function obj = FaultInjector(inner, faults, comm)
            % FaultInjector(inner, faults, comm)
            obj.inner  = inner;
            obj.faults = faults;
            if nargin > 2, obj.comm = comm; end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — inner control with faulty agents overridden
            % Silence first, so the inner filter's broadcast round already misses them.
            for f = obj.faults(:)'
                if strcmp(f.mode, 'silent') && swarm.time >= f.t_start && ~isempty(obj.comm)
                    if isempty(obj.comm.silent), obj.comm.silent = false(1, swarm.N); end
                    obj.comm.silent(f.agent) = true;
                end
            end
            u = obj.inner.compute_control(swarm, env);
            if isempty(obj.heading), obj.heading = 2*pi*rand(1, swarm.N); end

            for f = obj.faults(:)'
                if swarm.time < f.t_start, continue; end
                a   = swarm.agents{f.agent};
                dyn = a.dynamics;
                switch f.mode
                    case 'freeze'
                        if numel(a.state) == 4
                            u(:, f.agent) = -a.state(3:4) * 1e3;   % saturates to max braking
                        else
                            u(:, f.agent) = 0;
                        end
                    case 'random_walk'
                        obj.heading(f.agent) = obj.heading(f.agent) + 0.5 * randn;
                        d = [cos(obj.heading(f.agent)); sin(obj.heading(f.agent))];
                        if numel(a.state) == 4
                            u(:, f.agent) = dyn.max_accel * d;
                        elseif numel(a.state) == 3
                            u(:, f.agent) = [dyn.max_speed; dyn.max_omega * (2*rand - 1)];
                        else
                            u(:, f.agent) = dyn.max_speed * d;
                        end
                    case 'silent'
                        % command unchanged; handled above
                    otherwise
                        error('FaultInjector:Mode', 'Unknown fault mode %s', f.mode);
                end
            end
        end
    end
end
