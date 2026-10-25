classdef FunctionBehaviour < handle
    %% FunctionBehaviour — turn a function handle into a behaviour
    %
    % Handy for quick experiments and tests: the handle receives the swarm and
    % the environment and returns the control_dim×N control matrix.
    %
    % Example:
    %   to_origin = FunctionBehaviour(@(swarm, env) -swarm.get_positions());
    %   sim       = SimEngine(swarm, env, to_origin);

    properties
        fn      % @(swarm, env) -> control_dim×N matrix
    end

    methods
        function obj = FunctionBehaviour(fn)
            % FunctionBehaviour(fn)
            %   fn : function handle @(swarm, env)
            obj.fn = fn;
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — evaluate the wrapped handle
            u = obj.fn(swarm, env);
        end
    end
end
