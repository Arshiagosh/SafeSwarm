classdef FunctionResidual < handle
    %% FunctionResidual — a disturbance model given as a function (no uncertainty)
    %
    % Same interface as GPResidual. Useful as an oracle (the true
    % disturbance) or for a hand-made model. sd is always zero.
    %
    % Example:
    %   safe.residual = FunctionResidual(@(p) [0.2; 0]);

    properties
        fn      % @(p) -> 2×1 for one position p (2×1)
    end

    methods
        function obj = FunctionResidual(fn)
            obj.fn = fn;
        end

        function [mu, sd] = predict(obj, P)
            % predict(P) — fn at every column of P, zero std
            mu = zeros(2, size(P, 2));
            for k = 1:size(P, 2)
                mu(:, k) = obj.fn(P(:, k));
            end
            sd = zeros(size(mu));
        end
    end
end
