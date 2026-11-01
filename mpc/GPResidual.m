classdef GPResidual < handle
    %% GPResidual — Gaussian-process model of an unknown, position-dependent disturbance
    %
    % Learns d(p) in  xdot = f(x) + g(x) (u + d(p))  from samples of
    % (position, observed residual), one GP per axis (fitrgp, squared-
    % exponential kernel with one length scale per input). predict() returns
    % the posterior mean and standard deviation; SafetyFilter uses the mean
    % to correct its constraints and kappa * std to tighten them.
    %
    % residual_samples() collects training data from a logged run: for a
    % single integrator the residual is (p_{k+1} - p_k)/dt - u_k, for a double
    % integrator (v_{k+1} - v_k)/dt - u_k.
    %
    % Requires the Statistics and Machine Learning Toolbox (fitrgp).
    %
    % Example:
    %   [X, Y] = GPResidual.residual_samples(logger, dt);
    %   gp = GPResidual(); gp.fit(X, Y);
    %   safe = SafetyFilter(nominal); safe.residual = gp; safe.kappa = 2;

    properties
        models = {};    % one RegressionGP per axis
        n_train = 0;
    end

    methods
        function obj = GPResidual()
            if ~(exist('fitrgp', 'file') == 2)
                error('GPResidual:NoToolbox', ...
                      'GPResidual needs fitrgp from the Statistics and Machine Learning Toolbox.');
            end
        end

        function fit(obj, X, Y)
            % fit(X, Y) — X: 2×M positions, Y: 2×M observed residuals
            obj.models = cell(1, 2);
            for a = 1:2
                obj.models{a} = fitrgp(X', Y(a, :)', 'KernelFunction', 'ardsquaredexponential', ...
                                       'BasisFunction', 'constant', 'Standardize', true);
            end
            obj.n_train = size(X, 2);
        end

        function [mu, sd] = predict(obj, P)
            % predict(P) — posterior mean and std (2×N each) at positions P (2×N)
            mu = zeros(2, size(P, 2));
            sd = zeros(2, size(P, 2));
            for a = 1:2
                [m, s] = predict(obj.models{a}, P');
                mu(a, :) = m';
                sd(a, :) = s';
            end
        end
    end

    methods (Static)
        function [X, Y] = residual_samples(logger, dt)
            % residual_samples(logger, dt) — training pairs from consecutive logged steps
            %   Uses the command stored with each step, so the run must have been
            %   logged by SimEngine with control_period equal to dt (or empty).
            X = []; Y = [];
            for k = 2:logger.log_count
                S0 = logger.state_log{k-1};
                S1 = logger.state_log{k};
                u  = logger.control_log{k};
                if isempty(u), continue; end
                if size(S0, 1) == 4
                    r = (S1(3:4, :) - S0(3:4, :)) / dt - u;
                else
                    r = (S1(1:2, :) - S0(1:2, :)) / dt - u;
                end
                X = [X, S0(1:2, :)]; %#ok<AGROW>
                Y = [Y, r];          %#ok<AGROW>
            end
        end
    end
end
