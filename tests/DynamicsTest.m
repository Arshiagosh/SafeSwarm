classdef DynamicsTest < matlab.unittest.TestCase
    %% DynamicsTest — dynamics steps match closed-form solutions and respect bounds

    methods (Test)
        function singleIntegratorStep(tc)
            d = SingleIntegrator(2.0);
            tc.verifyEqual(d.step([1; 2], [1; -0.5], 0.1), [1.1; 1.95], 'AbsTol', 1e-12);
        end

        function singleIntegratorBound(tc)
            d = SingleIntegrator(1.0);
            [u, sat] = d.saturate([3; 4]);
            tc.verifyEqual(norm(u), 1.0, 'AbsTol', 1e-12);
            tc.verifyTrue(sat);
            [~, sat] = d.saturate([0.6; 0.8]);
            tc.verifyFalse(sat);
        end

        function doubleIntegratorExactZOH(tc)
            % p(t) = p0 + v0 t + a t^2 / 2 for constant a below every limit
            d  = DoubleIntegrator(Inf, 2.0);
            x0 = [1; -1; 0.5; 0.2];
            a  = [0.3; -0.4];
            x  = x0;
            for k = 1:20
                x = d.step(x, a, 0.05);
            end
            T  = 1.0;
            tc.verifyEqual(x(1:2), x0(1:2) + x0(3:4)*T + 0.5*a*T^2, 'AbsTol', 1e-12);
            tc.verifyEqual(x(3:4), x0(3:4) + a*T, 'AbsTol', 1e-12);
        end

        function doubleIntegratorBounds(tc)
            d = DoubleIntegrator(1.0, 2.0);
            x = [0; 0; 0; 0];
            for k = 1:100
                x = d.step(x, [10; 0], 0.05);
                tc.verifyLessThanOrEqual(norm(x(3:4)), 1.0 + 1e-12);
            end
            tc.verifyGreaterThan(d.speed_clips, 0);
        end

        function unicycleExactArc(tc)
            % Constant v and omega trace a circle of radius v/omega.
            d = Unicycle(2.0, pi);
            x = [0; 0; 0];
            v = 1.0; w = 0.5;
            for k = 1:40
                x = d.step(x, [v; w], 0.1);
            end
            T  = 4.0;
            ex = [v/w*sin(w*T); v/w*(1 - cos(w*T)); w*T];
            tc.verifyEqual(x, ex, 'AbsTol', 1e-12);
        end

        function unicycleBox(tc)
            d = Unicycle(1.0, 0.5);
            [u, sat] = d.saturate([3; -2]);
            tc.verifyEqual(u, [1.0; -0.5]);
            tc.verifyTrue(sat);
        end

        function controlAffineModels(tc)
            % f(x) + g(x) u must equal the time derivative used by step().
            u  = [0.4; -0.3];
            si = SingleIntegrator(5); x = [1; 2];
            tc.verifyEqual(si.f(x) + si.g(x)*u, u);
            di = DoubleIntegrator(Inf, 5); x = [1; 2; 0.3; -0.1];
            tc.verifyEqual(di.f(x) + di.g(x)*u, [x(3:4); u]);
            un = Unicycle(5, 5); x = [1; 2; pi/3];
            tc.verifyEqual(un.f(x) + un.g(x)*u, [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)], 'AbsTol', 1e-15);
        end

        function agentCountsSaturation(tc)
            a = Agent(1, [0; 0], SingleIntegrator(1.0));
            a.step([0.5; 0], 0.1);
            a.step([5; 0], 0.1);
            tc.verifyEqual(a.saturation_count, 1);
            tc.verifyEqual(a.position, [0.15; 0], 'AbsTol', 1e-12);
        end
    end
end
