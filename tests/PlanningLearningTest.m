classdef PlanningLearningTest < matlab.unittest.TestCase
    %% PlanningLearningTest — NMPC planner, disturbances and the GP residual

    methods (TestMethodSetup)
        function seed(~)
            rng(4);
        end
    end

    methods (Test)
        function mpcPredictionMatchesSimulation(tc)
            % The plan's first predicted position must equal where the robot ends up.
            swarm = Swarm({Agent(1, [0; 0; 0.3; -0.2], DoubleIntegrator(1, 1))});
            mpc   = NMPCPlanner([4; 2], 'Horizon', 8, 'Tau', 0.2);
            sim   = SimEngine(swarm, Environment([-9 9], [-9 9]), mpc, 0.05, 0.2);
            sim.control_period = 0.2;
            sim.run();
            tc.verifyEqual(swarm.get_positions(), mpc.plans(:, 2, 1), 'AbsTol', 1e-9);
        end

        function mpcRespectsBounds(tc)
            swarm = Swarm({Agent(1, [0; 0; 0; 0], DoubleIntegrator(0.8, 1))});
            mpc   = NMPCPlanner([6; 0]);
            sim   = SimEngine(swarm, Environment([-9 9], [-9 9]), mpc, 0.05, 15);
            sim.control_period = 0.2;
            sim.run();
            tc.verifyEqual(swarm.agents{1}.saturation_count, 0);
            tc.verifyEqual(swarm.agents{1}.dynamics.speed_clips, 0);
            tc.verifyLessThan(norm(swarm.get_positions() - [6; 0]), 0.1);
        end

        function mpcPairPassesWithoutContact(tc)
            a1 = Agent(1, [-3; 0.1], SingleIntegrator(1));
            a2 = Agent(2, [ 3; 0],   SingleIntegrator(1));
            swarm = Swarm({a1, a2}, 10, 'full');
            swarm.agent_radius = 0.3;
            mpc = NMPCPlanner([3 -3; 0 0]);
            sim = SimEngine(swarm, Environment([-6 6], [-6 6]), mpc, 0.05, 15);
            sim.contact_mode = 'none'; sim.control_period = 0.2; sim.logger = DataLogger();
            sim.run();
            tc.verifyGreaterThanOrEqual(sim.logger.summary().min_distance, 0.6);
            tc.verifyLessThan(max(vecnorm(swarm.get_positions() - [3 -3; 0 0])), 0.2);
        end

        function disturbanceEntersLikeInput(tc)
            d = SingleIntegrator(1);
            d.disturbance = @(p) [0.2; -0.1];
            tc.verifyEqual(d.step([0; 0], [5; 0], 0.5), [0.6; -0.05], 'AbsTol', 1e-12);
        end

        function gpLearnsSmoothField(tc)
            f = @(P) [sin(P(1,:)); 0.5*cos(P(2,:))];
            X = 4 * rand(2, 150) - 2;
            Y = f(X) + 0.02 * randn(2, 150);
            gp = GPResidual(); gp.fit(X, Y);
            Pt = 3 * rand(2, 50) - 1.5;
            [mu, sd] = gp.predict(Pt);
            tc.verifyLessThan(max(abs(mu - f(Pt)), [], 'all'), 0.1);
            [~, sd_far] = gp.predict([10; 10]);
            tc.verifyGreaterThan(min(sd_far), 5 * max(sd, [], 'all'));
        end

        function residualKeepsPairApart(tc)
            % A drift pushes two resting robots together; only a filter that
            % knows the drift keeps them D apart.
            push = @(p) [-0.4 * tanh(p(1)); 0];
            for known = [false true]
                a1 = Agent(1, [-0.6; 0], SingleIntegrator(1));
                a2 = Agent(2, [ 0.6; 0], SingleIntegrator(1));
                a1.dynamics.disturbance = push; a2.dynamics.disturbance = push;
                swarm = Swarm({a1, a2}, 10, 'full');
                swarm.agent_radius = 0.3;
                f = SafetyFilter(FunctionBehaviour(@(sw, ~) zeros(2, 2)), 'Mode', 'centralised');
                if known, f.residual = FunctionResidual(push); end
                sim = SimEngine(swarm, Environment([-5 5], [-5 5]), f, 0.01, 8);
                sim.contact_mode = 'none'; sim.logger = DataLogger();
                sim.run();
                dmin(known + 1) = sim.logger.summary().min_distance; %#ok<AGROW>
            end
            tc.verifyLessThan(dmin(1), 0.6);
            tc.verifyGreaterThanOrEqual(dmin(2), 0.65 - 1e-3);
        end

        function residualSamplesRecoverDrift(tc)
            a = Agent(1, [0; 0; 0; 0], DoubleIntegrator(Inf, 1));
            a.dynamics.disturbance = @(p) [0.3; -0.2];
            swarm = Swarm({a});
            sim = SimEngine(swarm, Environment([-9 9], [-9 9]), ...
                            FunctionBehaviour(@(sw, ~) [0.1; 0.1]), 0.05, 1);
            sim.logger = DataLogger();
            sim.run();
            [X, Y] = GPResidual.residual_samples(sim.logger, 0.05);
            tc.verifySize(X, [2 20]);
            tc.verifyEqual(Y, repmat([0.3; -0.2], 1, 20), 'AbsTol', 1e-9);
        end
    end
end
