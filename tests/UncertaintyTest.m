classdef UncertaintyTest < matlab.unittest.TestCase
    %% UncertaintyTest — noise, communication, faults and the Monte Carlo runner

    methods (TestMethodSetup)
        function seed(~)
            rng(2);
        end
    end

    methods (Test)
        function noiseDoesNotTouchTrueState(tc)
            swarm = UncertaintyTest.pair('di');
            S0    = swarm.get_states();
            seen  = {};
            probe = FunctionBehaviour(@(sw, ~) record(sw));
            NoisyObservation(probe, 0.1, 0.1).compute_control(swarm, []);
            tc.verifyEqual(swarm.get_states(), S0);                  % restored
            tc.verifyGreaterThan(norm(seen{1} - S0, 'fro'), 0);      % but noisy inside
            function u = record(sw)
                seen{end+1} = sw.get_states();
                u = zeros(2, sw.N);
            end
        end

        function noiseHasRightSpread(tc)
            swarm = Swarm({Agent(1, [0; 0], SingleIntegrator(1))});
            obs   = zeros(2, 4000);
            k     = 0;
            probe = FunctionBehaviour(@(sw, ~) grab(sw));
            nb    = NoisyObservation(probe, 0.2);
            for t = 1:4000, nb.compute_control(swarm, []); end
            tc.verifyEqual(std(obs, 0, 2), [0.2; 0.2], 'RelTol', 0.06);
            function u = grab(sw)
                k = k + 1; obs(:, k) = sw.get_positions(); u = [0; 0];
            end
        end

        function delayDeliversOldStates(tc)
            ch = CommChannel(Inf, 0, 2);
            for t = 1:5
                ch.update([t t+10; 0 0]);
            end
            V = ch.view(1, [5 15; 0 0]);
            tc.verifyEqual(V(:, 1), [5; 0]);       % own state exact
            tc.verifyEqual(V(:, 2), [13; 0]);      % neighbour's state 2 steps old
        end

        function totalLossKeepsFirstState(tc)
            ch = CommChannel(Inf, 1.0, 0);
            ch.update([0 1; 0 0]);
            ch.update([0 7; 0 0]);
            V = ch.view(1, [0 7; 0 0]);
            tc.verifyEqual(V(:, 2), [1; 0]);
            tc.verifyEqual(ch.n_lost, ch.n_sent);
        end

        function silentAgentIsNotHeard(tc)
            ch = CommChannel();
            ch.update([0 1; 0 0]);
            ch.silent = [false true];
            ch.update([0 4; 0 0]);
            V1 = ch.view(1, [0 4; 0 0]);
            V2 = ch.view(2, [9 4; 0 0]);
            tc.verifyEqual(V1(:, 2), [1; 0]);      % 1 does not hear 2
            tc.verifyEqual(V2(:, 1), [0; 0]);      % 2 still hears 1
        end

        function filterUsesReceivedStates(tc)
            % With every message lost, robot 1 thinks robot 2 is still far away
            % and does not brake, so the decentralised filter fails to keep D.
            for loss = [0 1]
                swarm = UncertaintyTest.pair('si');
                f = SafetyFilter(GoToGoal([3 -3; 0 0]), 'Mode', 'decentralised');
                f.comm = CommChannel(Inf, loss, 0);
                sim = SimEngine(swarm, Environment([-6 6], [-6 6]), f, 0.02, 6);
                sim.contact_mode = 'none';
                sim.logger = DataLogger();
                sim.run();
                d(loss + 1) = sim.logger.summary().min_distance; %#ok<AGROW>
            end
            tc.verifyGreaterThanOrEqual(d(1), 0.65 - 1e-3);
            tc.verifyLessThan(d(2), 0.6);
        end

        function freezeStopsAgent(tc)
            swarm = UncertaintyTest.pair('si');
            f   = struct('agent', 2, 'mode', 'freeze', 't_start', 0);
            sim = SimEngine(swarm, Environment([-6 6], [-6 6]), ...
                            FaultInjector(GoToGoal([3 -3; 0 0]), f), 0.05, 2);
            sim.run();
            tc.verifyEqual(swarm.agents{2}.position, [3; 0]);
            tc.verifyLessThan(swarm.agents{1}.position(1), -1);   % the other one moved
        end

        function randomWalkUsesFullInput(tc)
            swarm = UncertaintyTest.pair('si');
            fi = FaultInjector(GoToGoal([3 -3; 0 0]), struct('agent', 1, 'mode', 'random_walk', 't_start', 0));
            u  = fi.compute_control(swarm, []);
            tc.verifyEqual(norm(u(:, 1)), 1, 'AbsTol', 1e-12);
        end

        function monteCarloIsReproducible(tc)
            fn = @(seed) struct('x', randn);
            r1 = MonteCarloRunner.run(fn, 1:5);
            r2 = MonteCarloRunner.run(fn, 1:5);
            tc.verifyEqual([r1.x], [r2.x]);
            s = MonteCarloRunner.summarise(r1, 'x');
            tc.verifyEqual(s.mean, mean([r1.x]), 'AbsTol', 1e-12);
            [p, lo, hi] = MonteCarloRunner.rate(r1, @(r) r.x > 0);
            tc.verifyEqual(p, mean([r1.x] > 0));
            tc.verifyLessThanOrEqual(lo, p);
            tc.verifyGreaterThanOrEqual(hi, p);
        end
    end

    methods (Static)
        function swarm = pair(kind)
            if strcmp(kind, 'si')
                a1 = Agent(1, [-3; 0.05], SingleIntegrator(1));
                a2 = Agent(2, [ 3; 0],    SingleIntegrator(1));
            else
                a1 = Agent(1, [-3; 0.05; 0.1; 0], DoubleIntegrator(1, 1));
                a2 = Agent(2, [ 3; 0;   -0.1; 0], DoubleIntegrator(1, 1));
            end
            swarm = Swarm({a1, a2}, 10, 'full');
            swarm.agent_radius = 0.3;
        end
    end
end
