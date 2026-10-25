classdef BehavioursTest < matlab.unittest.TestCase
    %% BehavioursTest — behaviours do what they claim on small, seeded swarms

    methods (TestMethodSetup)
        function seed(~)
            rng(1);
        end
    end

    methods (Test)
        function aggregationShrinksSpread(tc)
            swarm = BehavioursTest.make_swarm(10, 'si', 5);
            sim   = BehavioursTest.simulate(swarm, Aggregation(1.0), 5);
            s     = [sim.logger.metric_log.spread];
            tc.verifyLessThan(s(end), 0.1 * s(1));
        end

        function dispersionGrowsSpacing(tc)
            swarm = BehavioursTest.make_swarm(10, 'si', 1);
            sim   = BehavioursTest.simulate(swarm, Dispersion(1.5, 4.0), 10);
            d     = [sim.logger.metric_log.min_distance];
            tc.verifyGreaterThan(d(end), 3 * d(1));
        end

        function flockingAligns(tc)
            swarm = BehavioursTest.make_swarm(20, 'di', 4);
            sim   = BehavioursTest.simulate(swarm, Flocking(2.0, 1.0, 1.0, 3.0), 20);
            S     = sim.logger.state_log{end};
            V     = S(3:4, :);
            pol   = norm(mean(V ./ vecnorm(V), 2));
            tc.verifyGreaterThan(pol, 0.9);
        end

        function displacementFormationConverges(tc)
            for kind = ["si", "di"]
                swarm = BehavioursTest.make_swarm(6, char(kind), 4);
                ang   = 2*pi*(0:5)/6;
                shape = 3 * [cos(ang); sin(ang)];
                ring  = circshift(eye(6), 1) + circshift(eye(6), -1);
                behav = DisplacementFormation(shape, ring, 1.0);
                e0    = behav.formation_error(swarm);
                BehavioursTest.simulate(swarm, behav, 30);
                tc.verifyLessThan(behav.formation_error(swarm), 1e-3 * e0, ...
                    sprintf('formation did not converge for %s', kind));
            end
        end

        function formationIsTranslationFree(tc)
            % Shifting the target shape must not change the control.
            swarm = BehavioursTest.make_swarm(4, 'si', 3);
            shape = [0 2 2 0; 0 0 2 2];
            A     = ones(4) - eye(4);
            u1 = DisplacementFormation(shape, A).compute_control(swarm, []);
            u2 = DisplacementFormation(shape + [5; -7], A).compute_control(swarm, []);
            tc.verifyEqual(u1, u2, 'AbsTol', 1e-12);
        end

        function virtualStructureTracksMovingCentre(tc)
            % With dt = 0.1 (not SwarmSim's hard-coded 0.05) the centre still moves at v.
            swarm = BehavioursTest.make_swarm(5, 'di', 1);
            shape = 2 * [cos(2*pi*(1:5)/5); sin(2*pi*(1:5)/5)];
            vs    = VirtualStructure(shape, [0; 0]);
            vs.set_virtual_velocity([0.5; 0]);
            BehavioursTest.simulate(swarm, vs, 20, 0.1);
            c = mean(swarm.get_positions(), 2);
            tc.verifyEqual(c, vs.center_at(20), 'AbsTol', 0.2);
        end

        function pathFollowingReachesGoal(tc)
            swarm = Swarm({Agent(1, [0; 0; 0], Unicycle(1.5, pi))});
            path  = [0 5 10; 0 0 5];
            BehavioursTest.simulate(swarm, PathFollowing(path, 1.0), 20);
            tc.verifyLessThan(norm(swarm.get_positions() - path(:, end)), 0.5);
        end
    end

    methods (Static)
        function swarm = make_swarm(N, kind, scale)
            agents = cell(1, N);
            for i = 1:N
                switch kind
                    case 'si', agents{i} = Agent(i, randn(2,1)*scale, SingleIntegrator(3.0));
                    case 'di', agents{i} = Agent(i, [randn(2,1)*scale; randn(2,1)*0.5], ...
                                                 DoubleIntegrator(3.0, 2.0));
                end
            end
            swarm = Swarm(agents, 10.0, 'metric');
        end

        function sim = simulate(swarm, behav, t_max, dt)
            if nargin < 4, dt = 0.05; end
            env = Environment([-100 100], [-100 100]);
            sim = SimEngine(swarm, env, behav, dt, t_max);
            sim.contact_mode = 'none';
            sim.logger = DataLogger();
            sim.run();
        end
    end
end
