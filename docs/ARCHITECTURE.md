# SafeSwarm architecture

SafeSwarm keeps SwarmSim's rule that layers depend downward only, and that every behaviour is an
object with one method, `u = compute_control(swarm, env)`, returning a control_dim×N matrix.
Everything new (filters, planners, noise, faults, ROS 2 nodes) is built on that one interface, so
the pieces stack.

```text
┌───────────────────────────────────────────────────────────────────────┐
│  Experiments and tools                                                 │
│  experiments/ (E1-E4) · safeswarm_bench · MonteCarloRunner             │
│  MetricsAnalyzer · ExperimentRunner · ParamSweep · PublicationPlot     │
├───────────────────────────────────────────────────────────────────────┤
│  Wrappers (behaviours around behaviours)                               │
│  SafetyFilter · NoisyObservation · FaultInjector                       │
│  CollisionAvoidance · FormationWithObstacles      ros2/ BehaviourNode  │
├───────────────────────────────────────────────────────────────────────┤
│  Behaviours and planners                                               │
│  GoToGoal · DisplacementFormation · Flocking · LeaderFollower          │
│  VirtualStructure · PathFollowing · Aggregation · Dispersion           │
│  NMPCPlanner · AStar · RRT · PotentialField · TopicCommand (ros2/)     │
├───────────────────────────────────────────────────────────────────────┤
│  Core                                                                  │
│  SimEngine · Swarm · Agent · DataLogger · CommChannel · Environment    │
├───────────────────────────────────────────────────────────────────────┤
│  Dynamics                                                              │
│  SingleIntegrator · DoubleIntegrator · Unicycle   (bounds, f, g, d(p)) │
└───────────────────────────────────────────────────────────────────────┘
```

## One control step

```text
SimEngine.step
  ├─ env.update_dynamic_obstacles(dt)
  ├─ every control_period: u = behaviour.compute_control(swarm, env)     (held in between)
  │     e.g. NoisyObservation → FaultInjector → SafetyFilter → GoToGoal
  │          noisy states       override faulty    CBF-QP       nominal
  │                             robots             (CommChannel
  │                                                 in decentralised mode)
  ├─ agent.step(u_i, dt) for every agent           (saturation, disturbance, exact ZOH)
  ├─ contact handling ('none' for safety studies)
  └─ logger.log(t, swarm, env, u)                  (closest approach, clearance, effort)
```

## Where to extend

| To add | Write | Notes |
|---|---|---|
| a controller | a class with `compute_control(swarm, env)` | it can be wrapped by `SafetyFilter` at once |
| a robot model | a class with `step`, `saturate`, `f`, `g`, `state_dim`, `control_dim` | `SafetyFilter` needs a barrier for it (see `constraints`) |
| an uncertainty | a wrapper behaviour, like `NoisyObservation` | restore any state you change |
| an experiment | a script in `experiments/` using `make_scene` and `run_case` | save with `save_results(key, value)` |
| a ROS 2 interface | reuse `BehaviourNode` / `TopicCommand` / `RosTopics` | messages are stamped for synchronisation |

## Conventions

- Positions are the first two state entries for every model.
- Commands are saturated by the dynamics; the filters and the MPC respect the bounds themselves
  (inner polygon of the disc), so saturation should not change a filtered command.
- Every figure and number in the README is written by a script; nothing is edited by hand.
