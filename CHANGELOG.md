# Changelog

## Unreleased (M7: ROS 2 bridge)

- ros2/: BehaviourNode (any behaviour or filter as a ROS 2 node), TopicCommand (commands from
  topics as a behaviour), SimRobotsNode (simulated robots), RosTopics (message conversions),
  demo_ros2_swap.m. 49 unit tests.

## Unreleased (M4: planning and learning)

- NMPCPlanner: distributed nonlinear MPC (fmincon SQP) with plan sharing, soft collision
  constraints, via points; single and double integrators.
- Dynamics: optional unknown disturbance d(p). SafetyFilter: `residual` model (mean corrects,
  kappa * std tightens every CBF row, including the speed barrier). GPResidual (fitrgp) and
  FunctionResidual.
- Experiment E4 (MPC vs CBF; GP residual under an unknown wind field); 47 unit tests.

## Unreleased (M3: uncertainty)

- CommChannel: per-link packet loss, delay in control steps, silent robots; each robot keeps the
  last state it received. The decentralised SafetyFilter uses it through its `comm` property.
- uncertainty/: NoisyObservation (sensor noise seen by the controllers only), FaultInjector
  (freeze, random walk, silent), MonteCarloRunner (seeded batches, Wilson intervals).
- Experiment E3 (noise, lossy and delayed messages, faulty robot); 40 unit tests.

## Unreleased (MVP: core and safety filters)

- Started from SwarmSim's core (dynamics, swarm, engine, behaviours, planners, tools).
- Dynamics: explicit input bounds with saturation counting, exact zero-order-hold steps,
  control-affine models f(x), g(x).
- Engine: `contact_mode` ('reflect' as in SwarmSim, or 'none' to keep collisions visible),
  public `step()`, dynamic obstacles moved by the engine, zero-order-hold `control_period`.
- Logger: closest approach, obstacle clearance, control effort, collision counts, `summary()`.
- New behaviours: DisplacementFormation, GoToGoal (with via points), FunctionBehaviour.
- Fixes: PathFollowing turning back, VirtualStructure hard-coded step and missing damping.
- SafetyFilter: CBF-QP, centralised and decentralised, single/double integrator and unicycle,
  obstacles, speed barrier, infeasibility logging, optional right-hand deadlock bias.
- Visualizer: GIF export, light theme, agent discs.
- Experiments E1 (benchmarks) and E2 (sampled-data sweep); 31 unit tests.
