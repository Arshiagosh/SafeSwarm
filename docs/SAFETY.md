# Safety filter: the maths in one page

SafeSwarm's `SafetyFilter` wraps any behaviour. Every control step the behaviour proposes
$\hat u$ and the filter returns the closest command that satisfies control barrier function (CBF)
constraints and the input bounds.

## Barrier

For agents $i, j$ with positions $p_i, p_j$ and required centre distance $D$
(default $D = 2r + \text{margin}$, with $r$ the agent radius):

$$h_{ij} = \lVert p_i - p_j \rVert^2 - D^2 .$$

The safe set is $h_{ij} \ge 0$ for every pair. Obstacles use the same form, with the obstacle
centre (circle, distance $r_o + D/2$) or the closest point of a rectangle (distance $D/2$).

## Single integrator ($\dot p = u$)

$h$ has relative degree one. The CBF condition $\dot h \ge -\alpha h$ is linear in the inputs:

$$-2 (p_i - p_j)^\top (u_i - u_j) \le \alpha\, h_{ij}.$$

## Double integrator ($\ddot p = u$)

$h$ has relative degree two. SafeSwarm uses the high-order CBF of Xiao and Belta (CDC 2019):
with $\psi = \dot h + \alpha h$, require $\dot\psi \ge -\alpha_2 \psi$, which gives

$$-2 \Delta p^\top (u_i - u_j) \le 2\lVert \Delta v\rVert^2 + (\alpha + \alpha_2)\dot h + \alpha\alpha_2 h,
\qquad \dot h = 2\Delta p^\top \Delta v .$$

When the speed limit $v_{\max}$ is finite, a speed barrier $v_{\max}^2 - \lVert v_i\rVert^2 \ge 0$
adds $2 v_i^\top u_i \le \alpha_v (v_{\max}^2 - \lVert v_i\rVert^2)$.

## Unicycle

The single-integrator barrier is applied to a look-ahead point
$p_i + \ell [\cos\theta_i, \sin\theta_i]^\top$, whose velocity is
$\begin{bmatrix}\cos\theta & -\ell\sin\theta\\ \sin\theta & \ell\cos\theta\end{bmatrix}[v; \omega]$,
with $D$ enlarged by $2\ell$ so that the robot centres keep $D$.

## Centralised and decentralised QPs

Centralised: one QP over all inputs,

$$\min_u \sum_i \lVert u_i - \hat u_i \rVert^2 \quad \text{s.t. all CBF rows and input bounds.}$$

Decentralised: each agent solves a QP over its own input and keeps half of each pairwise row,
$-2\Delta p^\top u_i \le b_{ij}/2$ (the reciprocal split of Wang, Ames and Egerstedt, IEEE T-RO 2017).
If both agents keep their halves, the pair row holds. Each agent needs only its neighbours'
positions and velocities.

## Input bounds and infeasibility

The disc $\lVert u_i \rVert \le u_{\max}$ is replaced by an inscribed 16-sided polygon, so the
filtered command is never changed by the actuator saturation afterwards. The unicycle uses its box.
With bounded inputs the CBF rows and the bounds can be incompatible. The filter then solves the
QP again with a slack on the CBF rows (bounds stay hard), counts the step in `n_infeasible` and
logs the largest slack. It never returns the nominal command silently.

## What is not guaranteed

* The conditions above are continuous-time. Under a zero-order hold of $T$ seconds the
  guarantee can be lost; experiment E2 measures how much.
* Feasibility under input bounds is not guaranteed (see the infeasibility counts in E1).
* Deadlock: in symmetric scenes the filtered agents can stop short of their goals with every
  constraint satisfied. `RightHandBias` is a heuristic that often breaks it, not a proof.
