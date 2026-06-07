# autopilot

A real-time iterative Model Predictive Control (iMPC) trajectory-tracking
controller for autonomous ground vehicles, built with
[CVXPY](https://www.cvxpy.org/) and validated against a
[MuJoCo](https://mujoco.org/) physics simulation.

<figure>
  <img src="img/banner.png" width="500" />
  <figcaption>MuJoCo simulation with the MUSHR car model</figcaption>
</figure>

The controller solves a convex quadratic program (QP) at every control step.
To handle the nonlinear vehicle kinematics while staying convex, it uses
*iterative linearization*: the predicted trajectory is linearized point by
point along the horizon and the QP is re-solved until the linear model
converges onto the true curved dynamics. This keeps each solve fast and
reliable while remaining accurate through sharp turns.

Author: **Harsh Patil**.

## Features

- Iterative-linearization MPC over a kinematic bicycle model.
- Track-relative cost (along-track / cross-track / velocity / heading errors)
  rather than raw XY tracking — the natural formulation for path following.
- Soft half-plane obstacle-avoidance constraints with a slack penalty, so the
  solver degrades gracefully instead of becoming infeasible when an obstacle
  cannot be perfectly avoided.
- Static and moving obstacle support, plus a field-of-view "camera" detector.
- DPP-compliant CVXPY problem built once and warm-started every cycle.
- Lean core library: depends only on the numerical/optimization stack.
  Visualization and physics (`matplotlib`, `mujoco`) are optional extras
  needed only by the example demos.

## Installation

Requires Python 3.11+.

```bash
git clone <your-fork-url> autopilot
cd autopilot

pip install -e .             # core library only
pip install -e ".[demos]"    # + matplotlib and mujoco for the example demos
pip install -e ".[dev]"      # + tests, ruff, black
```

### Conda

```bash
conda env create -f env.yml
conda activate autopilot
pip install -e .
```

### Nix flake ❄️

```bash
nix run --impure .#mujoco-demo   # GUI demo
nix run .#nosim-demo             # headless demo
nix develop                      # development shell
```

## Quick start

```python
import numpy as np
from autopilot import MPC
from autopilot.resources import MPC_CONFIG
from autopilot.utils import compute_path_from_wp, get_ref_trajectory

# Build the controller from the bundled config.
mpc = MPC(MPC_CONFIG, horizon_time=4.0)

# Generate a smooth path through some waypoints.
path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 2, 4], step=0.05)

# Build a reference trajectory over the horizon and solve.
state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, v, heading]
target = get_ref_trajectory(
    state, path, target_v=1.0, T=mpc.control_horizon * mpc.dt, DT=mpc.dt
)
x_opt, u_opt = mpc.solve(state, target, obstacle=None)

accel, steer = u_opt[:, 0]  # first optimal control to apply
```

`MPC` accepts a path, a `pathlib.Path`, or a raw config `dict`. Bundled config
is resolved relative to the package, so `MPC("mpc.yaml")` works from any
working directory.

## Examples

Two runnable demos live under [`examples/`](examples/). Install the `demos`
extra first, then run them directly:

```bash
pip install -e ".[demos]"

python examples/demo_nosim.py     # headless Matplotlib simulation
python examples/demo_mujoco.py    # MuJoCo physics simulation (needs a display)
```

Each demo reads its scenario — waypoints, target speed, sensor field of view,
and obstacles — from [`examples/simulation.yaml`](examples/simulation.yaml).
Edit that file to change the course, or set `obstacles: []` to disable
obstacle avoidance.

<table>
  <tr>
    <td><figure><img src="img/demo-mujoco.gif" width="500" /><figcaption>MuJoCo, no obstacles</figcaption></figure></td>
    <td><figure><img src="img/demo.gif" width="500" /><figcaption>Headless, no obstacles</figcaption></figure></td>
  </tr>
  <tr>
    <td><figure><img src="img/demo-mujoco_with_obs.gif" width="500" /><figcaption>MuJoCo, static obstacle avoidance</figcaption></figure></td>
    <td><figure><img src="img/demo_with_obs.gif" width="500" /><figcaption>Headless, static obstacle avoidance</figcaption></figure></td>
  </tr>
  <tr>
    <td><figure><img src="img/demo-mujoco_with_moving_obs.gif" width="500" /><figcaption>MuJoCo, moving obstacle avoidance</figcaption></figure></td>
    <td><figure><img src="img/demo_with_moving_obs.gif" width="500" /><figcaption>Headless, moving obstacle avoidance</figcaption></figure></td>
  </tr>
</table>

## Project layout

```
autopilot/
├── pyproject.toml          # packaging, dependencies, tool config
├── env.yml                 # conda environment
├── flake.nix               # Nix dev shell + run targets
├── CHANGELOG.md
├── CONTRIBUTING.md
├── img/                    # banner and demo gifs
├── examples/               # runnable demos + their scenario config
│   ├── demo_nosim.py
│   ├── demo_mujoco.py
│   └── simulation.yaml
├── tests/                  # pytest suite
└── src/
    └── autopilot/          # the installable package
        ├── controller.py   # MPC controller (start here)
        ├── utils.py        # path, reference, geometry, obstacle helpers
        ├── resources.py    # locators for bundled config / model assets
        ├── logging_utils.py
        ├── config/
        │   └── mpc.yaml     # controller configuration
        └── models/mushr/    # MUSHR MuJoCo model assets
```

## Configuration

The controller is configured by `src/autopilot/config/mpc.yaml`, which is
bundled with the package: vehicle limits, prediction horizon, cost weights,
and obstacle slack. Any `MPC(...)` constructor argument (`horizon_time`, cost
weights, `slack_penalty`, …) overrides the corresponding value in that file.

Demo scenarios are configured separately in `examples/simulation.yaml`.

## How it works

The vehicle is modeled as a kinematic bicycle with state `[x, y, v, heading]`
and control `[acceleration, steering]`. Each control cycle:

1. A reference trajectory is sampled from the path over the prediction
   horizon, expressed in the vehicle's ego frame to keep the QP
   well-conditioned.
2. The nonlinear dynamics are linearized at every step along the current guess
   trajectory, producing a linear time-varying (LTV) model.
3. A convex QP minimizes tracking error and control effort subject to the LTV
   dynamics, actuator limits, rate limits, and soft obstacle half-plane
   constraints.
4. The guess is updated with the new solution and steps 2–3 repeat until the
   trajectory converges (or a max-iteration cap is hit).
5. Only the first control input is applied; the rest seeds the next cycle's
   warm start.

If the solver fails, the controller falls back to an emergency-braking command
rather than emitting an invalid control.

## Development

```bash
pip install -e ".[dev]"

pytest                       # run the test suite
pytest --cov=autopilot       # with coverage
ruff check .                 # lint
black .                      # format
```

Logging is opt-in for applications. The demos call `configure_logging()` at
start-up; library code only uses named loggers and never configures the root
logger. Set the level with the `MPC_LOG_LEVEL` environment variable
(e.g. `MPC_LOG_LEVEL=DEBUG`).

CI (GitHub Actions, see `.github/workflows/ci.yml`) runs linting, format
checks, the test suite on Python 3.11 and 3.12, and a wheel build on every
push and pull request.

See [CONTRIBUTING.md](CONTRIBUTING.md) to get started.

## Acknowledgements

This project draws on ideas and prior work from:

- [Prof. Borrelli — MPC papers and material](https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf)
- [AtsushiSakai — PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/)
- [alexliniger — MPCC](https://github.com/alexliniger/MPCC) and the accompanying [paper](https://onlinelibrary.wiley.com/doi/abs/10.1002/oca.2123)
- [arex18 — rocket-lander](https://github.com/arex18/rocket-lander)
- [prl-mushr — MUSHR](https://github.com/prl-mushr/mushr_mujoco_ros) for the vehicle model

## License

Released under the MIT License. See [LICENSE](LICENSE).
