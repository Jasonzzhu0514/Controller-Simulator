# Controller Simulator

NMPC/MPC use model-based receding-horizon optimization; LQR/PID are feedback-control baselines that react to the current state.

This repository contains two dependency-free demos. The web simulator lives in `web/`:

- `nmpc_demo.py`: command-line demo showing the core NMPC loop.
- `web/index.html`: web simulator entry point for comparing NMPC, MPC, LQR, and PID.
- `web/style.css`: web simulator styles.
- `web/simulator.js`: web simulator interaction, simulation, and drawing logic.
- `start.sh`: Linux/macOS launcher.
- `start.bat` / `start.ps1`: Windows launchers.

```text
Controller-Simulator/
├── README.md
├── README.zh-CN.md
├── nmpc_demo.py
├── start.bat
├── start.ps1
├── start.sh
└── web/
    ├── index.html
    ├── simulator.js
    └── style.css
```

## Run the Command-Line Demo

```bash
python3 nmpc_demo.py
```

The command-line demo shows:

1. Read the current state.
2. Enumerate future control sequences.
3. Predict the future with a model.
4. Score each candidate trajectory with a cost function.
5. Apply only the first input from the best sequence.
6. Recompute on the next step.

## Open the Web Simulator

Linux/macOS:

```bash
./start.sh
```

Windows:

```bat
start.bat
```

PowerShell:

```powershell
.\start.ps1
```

The launcher starts a local HTTP server and opens the simulator in your default browser. It starts looking for a free port from `8008`. To choose a different starting port:

```bash
PORT=8010 ./start.sh
```

Windows PowerShell:

```powershell
$env:PORT=8010; .\start.ps1
```

You can also open the printed URL manually, for example:

```text
http://127.0.0.1:8008/index.html
```

## Controllers

The web simulator includes four modes:

- `NMPC`: predicts the future with a nonlinear drag model.
- `MPC`: predicts the future with a linear model.
- `LQR`: linear-quadratic-regulator style state feedback.
- `PID`: standard proportional, integral, and derivative feedback.

The top-right controls include an info dialog and language switcher. The interface defaults to English and can switch to Simplified Chinese, Traditional Chinese, Japanese, and Russian.

All controllers drive the same nonlinear plant:

```text
x_dot = v
v_dot = u - drag * v * |v|
```

This makes the differences visible:

- NMPC uses a prediction model closer to the real plant.
- MPC still predicts the future, but its linear model ignores drag.
- LQR does not predict constraints; it uses position error and velocity feedback.
- PID does not predict ahead; it reacts to current error, accumulated error, and velocity feedback.
- In the speed-limit-zone scenario, NMPC/MPC can account for future speeding in the cost. PID reacts only after the current state changes.
- Near the target, NMPC/MPC use terminal capture and hold logic to avoid oscillating around the target with coarse discrete inputs.

## Adjustable Parameters

Scene parameters:

- Target position
- Real drag coefficient
- Speed-limit-zone start
- Speed limit
- Speed-limit penalty weight

MPC/NMPC parameters:

- Prediction horizon
- Control levels
- Position-error weight
- Terminal-error weight
- Velocity penalty weight
- Control smoothing weight
- Control block count
- Control effort weight
- Terminal velocity multiplier
- Terminal position and velocity boosts
- Settling radius and settling velocity weight
- Target hold thresholds
- Target capture range, gains, and limit

PID parameters:

- Proportional gain `Kp`
- Integral gain `Ki`
- Derivative gain `Kd`
- Integral limit

LQR parameters:

- Position feedback `Kx`
- Velocity feedback `Kv`

Buttons:

- Top-left `Reset`: resets only the car state and curves.
- Right-side `Reset Params`: restores defaults for the current controller.
- Right-side `Reset All`: restores parameters and resets the car state.

Current default presets:

- NMPC: horizon 28, control levels 7, position weight 1.2, terminal weight 13, velocity weight 0.34, smoothing weight 0.34.
- MPC: horizon 32, control levels 7, position weight 0.9, terminal weight 16, velocity weight 0.58, smoothing weight 0.42.

## Display Notes

The page is compressed for desktop viewports and aims to avoid page scrollbars. If the browser window is very short, full-screen mode is recommended.

The `Speed` control supports slow motion and fast-forward: below `1x`, simulation steps are skipped across frames; above `1x`, multiple simulation steps run per frame.

The chart uses adaptive axes:

- The y-axis expands based on recent target, position, velocity, and control values.
- The y-axis shows dynamic numeric ticks.
- The x-axis shows simulation time `t` in seconds.
- History is kept from `t = 0`; the right edge grows with current simulation time. Long histories are sampled for drawing, but the data is not deleted as a sliding window.
- After the system stabilizes near the target, the chart records another 5 seconds of stable tail data, then stops appending points. The control-flow panel also freezes at the last state.

The bottom panel shows a live control-flow visualization:

- NMPC/MPC: read state, generate candidates, ForLoop, rollout, cost, best command, execute first step.
- PID: error, P term, I term, D term, combine, clamp, output.
- LQR: state error, Kx feedback, Kv feedback, combine, clamp, output.
- The blueprint-style SVG uses white links for execution order, green links for state/candidate data, and blue links for the updated plant state `x,v` feeding back into the next frame.

When switching controllers, the control-flow nodes switch automatically. During simulation, the active path is highlighted and key values are shown. After the stable tail freezes, the flow no longer loops.

[中文说明](README.zh-CN.md)
