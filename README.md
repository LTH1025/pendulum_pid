# MuJoCo Pendulum PID-Control Learning

A minimal project for learning feedback control with MuJoCo. The current controller is PD (`Ki = 0`): Python calculates the control torque from the pendulum's position and velocity errors, and a direct-drive motor applies it. The project displays the simulation in MuJoCo's passive viewer and saves state and torque trajectories with Matplotlib.

## Project structure

```text
pendulum_pid/
├── .vscode/
│   └── launch.json             # VS Code debug configuration
├── .gitignore                  # Generated and local-only files
├── asset/
│   └── pendulum.xml            # MuJoCo model and scene
├── fig/
│   ├── control_input.png       # Control-input trajectory
│   └── state_trajectories.png  # Angle and angular-velocity trajectories
├── control_pendulum.py         # Controller, simulation loop, and plotting
├── environment.yml             # Minimal Conda environment
└── README.md
```

## Setup

Create and activate the minimal environment:

```bash
conda env create -f environment.yml
conda activate pendulum-pid
```

## Run

From the project directory:

```bash
python control_pendulum.py
```

The simulation stops automatically after 10 simulated seconds, closes the MuJoCo viewer, and writes both plots to `fig/`. Change `TOTAL_SIMULATION_TIME` in `control_pendulum.py` to use a different duration.

## Debug in VS Code

Select the `pendulum-pid` Python interpreter, add breakpoints, and run the **Debug Pendulum Controller** configuration from the Run and Debug panel.
