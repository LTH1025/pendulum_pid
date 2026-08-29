import time
from pathlib import Path

import matplotlib
import mujoco
import mujoco.viewer
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


XML_PATH = Path(__file__).resolve().parent / "asset" / "pendulum.xml"
FIG_DIR = Path(__file__).resolve().parent / "fig"
SIMULATION_TIMESTEP = 0.002
TOTAL_SIMULATION_TIME = 10.0
KP = 10.0
KD = 1.0
POSITION_TARGET = 0.0
VELOCITY_TARGET = 0.0


def apply_pd_control(data: mujoco.MjData) -> None:
    """Compute a PD control law and apply its output as motor torque."""
    # Calculate the position and velocity errors relative to their targets.
    position_error = POSITION_TARGET - data.sensordata[0]
    velocity_error = VELOCITY_TARGET - data.sensordata[1]

    # Apply tau = Kp * position_error + Kd * velocity_error to the motor.
    data.ctrl[0] = KP * position_error + KD * velocity_error


def save_figures(
    times: np.ndarray,
    thetas: np.ndarray,
    theta_dots: np.ndarray,
    controls: np.ndarray,
) -> None:
    """Save state-trajectory and control-input plots."""
    if times.size == 0:
        return

    FIG_DIR.mkdir(parents=True, exist_ok=True)

    state_figure, state_axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    state_axes[0].plot(times, thetas)
    state_axes[0].set_ylabel(r"$\theta$ (rad)")
    state_axes[0].set_title("Pendulum State Trajectories")
    state_axes[0].grid(True)
    state_axes[1].plot(times, theta_dots)
    state_axes[1].set_xlabel("Time (s)")
    state_axes[1].set_ylabel(r"$\dot{\theta}$ (rad/s)")
    state_axes[1].grid(True)
    state_figure.tight_layout()
    state_figure.savefig(FIG_DIR / "state_trajectories.png", dpi=300)
    plt.close(state_figure)

    control_figure, control_axis = plt.subplots(figsize=(8, 4))
    control_axis.plot(times, controls)
    control_axis.set_xlabel("Time (s)")
    control_axis.set_ylabel("Torque (N m)")
    control_axis.set_title("Control Torque")
    control_axis.grid(True)
    control_figure.tight_layout()
    control_figure.savefig(FIG_DIR / "control_input.png", dpi=300)
    plt.close(control_figure)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    # Set how much simulated time each call to mj_step advances.
    model.opt.timestep = SIMULATION_TIMESTEP

    # Set the initial angle, then update derived state without advancing time.
    data.qpos[0] = np.pi / 2
    mujoco.mj_forward(model, data)

    times, thetas, theta_dots, controls = [], [], [], []

    # The passive viewer handles window creation, rendering, and mouse input.
    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.azimuth = 90.0
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -5.0
        viewer.cam.lookat[:] = (0.012768, 0.0, 1.254336)

        # Stop automatically after the prescribed amount of simulated time.
        while viewer.is_running() and data.time < TOTAL_SIMULATION_TIME:
            # Measure wall time so the simulation can be paced in real time.
            step_start = time.time()

            apply_pd_control(data)

            # Record the state and applied control before advancing simulation time.
            times.append(data.time)
            thetas.append(data.qpos[0])
            theta_dots.append(data.qvel[0])
            controls.append(data.ctrl[0])

            # mj_step uses data.ctrl and advances physics by one timestep.
            mujoco.mj_step(model, data)

            # Synchronize the latest physics state and viewer interactions.
            viewer.sync()

            # Sleep for the unused part of the timestep to avoid running too fast.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Save both plots after the time limit closes the viewer.
    save_figures(
        np.asarray(times),
        np.asarray(thetas),
        np.asarray(theta_dots),
        np.asarray(controls),
    )


if __name__ == "__main__":
    main()
