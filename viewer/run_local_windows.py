"""
run_local_windows.py — Native interactive MuJoCo viewer.

Use this when you've cloned the repo onto your Windows (or Mac/Linux)
machine that HAS a display. It opens the real MuJoCo GUI window where you
can orbit the camera with the mouse, pause, and scrub — the full
interactive experience the headless box can't give you.

    python viewer/run_local_windows.py --model models/arm_actuators.xml

Controls in the window: left-drag orbit, right-drag pan, scroll zoom,
space to pause, Ctrl-drag to apply forces.

This uses mujoco.viewer.launch_passive so we keep stepping the sim in
Python (you can wire in your own control here).
"""
import argparse
import time

import mujoco
import mujoco.viewer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to an MJCF .xml")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    print("Launching native viewer. Close the window or Ctrl-C to quit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            # Sleep to roughly real time.
            dt = model.opt.timestep - (time.perf_counter() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
