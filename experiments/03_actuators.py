"""
Experiment 03 — Actuators: motor vs position vs velocity.

Concept: the SAME arm responds very differently depending on the actuator
transmission. `motor` = raw torque (open loop, drifts under gravity).
`position` = PD servo to a target angle (holds against gravity). `velocity`
= velocity servo. `general` is the superset. For a humanoid this choice is
a real sim-to-real lever — idealized position actuators hide torque limits,
bandwidth, and reflected inertia (`armature`).

We command the shoulder to hold ~0 rad and the elbow to track a sine, then
compare tracking error across actuator types.

Run:
    python experiments/03_actuators.py
"""
import numpy as np
from _common import load, rollout


def run(actuator_kind, seconds=4.0):
    """actuator_kind in {'motor','position','velocity'}."""
    model, data = load("arm_actuators.xml")

    aid = {name: model.actuator(name).id for name in [
        "motor_shoulder", "motor_elbow",
        "position_shoulder", "position_elbow",
        "velocity_shoulder", "velocity_elbow",
    ]}
    sh_qpos = model.joint("shoulder").qposadr[0]
    el_qpos = model.joint("elbow").qposadr[0]

    def target_elbow(t):
        return 0.6 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz sine

    def ctrl(model, data, t):
        data.ctrl[:] = 0.0
        tgt_el = target_elbow(t)
        if actuator_kind == "position":
            data.ctrl[aid["position_shoulder"]] = 0.0
            data.ctrl[aid["position_elbow"]] = tgt_el
        elif actuator_kind == "velocity":
            # crude: command velocity proportional to error
            data.ctrl[aid["velocity_shoulder"]] = -2.0 * data.qpos[sh_qpos]
            data.ctrl[aid["velocity_elbow"]] = 4.0 * (tgt_el - data.qpos[el_qpos])
        elif actuator_kind == "motor":
            # naive hand-tuned torque (no feedback on shoulder) — will sag
            data.ctrl[aid["motor_shoulder"]] = 0.0
            data.ctrl[aid["motor_elbow"]] = 8.0 * (tgt_el - data.qpos[el_qpos])

    rec = {
        "elbow": lambda m, d: d.qpos[el_qpos],
        "elbow_target": lambda m, d: target_elbow(d.time),
        "shoulder": lambda m, d: d.qpos[sh_qpos],
    }
    t, logs = rollout(model, data, seconds, ctrl_fn=ctrl, record=rec)
    rmse = float(np.sqrt(np.mean((logs["elbow"] - logs["elbow_target"]) ** 2)))
    shoulder_drift = float(np.abs(logs["shoulder"]).max())
    return rmse, shoulder_drift


if __name__ == "__main__":
    print("Commanding elbow to track a 0.5 Hz sine, shoulder to hold 0.\n")
    print(f"{'actuator':>10} | {'elbow track RMSE (rad)':>22} | "
          f"{'shoulder drift (rad)':>20}")
    print("-" * 60)
    for kind in ("motor", "position", "velocity"):
        rmse, drift = run(kind)
        print(f"{kind:>10} | {rmse:22.4f} | {drift:20.4f}")
    print("\nExpect: position servo holds the shoulder against gravity (low "
          "drift) and tracks tightly; raw motor with no shoulder feedback "
          "sags. Try deleting `armature` in the XML and re-running — the "
          "links will accelerate faster (less reflected inertia).")
