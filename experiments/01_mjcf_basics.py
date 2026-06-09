"""
Experiment 01 — MJCF structure, from the inside.

Concept: an MJCF is a kinematic tree of <body> elements; each carries
<geom>/<joint>/<site>; <actuator>/<sensor>/<default> are siblings of
<worldbody>. This script loads a model and walks that structure via the
compiled `mjModel`, so you can SEE the tree the XML described.

Run:
    python experiments/01_mjcf_basics.py
"""
from _common import load
import mujoco


def dump(model_filename):
    model, data = load(model_filename)
    print(f"\n=== {model_filename} ===")
    print(f"nq (generalized coords)   : {model.nq}")
    print(f"nv (generalized velocities): {model.nv}")
    print(f"nu (actuators / controls) : {model.nu}")
    print(f"nbody : {model.nbody}  ngeom: {model.ngeom}  "
          f"njnt: {model.njnt}  nsensor: {model.nsensor}")

    print("\nBody tree (name <- parent):")
    for b in range(model.nbody):
        name = model.body(b).name or "(world)"
        parent = model.body(model.body(b).parentid).name or "(world)"
        print(f"  [{b}] {name:14s} <- {parent}")

    print("\nJoints (the DOFs):")
    for j in range(model.njnt):
        jt = mujoco.mjtJoint(model.jnt_type[j]).name
        print(f"  {model.joint(j).name:14s} type={jt}")

    print("\nActuators:")
    for a in range(model.nu):
        print(f"  {model.actuator(a).name}")

    print("\nSensors:")
    for s in range(model.nsensor):
        st = mujoco.mjtSensor(model.sensor_type[s]).name
        print(f"  {model.sensor(s).name:14s} {st}")


if __name__ == "__main__":
    for f in ("cube_contact.xml", "arm_actuators.xml", "sensored_arm.xml"):
        dump(f)
    print("\nWhiteboard takeaway: the XML <body> nesting becomes this tree; "
          "nq != nv whenever a freejoint/ball joint is present "
          "(quaternion has 4 coords but 3 DOFs).")
