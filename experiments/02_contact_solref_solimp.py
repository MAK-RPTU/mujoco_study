"""
Experiment 02 — Contact softness: solref / solimp (the heart of MuJoCo).

Concept: MuJoCo contacts are SOFT, convex, constraint-based — not rigid
impulses. `solref` (timeconst, dampratio) and `solimp` (dmin,dmax,width,
mid,power) set how stiff/bouncy the contact is. Too soft -> the body
sinks into the floor (wrong dynamics, the #1 sim-to-real gap in
contact-rich tasks). Too stiff -> solver jitter/instability.

This drops the cube under several solref settings and reports how far it
penetrates the floor at rest. You'll watch penetration grow as contacts
soften.

Run:
    python experiments/02_contact_solref_solimp.py
"""
import numpy as np
import mujoco
from _common import load, save_frame

# Floor top is z=0. Cube half-size is 0.1, so a perfectly rigid contact
# rests the cube center at z=0.1. Anything below that is penetration.
REST_Z_RIGID = 0.1


def settle_penetration(solref, solimp=None, seconds=2.0):
    model, data = load("cube_contact.xml")

    # Override solref/solimp on BOTH contacting geoms at runtime so we can
    # sweep without editing the XML. (geom_solref shape: ngeom x 2)
    gid_floor = model.geom("floor").id
    gid_cube = model.geom("cube_geom").id
    for gid in (gid_floor, gid_cube):
        model.geom_solref[gid] = solref
        if solimp is not None:
            model.geom_solimp[gid] = solimp

    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)

    cube_z = data.body("cube").xpos[2]
    penetration = REST_Z_RIGID - cube_z  # >0 means sunk into floor
    return penetration, cube_z


if __name__ == "__main__":
    print("solref = (timeconst, dampratio).  Larger timeconst = softer.\n")
    print(f"{'solref':>16} | {'rest cube_z':>11} | {'penetration (mm)':>16}")
    print("-" * 52)

    sweeps = [
        (0.005, 1.0),   # very stiff
        (0.02, 1.0),    # MuJoCo default-ish
        (0.05, 1.0),    # soft
        (0.2, 1.0),     # very soft -> visible sink
        (0.02, 0.2),    # underdamped -> bouncy (watch it settle)
    ]
    last_model = None
    for tc, dr in sweeps:
        pen, z = settle_penetration([tc, dr])
        flag = "  <- sinks!" if pen > 1e-3 else ""
        print(f"{f'({tc}, {dr})':>16} | {z:11.5f} | {pen*1000:16.2f}{flag}")

    # Save a picture of the softest case so you can eyeball the sink.
    model, data = load("cube_contact.xml")
    for gid in (model.geom("floor").id, model.geom("cube_geom").id):
        model.geom_solref[gid] = [0.2, 1.0]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    out = save_frame(model, data, "02_soft_contact.png")
    print(f"\nSaved a frame of the soft (sinking) case -> {out}")
    print("\nInterview line: 'When I softened solref on the feet, the body "
          "penetrated the floor and the locomotion policy learned wrong "
          "ground-reaction dynamics — that's why solref/solimp calibration "
          "against measured contact stiffness matters for sim-to-real.'")
