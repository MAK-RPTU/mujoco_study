"""
Experiment 05 — timestep, integrators, stability vs throughput.

Concept: `<option timestep>` and integrator choice (Euler, implicit,
implicitfast, RK4) trade fidelity against speed. Smaller timestep =
more stable contacts but slower. Implicit/implicitfast add numerical
damping that stays stable at LARGER timesteps — which is what lets you
crank up throughput for batch runs / parameter sweeps (a benchmarking
design decision: fidelity vs samples-per-second).

We load the arm with very high joint damping and step it with various
(integrator, timestep) combos, measuring (a) did the sim blow up? and
(b) steps/second throughput. Stiff damping is what exposes the integrator
difference: RK4 is fully explicit and diverges; MuJoCo's Euler integrates
damping implicitly and survives.

Run:
    python experiments/05_integrators_timestep.py
"""
import time
import numpy as np
import mujoco
from _common import load

INTEGRATORS = {
    "Euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "RK4": mujoco.mjtIntegrator.mjINT_RK4,
}


def trial(integrator_name, timestep, sim_seconds=2.0):
    # The textbook instability: EXPLICIT integration of large joint damping.
    # Euler treats the damping force using the START-of-step velocity, so a
    # big damping coefficient at a big timestep overshoots and the velocity
    # oscillates with growing amplitude until it explodes. The `implicit`/
    # `implicitfast` integrators solve the damping term IMPLICITLY (using the
    # end-of-step velocity), which is unconditionally stable for damping —
    # this is the whole reason they exist. We crank joint damping way up so
    # the difference is unmistakable.
    model, data = load("arm_actuators.xml")
    model.opt.timestep = timestep
    model.opt.integrator = INTEGRATORS[integrator_name]
    model.dof_damping[:] = 80.0          # huge damping -> stiff in velocity
    data.qvel[:] = 5.0                    # give it energy to dissipate

    n = int(sim_seconds / timestep)
    t0 = time.perf_counter()
    blew_up = False
    for _ in range(n):
        mujoco.mj_step(model, data)
        # Divergence check: positions/velocities going non-finite or huge.
        if not np.all(np.isfinite(data.qpos)) or np.abs(data.qvel).max() > 1e4:
            blew_up = True
            break
    wall = time.perf_counter() - t0
    steps_done = _ if False else (n if not blew_up else 0)
    sps = (n if not blew_up else 0) / wall if wall > 0 else 0
    return blew_up, sps, wall


if __name__ == "__main__":
    print("Dropping the cube; reporting stability + throughput.\n")
    print(f"{'integrator':>13} | {'timestep':>9} | {'stable?':>7} | "
          f"{'steps/sec':>11}")
    print("-" * 50)
    for ts in (0.0005, 0.002, 0.005, 0.01, 0.02):
        for name in ("Euler", "implicitfast", "RK4"):
            blew_up, sps, wall = trial(name, ts)
            stable = "no" if blew_up else "yes"
            print(f"{name:>13} | {ts:9.4f} | {stable:>7} | {sps:11.0f}")
        print()
    print("What you should see with high joint damping: RK4 (FULLY explicit) "
          "diverges as the timestep grows, while Euler and implicitfast stay "
          "stable. Key MuJoCo detail: its `Euler` integrator already treats "
          "joint `damping` IMPLICITLY, so it's robust to stiff damping; "
          "`implicit`/`implicitfast` extend that to more force terms; RK4 has "
          "no such treatment. Throughput (steps/sec) is the lever for big "
          "sweeps — RK4 also costs ~2-3x per step (4 evaluations). Pick the "
          "largest stable timestep for YOUR dynamics; implicitfast is the "
          "usual sweet spot.")
