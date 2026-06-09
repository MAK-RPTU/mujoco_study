# MuJoCo internals — whiteboard notes

Five concepts, each mapped to the experiment that demonstrates it and the
one-sentence sim-to-real connection to say out loud.

---

## 1. MJCF — the model format
- MuJoCo's native XML. Like URDF but richer and scene-level.
- `<worldbody>` holds the kinematic tree of nested `<body>`.
  Each body: `<geom>` (shape + mass via density or `<inertial>`),
  `<joint>` (the DOF to its parent), `<site>` (massless frame for
  sensors/actuators).
- Siblings of `<worldbody>`: `<actuator>`, `<sensor>`, `<contact>`,
  `<default>`.
- `<default>` class inheritance = set params once, reuse → signals real
  authoring experience.
- **URDF vs MJCF:** URDF is a robot-description standard (ROS, one robot,
  no environment, no solver params). MJCF describes the whole scene + the
  MuJoCo physics (contact solver, actuators, sensors). MuJoCo ships a
  URDF importer, but the result needs hand-tuning of contacts, inertias,
  and actuators URDF can't express.
- Demo: `experiments/01_mjcf_basics.py` (walks the compiled tree).
- Detail to drop: `nq != nv` when a freejoint/ball joint exists
  (quaternion = 4 coords, 3 DOFs).

## 2. Contact model — the heart of MuJoCo (deep-probe target)
- **Soft, convex, constraint-based** contacts solved as a convex
  optimization each timestep — *not* rigid impulses like PyBullet/Bullet.
- `solref = (timeconst, dampratio)`: virtual spring-damper that sets how
  the constraint is approached. Smaller timeconst = stiffer.
- `solimp = (dmin, dmax, width, midpoint, power)`: constraint impedance
  as a function of penetration depth — how "hard" contact gets as objects
  penetrate.
- Solvers: **Newton, CG, PGS**. Friction via a **cone/pyramid**
  approximation; `condim` ∈ {1,3,4,6} = frictionless / tangential /
  +torsional / +rolling.
- **Sim-to-real:** softness is the #1 contact-rich sim-to-real gap. Too
  soft → foot sinks into floor, locomotion learns wrong ground reaction.
  Too stiff → solver jitter/instability. Calibrating solref/solimp to
  measured contact stiffness *is* sim-to-real tuning.
- Demo: `experiments/02_contact_solref_solimp.py` (sweep softness, watch
  penetration grow).

## 3. Actuation
- Maps control signal → joint force.
  - `motor`: control = torque directly (open loop).
  - `position`: built-in PD servo, gain `kp`.
  - `velocity`: velocity servo, gain `kv`.
  - `general`: superset; the others are shorthands.
- **Sim-to-real:** real motors have torque limits, gear ratios, bandwidth
  an idealized position actuator ignores. `forcerange`/`ctrlrange` clamp
  output; **`armature`** models reflected rotor inertia — a small attr
  that closes a real gap and few candidates name.
- Demo: `experiments/03_actuators.py` (motor vs position vs velocity
  tracking; delete `armature` and watch links accelerate faster).

## 4. Sensors
- `<sensor>` block: accelerometer, gyro, framepos/framequat,
  jointpos/jointvel, force/torque, touch, rangefinder — usually on a
  `<site>`.
- Default sensors are **clean**. Real ones have noise, bias, latency,
  quantization.
- **Sim-to-real:** injecting + randomizing those (domain randomization)
  closes the perception gap. Tie to G1: a randomized noisy/biased IMU is
  why the real-robot policy didn't diverge on real sensor drift.
- Demo: `experiments/04_sensors_noise.py` (clean vs corrupted readings).

## 5. timestep, integrators, stability
- `<option timestep>` + integrator: **Euler, implicit, implicitfast,
  RK4**.
- Smaller timestep = more stable, slower. Key detail: MuJoCo's **Euler**
  already integrates joint `damping` *implicitly*; `implicit`/
  `implicitfast` extend implicit treatment to more force terms → stable at
  larger timesteps. **RK4** is fully explicit (no implicit damping) and
  diverges first under stiff damping, and costs ~2-3x/step (4 evals).
- **Trade-off:** fidelity vs throughput is a benchmarking design decision
  (batch runs / parameter sweeps). Pick the largest stable timestep for
  *your* dynamics; `implicitfast` is the usual sweet spot.
- Demo: `experiments/05_integrators_timestep.py` (stability + steps/sec
  across combos).

---

## The "I built a thing" sentences (say these truthfully after running)
- "When I softened `solref` on the cube, it penetrated the floor by N mm —
  that's the contact-stiffness sim-to-real gap in miniature."
- "Swapping a `position` actuator for raw `motor` made the arm sag under
  gravity because there's no feedback holding it."
- "Adding IMU bias + latency in software degraded the signal by X — that's
  the domain randomization I'd apply per episode."
- "RK4 diverged under stiff joint damping at large timesteps while Euler
  held — because MuJoCo's Euler integrates damping implicitly. That's why
  I'd pick implicitfast for high-throughput sweeps."
