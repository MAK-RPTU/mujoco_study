"""
Experiment 04 — Sensors and the perception sim-to-real gap.

Concept: MuJoCo's <sensor> block gives CLEAN readings. Real IMUs/encoders
have noise, bias, latency, and quantization. Injecting (and randomizing)
those in software is domain randomization — it forces a policy to be robust
to imperfect perception, closing the perception sim-to-real gap.

We read the clean IMU + encoders from sensored_arm.xml while the arm
swings, then produce a "realistic" corrupted copy (bias + Gaussian noise +
latency + quantization) and report how far the corrupted signal departs
from ground truth.

Run:
    python experiments/04_sensors_noise.py
"""
import numpy as np
import mujoco
from _common import load, rollout


def sensor_slice(model, name):
    """Return the (start, dim) into data.sensordata for a named sensor."""
    sid = model.sensor(name).id
    adr = model.sensor_adr[sid]
    dim = model.sensor_dim[sid]
    return adr, dim


def corrupt(signal, rng, bias=0.0, noise_std=0.0, latency_steps=0, quant=None):
    """Apply a fixed bias, Gaussian noise, integer-step latency, and
    quantization to a 1-D-over-time signal array (shape [T] or [T, k])."""
    out = signal.astype(float).copy()
    out = out + bias
    out = out + rng.normal(0.0, noise_std, size=out.shape)
    if latency_steps > 0:
        out = np.roll(out, latency_steps, axis=0)
        out[:latency_steps] = out[latency_steps]  # hold first value
    if quant is not None:
        out = np.round(out / quant) * quant
    return out


if __name__ == "__main__":
    model, data = load("sensored_arm.xml")
    rng = np.random.default_rng(0)

    a_acc, d_acc = sensor_slice(model, "imu_acc")
    a_enc, _ = sensor_slice(model, "enc_elbow")

    # Give it a shove and let it swing freely (motors at zero).
    data.qvel[model.joint("shoulder").dofadr[0]] = 3.0

    rec = {
        "acc": lambda m, d: d.sensordata[a_acc:a_acc + d_acc],
        "enc_elbow": lambda m, d: d.sensordata[a_enc],
    }
    t, logs = rollout(model, data, 3.0, record=rec)

    acc_clean = logs["acc"]                  # [T, 3]
    enc_clean = logs["enc_elbow"]            # [T]

    # Realistic IMU: 0.05 m/s^2 bias, 0.2 noise, 5-step (10 ms) latency.
    acc_dirty = corrupt(acc_clean, rng, bias=0.05, noise_std=0.2,
                        latency_steps=5)
    # Realistic encoder: 12-bit-ish quantization + small noise.
    enc_dirty = corrupt(enc_clean, rng, noise_std=0.002,
                        quant=2 * np.pi / 4096)

    acc_err = np.linalg.norm(acc_dirty - acc_clean, axis=1).mean()
    enc_err = np.abs(enc_dirty - enc_clean).max()

    print("Clean MuJoCo sensors vs software-corrupted ('realistic') copies:")
    print(f"  IMU accel  : mean L2 error  = {acc_err:.4f} m/s^2")
    print(f"  Elbow enc  : max abs error  = {np.degrees(enc_err):.3f} deg")
    print("\nThese corruption params are exactly what you'd RANDOMIZE per "
          "episode (bias, noise_std, latency, quant) so the policy can't "
          "overfit to perfect perception. Tie this to the G1: a noisy/biased "
          "IMU randomized in sim is why the real-robot policy didn't diverge "
          "on first contact with real sensor drift.")
