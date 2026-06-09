"""Shared helpers for the experiment scripts.

Keeps the headless-rendering boilerplate in one place so each experiment
file stays focused on the ONE concept it demonstrates.
"""
import os

# MuJoCo picks its GL backend at IMPORT time, so this must run before
# `import mujoco` anywhere in the process. On the headless Linux box we
# use EGL (GPU offscreen). On a Windows/Mac laptop with a display you can
# leave MUJOCO_GL unset (it'll use the native GL) — so we only force EGL
# when there's no display.
if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

from pathlib import Path
import numpy as np
import mujoco

MODELS_DIR = Path(__file__).resolve().parent.parent / "models"
OUT_DIR = Path(__file__).resolve().parent.parent / "out"
OUT_DIR.mkdir(exist_ok=True)


def load(model_filename):
    """Load an MJCF model from the models/ dir."""
    path = MODELS_DIR / model_filename
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    return model, data


def rollout(model, data, seconds, ctrl_fn=None, record=None):
    """Step the sim for `seconds`. Optionally apply control and record signals.

    ctrl_fn(model, data, t) -> writes into data.ctrl each step.
    record: dict[name] -> callable(model, data) -> scalar/array, sampled each step.
    Returns (times, {name: np.array}).
    """
    n = int(seconds / model.opt.timestep)
    times = np.zeros(n)
    logs = {k: [] for k in (record or {})}
    for i in range(n):
        t = data.time
        if ctrl_fn is not None:
            ctrl_fn(model, data, t)
        mujoco.mj_step(model, data)
        times[i] = t
        for k, fn in (record or {}).items():
            logs[k].append(np.asarray(fn(model, data)).copy())
    return times, {k: np.array(v) for k, v in logs.items()}


def save_frame(model, data, filename, width=640, height=480, cam=None):
    """Render one offscreen frame to out/<filename>. Useful for eyeballing
    a result without a live viewer."""
    renderer = mujoco.Renderer(model, height, width)
    mujoco.mj_forward(model, data)
    if cam is not None:
        renderer.update_scene(data, camera=cam)
    else:
        renderer.update_scene(data)
    img = renderer.render()
    try:
        from PIL import Image
        Image.fromarray(img).save(OUT_DIR / filename)
    finally:
        renderer.close()
    return OUT_DIR / filename
