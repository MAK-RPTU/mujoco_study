# mujoco-study

A small, runnable MuJoCo study project. Each of the five core MuJoCo
concepts gets one MJCF model and one experiment script that **breaks and
fixes** something so the concept becomes a sentence you can say truthfully:
MJCF structure, the contact model (`solref`/`solimp`), actuators, sensors,
and integrators/timestep.

It runs two ways:
- **Headless on a remote Linux/GPU box** → stream the live sim to your
  Windows browser over a port (EGL offscreen rendering).
- **Natively on Windows/Mac/Linux with a display** → the real interactive
  MuJoCo GUI viewer.

---

## Layout

```
mujoco_study/
├── README.md
├── requirements.txt
├── .gitignore
├── docs/
│   └── concepts.md            # whiteboard notes: the 5 concepts + sim-to-real lines
├── models/                    # the MJCF (.xml) models
│   ├── cube_contact.xml       # contact model: solref/solimp/condim
│   ├── arm_actuators.xml      # motor / position / velocity / general; armature
│   └── sensored_arm.xml       # IMU, encoders, F/T sensors on sites
├── experiments/               # one script per concept (headless-friendly)
│   ├── _common.py             # shared load/rollout/render helpers (auto-EGL)
│   ├── 01_mjcf_basics.py      # walk the compiled kinematic tree
│   ├── 02_contact_solref_solimp.py  # sweep contact softness, measure sink
│   ├── 03_actuators.py        # compare actuator types' tracking
│   ├── 04_sensors_noise.py    # clean vs noisy/biased/latent sensors
│   └── 05_integrators_timestep.py   # stability vs throughput sweep
├── viewer/
│   ├── web_viewer.py          # headless → MJPEG stream over an HTTP port
│   └── run_local_windows.py   # native interactive viewer (has a display)
└── scripts/
    ├── start_remote_viewer.sh # launch the streaming viewer on the box
    └── README_PORT.md         # opening the port: SSH tunnel vs firewall
```

`out/` (rendered frames/plots) is created on demand and git-ignored.

---

## Install

```bash
pip install -r requirements.txt
```

The `mujoco` wheel bundles EGL, so headless GPU rendering works with no
extra system packages.

---

## Run the experiments (works headless, no display needed)

```bash
cd experiments
python3 01_mjcf_basics.py
python3 02_contact_solref_solimp.py
python3 03_actuators.py
python3 04_sensors_noise.py
python3 05_integrators_timestep.py
```

Each prints a small table/summary and (where useful) saves a frame to
`out/`. Read `docs/concepts.md` alongside them.

---

## Watch it live from Windows (headless box → your browser)

The box has no monitor, so it renders offscreen and streams over a port.

1. **On the box**, start the viewer:
   ```bash
   ./scripts/start_remote_viewer.sh models/cube_contact.xml 8910
   ```

2. **On Windows**, open the port. Recommended (secure, no firewall edits):
   ```powershell
   ssh -L 8910:localhost:8910 <user>@<box-ip>
   ```
   then open **http://localhost:8910** in your browser.

   Full instructions incl. the direct-firewall alternative and
   troubleshooting: [scripts/README_PORT.md](scripts/README_PORT.md).

Swap the model to view any of them, e.g.:
```bash
./scripts/start_remote_viewer.sh models/arm_actuators.xml 8910
```

---

## Run natively on Windows (real interactive GUI)

Once you've cloned/pushed this repo to your Windows machine (which has
MuJoCo + a display):

```powershell
pip install -r requirements.txt
python viewer/run_local_windows.py --model models/arm_actuators.xml
```

Mouse: left-drag orbit, right-drag pan, scroll zoom, space pause,
Ctrl-drag apply force. The experiments run the same on Windows.

---

## Pushing to git later

This folder is already structured as a clean repo root (has `.gitignore`,
`requirements.txt`, `README.md`). When you're ready:

```bash
cd mujoco_study
git init
git add .
git commit -m "MuJoCo study project: 5 concepts + remote/native viewers"
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main
```
