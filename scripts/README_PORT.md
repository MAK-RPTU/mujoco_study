# Viewing the headless sim from Windows — opening the port

The Linux box renders MuJoCo **offscreen** (no monitor) and serves a live
video stream over a TCP port. You have two ways to reach it from Windows.
Pick **Option A** unless you have a reason not to — it needs no firewall
changes and is encrypted.

The server is started with:

```bash
./scripts/start_remote_viewer.sh models/cube_contact.xml 8910
# (binds 0.0.0.0:8910 on the box)
```

---

## Option A — SSH tunnel (recommended, secure, no firewall edits)

This forwards the box's port to your Windows machine over the SSH
connection you already use. Nothing is exposed to the public internet.

On **Windows** (PowerShell, or any terminal with `ssh` — built into
Windows 10/11):

```powershell
ssh -L 8910:localhost:8910 <user>@<box-ip>
```

Keep that SSH session open, then open in your Windows browser:

```
http://localhost:8910
```

`-L 8910:localhost:8910` means: "forward my local port 8910 to
`localhost:8910` as seen from the box." Because the viewer listens on the
box, `localhost` there is the box itself.

To use a different local port (e.g. 8910 is busy on Windows):

```powershell
ssh -L 9000:localhost:8910 <user>@<box-ip>   # then http://localhost:9000
```

---

## Option B — Direct connection (open the port publicly)

Use only if you can't keep an SSH session open. This exposes the stream to
anyone who can reach the box's IP, so prefer A.

1. **Open the port in the cloud firewall / security group.**
   The box is a Shadeform/cloud instance, so the firewall is usually at
   the provider, not just the OS. Add an inbound rule:
   - Protocol: TCP
   - Port: `8910`
   - Source: **your Windows machine's public IP** (find it at
     https://ifconfig.me) — not `0.0.0.0/0`.

2. **Open the OS firewall too (if ufw is active):**
   ```bash
   sudo ufw allow 8910/tcp
   sudo ufw status
   ```

3. **Find the box's public IP:**
   ```bash
   curl -s ifconfig.me; echo
   ```

4. On **Windows**, open:
   ```
   http://<box-ip>:8910
   ```

To lock it back down afterward:
```bash
sudo ufw delete allow 8910/tcp
# and remove the cloud security-group rule
```

---

## Quick checks if it doesn't load

| Symptom | Check |
|---|---|
| Browser hangs / refused (Option A) | Is the SSH tunnel session still open? Is the viewer still running on the box? |
| Refused (Option B) | Cloud security group rule added? `sudo ufw status` shows the port? Right public IP? |
| Page loads but no image | EGL render failing — run `MUJOCO_GL=egl python3 -c "import mujoco; print('ok')"` on the box. |
| Port already in use | Pick another: `./scripts/start_remote_viewer.sh models/cube_contact.xml 8911` |
| Laggy stream | Lower resolution/fps: `python3 viewer/web_viewer.py --model ... --width 480 --height 360 --fps 20` |

---

## Native alternative (no streaming)

If you'd rather run the sim **on Windows itself** with the real
interactive MuJoCo GUI (mouse-orbit, force-drag), skip the port entirely:

```powershell
pip install -r requirements.txt
python viewer/run_local_windows.py --model models/arm_actuators.xml
```
