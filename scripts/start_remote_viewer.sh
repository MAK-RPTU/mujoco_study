#!/usr/bin/env bash
# start_remote_viewer.sh — launch the headless web viewer on this Linux box.
#
# Usage:
#   ./scripts/start_remote_viewer.sh models/cube_contact.xml 8910
#
# Then connect from Windows (see scripts/README_PORT.md). The recommended
# path is an SSH tunnel so you do NOT have to open the port publicly:
#   ssh -L 8910:localhost:8910 <user>@<box-ip>
#   # then open http://localhost:8910 in your Windows browser
set -euo pipefail

MODEL="${1:-models/cube_contact.xml}"
PORT="${2:-8910}"

# Resolve repo root (parent of this script's dir) so it works from anywhere.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export MUJOCO_GL="${MUJOCO_GL:-egl}"

echo "[start_remote_viewer] repo:  $ROOT"
echo "[start_remote_viewer] model: $MODEL"
echo "[start_remote_viewer] port:  $PORT  (binding 0.0.0.0)"
echo "[start_remote_viewer] tunnel from Windows:  ssh -L ${PORT}:localhost:${PORT} <user>@<box-ip>"
echo

exec python3 viewer/web_viewer.py --model "$MODEL" --port "$PORT" --host 0.0.0.0
