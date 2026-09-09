#!/usr/bin/env bash
set -euo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
/usr/bin/python3 - "$BUNDLE" <<'STOPPY'
import os,signal,sys
from pathlib import Path
root=Path(sys.argv[1]); pidfile=root/'live.pid'
if not pidfile.exists():raise SystemExit('Live candidate is not running')
pid=int(pidfile.read_text())
cmd=Path(f'/proc/{pid}/cmdline')
if not cmd.exists():raise SystemExit('Recorded process is no longer running')
if str(root/'run_live.py').encode() not in cmd.read_bytes().split(b'\x00'):
    raise SystemExit('PID does not identify this live candidate; refusing to signal it')
os.kill(pid,signal.SIGINT)
print(f'Sent shutdown request to live candidate PID {pid}')
STOPPY
