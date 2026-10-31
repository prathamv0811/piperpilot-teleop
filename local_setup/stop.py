"""Gracefully end the local controller, including one started in another terminal."""
import fcntl
import os
from pathlib import Path
import re
import signal

from piper_teleop.config import load_config

root = Path(__file__).resolve().parents[1]
cfg = load_config(str(root / 'left.local.yaml'))
channel = re.sub(r'[^a-zA-Z0-9_-]', '_', cfg.arm.channel)
path = Path('/tmp') / f'piperpilot-{os.getuid()}-{channel}.lock'
if not path.exists():
    print('No active PiperPilot controller.')
    raise SystemExit(0)
with path.open('r') as lock:
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        value = lock.read().strip()
        if not value.isdecimal():
            raise SystemExit('This controller predates PID tracking; quit it in its terminal.')
        pid = int(value)
        proc = Path('/proc') / str(pid)
        cmd = proc.joinpath('cmdline').read_bytes().split(b'\0')
        if (proc.joinpath('cwd').resolve() != root
                or not any(x.startswith(b'piper_teleop.apps.') for x in cmd)):
            raise SystemExit('Controller identity could not be confirmed; no signal sent.')
        os.kill(pid, signal.SIGINT)
        print(f'Requested PiperPilot shutdown (PID {pid}). The last position target may finish.')
    else:
        print('No active PiperPilot controller.')
