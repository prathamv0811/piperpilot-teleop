# Optional Portal teleoperation

This source preserves the earlier Portal Robot/Operator integration. Ordinary
native CAN following, local Quest teleop and the UVC viewer do not use it.
Choose this mode explicitly when a Portal/SFU connection is wanted.

`robot.py` wraps both PiPER arms as one typed Portal robot. `teleoperator.py`
sends Quest poses/buttons and receives state/video. `hardware.py` retains
fault/feedback checks; `bridge.py` routes both hands through PiperPilot's dual
controller. The YAML schemas define joint/gripper and optional camera tracks.

## Install the native dependency

From the repository root, install Rust/Cargo, the native build prerequisites
listed by [LiveKit Portal](https://github.com/livekit/portal), and `uv`. Then:

```bash
mkdir -p vendor
git clone https://github.com/livekit/portal.git vendor/portal
git -C vendor/portal checkout 999c118dd082cac322545518b51d0d5d21f4234c
(cd vendor/portal && bash scripts/build_ffi_python.sh release)
cd integrations/portal
uv sync --extra hardware --extra test --extra visualization
cp -n robot.example.yaml robot.local.yaml
cp -n .env.example .env
```

`vendor/` is ignored. The integration imports PiperPilot from this repository,
not an external bench snapshot. No binaries, credentials or old session reports
are included. The committed Python tests can be run with `uv run pytest -q`.

## Local SFU, when requested

Install the official `livekit-server` binary separately. To generate fresh
loopback-only server credentials and configuration:

```bash
.venv/bin/python configure_sfu.py --bind-address 127.0.0.1
bash run_sfu.sh
```

The generator creates `.sfu/livekit.local.yaml` and `.sfu/.env.sfu` with private
permissions. Use `--env-file .sfu/.env.sfu` on robot/operator commands, or fill
`.env` with your chosen authorized endpoint. Never commit generated credentials.
For a LAN host, provide its intended private bind address explicitly; do not
change firewall rules or send camera/robot telemetry to a cloud endpoint merely
to make an example run.

## Prove transport, then operate

Simulated robot and synthetic operator (two terminals, same schema):

```bash
.venv/bin/python robot.py --env-file .sfu/.env.sfu --schema portal.video.yaml
.venv/bin/python teleoperator.py --env-file .sfu/.env.sfu --schema portal.video.yaml --claim-control --synthetic --display
```

For real arms, populate `robot.local.yaml` with verified serial/model/firmware
settings. Keep each arm on a separate CAN adapter; disconnect any native
leader–follower link. Home using the local helper after healthy preflight.

```bash
# Hardware state only; no motion authority:
bash run_robot.sh --env-file .sfu/.env.sfu
# Explicit motion-capable robot session:
bash run_robot.sh --env-file .sfu/.env.sfu --allow-motion
# Quest operator; requires the chosen room's credentials/token:
bash run_operator.sh --env-file .sfu/.env.sfu
```

Run only one robot session at a time. Release grips/triggers before engagement.
Use `portal.video.yaml` plus `--camera front=/dev/v4l/by-id/ACTUAL_CAMERA` for an
explicitly selected workspace camera. `mint_operator_env.py` creates a scoped
operator token file for a separate host; transfer it privately, never through
Git. `rerun_viewer.py --help` describes the optional observer.
