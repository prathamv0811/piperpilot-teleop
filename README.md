# PiperPilot Teleop

Native PiPER leader–follower operation, dual-arm Quest teleoperation, single-arm
Quest/SpaceMouse control, and local UVC camera viewing. Includes the Quest
OpenXR app, host controllers, data collection/replay tools, optional Portal
integration, and repository-local skills for coding agents.

Derived from [tomakeIT/PiperPilot](https://github.com/tomakeIT/PiperPilot).
The upstream MIT license and Git history are retained. Robot SDKs remain
external dependencies rather than copied binaries.

## Choose an operating mode

| Mode | Connection | Start here |
| --- | --- | --- |
| Native left leader → right follower, including gripper | Arms share one CAN bus; one USB-CAN adapter for inspection | [Native setup](NATIVE_LEADER_FOLLOWER.md) |
| Dual Quest teleop | One separate USB-CAN bus per arm; Quest over USB/ADB | [Local commands](LOCAL_TELEOP_COMMANDS.md) |
| Single-arm Quest or SpaceMouse | One arm connected to the host | [Setup](SETUP_LOCAL.md) |
| Top UVC camera window | Local Rerun viewer; independent of arm control | [Camera commands](LOCAL_TELEOP_COMMANDS.md#top-camera-and-rerun) |
| Optional Portal teleop | Explicitly selected local/private-network SFU | [Portal integration](integrations/portal/README.md) |

Native following runs in the arms. It is not a PC forwarding loop and does not
need Quest or an SFU. Its high-follow mode does **not** obey the saved position
control speed percentage. Do not run a PC motion controller while the native
leader is connected to the follower.

## Install

Linux, Python 3.10–3.11 (3.11 recommended), and a 1 Mbit/s CAN interface are
required for the hardware workflows. From this repository:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[full,test]'
python -m pip install 'pyagxarm @ git+https://github.com/agilexrobotics/pyAgxArm.git@d90e234ba8e408cfa964b38aa9120b3131fc9f85'
cp -n dual.example.yaml dual.local.yaml
cp -n left.example.yaml left.local.yaml
./piperpilot dual-inventory
```

Edit the local configuration for your actual arm model, firmware, adapter
serials, home pose and workspace. Confirm left/right identity physically before
setting `dual.mapping_confirmed: true`. The examples target **standard PiPER,
firmware S-V1.8-3 through S-V1.8-7 (`V183`)**. The upstream generic defaults
still target Piper-X; always use the appropriate overlay for standard PiPER.
Do not copy another bench's encoder calibration or wrist offset.

The launcher prefers `PIPERPILOT_PYTHON`, an activated virtual environment,
`.venv`, or an existing `piper_teleop` conda environment. Android paths follow
`ANDROID_HOME` and `JAVA_HOME`; no `/home/dev` paths are required.

If a reconnected adapter is down, activate the actual interface reported by
inventory, for example:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

This requires local administrator authentication. A detected USB device is not
proof that the CAN interface is up or that either arm is healthy.

## Native leader–follower

While the arms are on **separate** CAN buses, check health and assign roles:

```bash
./piperpilot leader-follower-check
./piperpilot leader-follower-configure --separate-buses --workspace-clear
```

The two flags confirm physical facts. Role configuration uses a broadcast
message; never send opposite role settings with both arms sharing one bus.
After powering off and wiring as shown in the [official AgileX guide](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/double_piper.MD),
power the right follower first and the left leader second. Then inspect:

```bash
./piperpilot leader-follower-check --shared
```

Verify a small, supervised manual leader movement and matching follower joint
and gripper feedback. Sent role commands and a stationary matching pose do not
prove successful following. The helper reports evidence without declaring
unobserved motion verified. `dual-stop` stops a PC session, not native firmware
following. See the [full procedure](NATIVE_LEADER_FOLLOWER.md).

## Local Quest / SpaceMouse

For two arms on separate buses:

```bash
./piperpilot connect
./piperpilot dual-check --arm-only
./piperpilot dual-check
./piperpilot dual-home
./piperpilot dual-teleop
```

Homing moves hardware. Verify the workspace and configured home targets first.
Release both controllers' grips and triggers before engaging. Hold a grip to
move its arm; triggers control the matching grippers. `Ctrl+C` or
`./piperpilot dual-stop` ends the PC session.

For one arm with `left.local.yaml` configured:

```bash
./piperpilot check
./piperpilot home
./piperpilot teleop
# Alternative input, using the same explicit arm configuration:
python -m piper_teleop.apps.collect_data --config left.local.yaml --input spacemouse
```

Build/install the Quest app with the existing `install/` scripts and
`./piperpilot build-apk`. Details: [setup](SETUP_LOCAL.md),
[Quest](docs/guide/quest.md), [SpaceMouse](docs/guide/spacemouse.md).

## Agent skills

Read [AGENTS.md](AGENTS.md). Skills are included in `.agents/skills/` and can be
invoked by name in an agent that supports repository skills, or read directly:

- `$piper-leader-follower`: configure and verify the native left/right pair.
- `$piper-local-teleop`: dual Quest, single Quest, or SpaceMouse operation.
- `$piper-camera`: restore UVC capture and open the local camera window.
- `$piper-portal-teleop`: optional Portal robot/operator workflow.

Example: “Use $piper-leader-follower to start the left leader and right follower,
including the gripper, then use $piper-camera to open the top camera.”

Skills inspect current processes, wiring, feedback and configuration; they do
not assume a past report still describes the connected hardware.

## Validation and source contents

```bash
./piperpilot test
python -m pip install -e '.[docs]'
python -m mkdocs build --strict
```

CI runs hardware-free tests with the pinned SDK and builds the documentation.
Tests do not establish that a physical pair is aligned or following correctly.
Reports, datasets, camera recordings, `.env` credentials, local YAML settings,
virtual environments, SDK checkouts and built APKs are intentionally excluded
from Git. All application source and build instructions are included. GitHub
Pages deployment is manual and requires separately configuring Pages.
