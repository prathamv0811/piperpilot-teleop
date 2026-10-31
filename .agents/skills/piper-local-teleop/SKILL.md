---
name: piper-local-teleop
description: Start, home, diagnose, or stop local PiPER teleoperation with Quest controllers
  or a SpaceMouse. Covers dual-arm routing and single-arm sessions; native shared-CAN
  following uses piper-leader-follower.
---

# Local Quest and SpaceMouse teleop

Read `SETUP_LOCAL.md` and `LOCAL_TELEOP_COMMANDS.md`. Select the requested input
and number of arms from context. Use the current repository's `./piperpilot`
launcher, which selects the Python environment, and explicit local configs.
The generic package defaults target Piper-X; standard PiPER needs its overlay.

## Preflight

Inspect running controllers and CAN interfaces. For dual Quest, each arm needs
its own CAN bus and confirmed adapter serial in `dual.local.yaml`. Resolve
serials on every start. Disconnect any native leader–follower link before PC
motion: stopping a PC process does not stop native commands.

Reuse a healthy existing session rather than starting a second controller.
The dual and per-channel lock files prevent ordinary local process conflicts;
a lock file's existence alone does not prove a live process. Check process
identity before stopping it.

## Dual Quest

```bash
./piperpilot dual-inventory
./piperpilot connect
./piperpilot dual-check --arm-only
./piperpilot dual-check
```

The headset app must be focused and both controllers visible. `connect` starts
the app and forwards pose/video over USB/ADB. No SFU is needed. If the headset is
absent, continue arm-only diagnosis without describing Quest teleop as ready.

When homing is needed and within the requested motion scope, verify the actual
configured targets and clearance, then use `./piperpilot dual-home`. It waits
for CAN joint-mode acknowledgement and rejects stale feedback, driver faults,
disabled motors or no progress. Do not loop reset/home after an unchanged fault.

Start `./piperpilot dual-teleop`. Release both grips/triggers before engaging;
each grip moves its matching arm and each trigger controls its gripper. Verify
fresh live status and correct side routing. If directions are wrong, inspect the
XR anchor/base rotation and mapping, not encoder calibration. `Ctrl+C` or
`./piperpilot dual-stop` ends this PC session; the last position target can finish.

## Single arm / SpaceMouse

Configure `left.local.yaml` from `left.example.yaml`, choosing the actual hand,
channel, firmware and model. Use `./piperpilot check`, an appropriate
`./piperpilot home`, and `./piperpilot teleop`. For SpaceMouse:

```bash
python -m piper_teleop.apps.collect_data --config left.local.yaml --input spacemouse
```

Check HID permissions using the documented setup script if needed. For a
hardware-free run use `--sim`; never pass a simulation result off as real-arm
validation. Recording/replay/inference commands are documented under `docs/`;
do not automatically replay recorded motion to test startup.

## Diagnose and report

Distinguish missing Quest tracking, arm fault, joint working-limit rejection,
CAN-down/USB reconnection, and wrong mapping. Gripper movement does not prove the
six arm joints are healthy. Preserve existing limits and chosen speeds. Saved
home angles are not encoder zero settings. Report which processes are live,
preflight evidence, whether physical motion was verified, and the exact blocker
when administrator access or a physical action is still needed.
