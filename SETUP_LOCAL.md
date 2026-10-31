# Fresh-machine setup

See [README.md](README.md#install) for Python dependencies and portable config
creation. The original PiPER/Piper-X application and Quest C++ source live in
`piper_teleop/` and `quest_app/`; local operational helpers live in `local_setup/`.

## Choose hardware and configuration

1. Run `./piperpilot dual-inventory` and identify each adapter by USB serial.
2. For dual Quest operation or initial role assignment, connect each arm to a
   separate CAN bus. Fill `dual.local.yaml` from `dual.example.yaml` and confirm
   the physical sides before setting `mapping_confirmed: true`.
3. For one arm, fill `left.local.yaml` from `left.example.yaml`; choose the actual
   CAN interface, model, firmware and controller hand.
4. Activate each interface at 1 Mbit/s with administrator access. USB reconnection
   can change channel names and leave interfaces down.
5. Keep saved home angles distinct from encoder zero calibration. A right-J4
   home offset only changes homing; native following copies raw joint targets.

Examples target standard PiPER/V183. Check the actual firmware before motion;
use the matching official SDK profile for other firmware/models.

## Quest build and USB setup

The existing scripts support a conda-based Android toolchain. Set `CONDA_BIN`
to your installed conda executable if it is not under `~/miniforge3`:

```bash
bash install/01_android_toolchain.sh
bash install/05_quest_udev.sh
./piperpilot build-apk
./piperpilot connect
```

Building requires JDK 17, Android SDK/NDK and Gradle. The toolchain script
accepts Android SDK licenses; review those terms before running it. Use
`ANDROID_HOME` and `JAVA_HOME` to select an existing installation.
Enable Quest developer mode and authorize USB debugging in the headset.
`connect` starts the app and forwards pose/video ports 8735/8736 over ADB.

## Start a chosen mode

- Native CAN following: [NATIVE_LEADER_FOLLOWER.md](NATIVE_LEADER_FOLLOWER.md).
- Dual Quest and camera: [LOCAL_TELEOP_COMMANDS.md](LOCAL_TELEOP_COMMANDS.md).
- Single Quest: `./piperpilot check`, then an authorized `home` / `teleop`.
- Single SpaceMouse: install HID access with `bash install/04_spacemouse_setup.sh`,
  then run `python -m piper_teleop.apps.collect_data --config left.local.yaml --input spacemouse`.
- No-hardware simulation: `python -m piper_teleop.apps.collect_data --sim`.
- Optional remote/local-SFU mode: [Portal integration](integrations/portal/README.md).

## Operational boundaries

Stop the current PC controller before changing control modes. Native following
must be disconnected before PC control of the follower. Hold/disable commands
on a shared bus may affect both arms; do not use them for isolated wrist work.
Faults, missing fresh feedback, and no-motion homing failures must be diagnosed
rather than hidden by repeated reset/enable commands. The homing helper checks
mode acknowledgement, driver faults, motor enables, and progress.

The camera is independent of CAN and can run while robot operation is blocked.
Use its stable `/dev/v4l/by-id/...-video-index0` path. After unplugging/replugging,
restart a stale capture process rather than launching a competing second one.

Runtime reports are written to ignored `local_setup/reports/`. Inspect their
freshness and the live process/feedback; old successful reports are not current
health checks. The agent skills explain the mode-specific checks and handoffs.
