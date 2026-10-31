# Native PiPER leader–follower setup

Target: **left arm = leader; right arm = follower**. This uses the arm firmware's
native CAN linkage. The local helper assigns roles and inspects feedback; it
does not forward joint targets through the PC.

## Hardware identity

The existing confirmed USB-serial mapping in `dual.local.yaml` is authoritative;
the helper resolves channel names each time it runs.

| Physical arm | USB-CAN serial | Intended role |
| --- | --- | --- |
| Left | `REPLACE_LEFT_ADAPTER_SERIAL` | Leader |
| Right | `REPLACE_RIGHT_ADAPTER_SERIAL` | Follower |

The standard-PiPER examples use `PiperFW.V183`; verify your actual firmware. No encoder calibration,
joint limits, gripper configuration, or saved home poses are changed by this
helper. Role changes can change the arm's physical behavior.

## Read the current state

```bash
cd /path/to/piperpilot-teleop
./piperpilot leader-follower-check
```

This receives CAN frames and reads SDK feedback without sending arm commands.
It shares the existing dual-session and per-channel locks. The report is saved
to `local_setup/reports/leader-follower-check.json`.

`standard arm feedback` means ordinary feedback is present; it alone does not
prove the arm's persisted role. `leader traffic` means fresh native control
frames are observed. Mixed traffic on a shared bus cannot identify which
physical arm sent each frame.

## Assign roles while the buses are separate

Stop active Quest/PC arm controllers. Keep each arm connected to its own
USB-CAN adapter with **no inter-arm CAN connection** while assigning roles.
The SDK's role setting uses broadcast ID `0x470`; assigning opposite roles on
one shared bus would address both arms.

After physically confirming separation and clearance:

```bash
./piperpilot leader-follower-configure --separate-buses --workspace-clear
```

The flags describe physical facts; they must not be used to bypass an unknown
wiring arrangement or obstruction. The helper checks both arms before writing
either role, refuses faults/stale feedback/existing control traffic, calls
`right.set_follower_mode()` then `left.set_leader_mode()`, and records commands
that were successfully sent. It does not clear errors, home, enable, initialize
grippers, or calibrate joints. A transmitted command is **not** proof that the
firmware accepted or retained the role.

Inspect the resulting isolated traffic before connecting the arms. If a restart
is needed to apply a role, recheck each arm while the buses remain separate.

## Wire and verify native following

Follow [AgileX's official guide and wiring diagram](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/double_piper.MD).
Arrange the CAN wiring with power off: CAN-H to CAN-H, CAN-L to CAN-L, using
the manufacturer's connection arrangement. Power the **right follower first**,
then the **left leader**. Keep the workspace clear during startup.

Native high-follow mode ignores the normal speed percentage, so the saved 50%
setting is not a native following limit. The firmware also sends gripper
commands. The intended pair includes **right gripper following left gripper**. The inspection report compares native gripper command and
feedback openings when both streams are present.

Inspect CAN traffic after startup. Once the separately assigned roles and
physical wiring are established, verify a small manual leader movement and
the corresponding right-arm movement. A single matching stationary pose is
not proof of tracking. The report deliberately leaves
`native_following_verified: false`; final verification requires observed motion
and operator confirmation of the physical left/right arms.

After joining both arms on one USB-CAN adapter, use the shared-bus check:

```bash
./piperpilot leader-follower-check --shared
```

It recognizes either of the two saved USB adapter serials even if Linux renames
the remaining adapter to `can0`. If the adapter was unplugged, Linux may leave
it down after reconnection. For a reconnected `can0`, bring it up with:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

The shared check reads follower health and compares leader/follower joint and
gripper frames on that one bus. It cannot send role settings in shared mode.

If a saved right J4 home angle differs from the left, native linkage still copies raw
joint targets; it does not load PiperPilot's saved home offsets or Quest mapping.

Disconnect the native leader's CAN link before returning the follower to PC
control, as specified by AgileX. Native following runs inside the arms; quitting
the inspection helper or using `dual-stop` does not stop that firmware linkage.

API reference: [pyAgxArm leader–follower functions](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md#leader-follower-arm).
