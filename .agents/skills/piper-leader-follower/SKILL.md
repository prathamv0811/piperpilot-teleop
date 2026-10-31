---
name: piper-leader-follower
description: "Configure or start the official PiPER native CAN leader\u2013follower\
  \ pair, usually left leader and right follower with gripper following. Use for shared-CAN\
  \ setup, role checks, or native tracking problems."
---

# Native PiPER leader–follower

Work from the repository root. Read `NATIVE_LEADER_FOLLOWER.md`; use
`local_setup/leader_follower.py` through `./piperpilot`. Do not replace native
following with a host joint-forwarding loop.

## Establish the current topology

Run `./piperpilot dual-inventory`, inspect `ip -details -statistics link show type can`,
and inspect live controller processes. Resolve serials from `dual.local.yaml`;
never reuse a channel's former physical-side label after USB reconnection.
On a fresh clone create the local file from `dual.example.yaml`, determine the
physical mapping, and select the actual model/firmware. Do not copy placeholder
serials or mark a mapping confirmed without evidence.

For separate buses, use `./piperpilot leader-follower-check`. For a native pair
on one known adapter, use `./piperpilot leader-follower-check --shared`. The
shared check is read-only and does not require both USB adapters.

If a link is down, activate the observed interface at 1 Mbit/s. When `sudo`
requires a password, have the operator run the exact command in their terminal;
do not request their password or repeatedly claim activation succeeded.

## Configure only when needed

Honor an existing verified left-leader/right-follower setup. If roles need
assignment, first establish that each arm is on a separate CAN bus and the
workspace is clear. The user's setup request authorizes role assignment;
resolve missing physical facts, not repeated generic permission.

```bash
./piperpilot leader-follower-configure --separate-buses --workspace-clear
```

The flags assert real conditions. Role writes use broadcast `0x470`; opposite
roles cannot be assigned safely on a shared bus. Do not bypass the helper's
fault/staleness/control-traffic rejection. A sent command is not a persisted
role acknowledgement. Left normal feedback may cease after the role change;
inspect the official startup sequence rather than blindly sending more writes.

With both arms safely supported and powered off, have the operator connect
CAN-H to CAN-H and CAN-L to CAN-L using AgileX's diagram, with each power supply
unchanged. Power the right follower first, left leader second. Keep clear during
startup. Native high-follow ignores the ordinary speed percentage and normally
includes gripper following; clarify only if the user's request conflicts with it.

## Verify the requested result

Read the shared bus. Expect leader IDs `0x151/155/156/157/159` and follower
`0x2A1/2A5/2A6/2A7/2A8`. Inspect fresh follower health, enabled state, six joint
errors and gripper opening/error; `local_setup/reports/leader-follower-check.json`
contains current observations. Standard feedback alone is not native following.

After healthy feedback and an attended clear workspace, ask for a small manual
leader movement and gripper opening change, then verify the matching right-arm
movement/feedback. Do not infer dynamic tracking from one stationary pose or
mark `native_following_verified` true merely because frame IDs exist.

For a wrist mismatch, compare commanded and measured angles first. A saved J4
home pose does not correct native tracking. Do not redefine encoder zero or
calibrate at an arbitrary pose. Individual-joint adjustment requires separating
the follower from the native leader's commands before any PC control.

Report confirmed roles, traffic/health evidence, actual tracking evidence and
remaining physical steps. `dual-stop` only stops PC teleop; it does not stop
firmware-native following. Stop native operation using the manufacturer's
physical procedure, and disconnect the leader before returning to PC control.
