# Operating this repository

This checkout contains real-arm control software. A request to start a selected
mode authorizes the work needed for that mode; do not ask the user to approve
routine read-only checks or repeat an already established preference. Ask only
for missing physical facts, administrator authentication, or a materially
ambiguous motion/calibration request.

## Skills

Read the matching skill before operating hardware:

- Native PiPER pair: [.agents/skills/piper-leader-follower/SKILL.md](.agents/skills/piper-leader-follower/SKILL.md).
- Quest or SpaceMouse: [.agents/skills/piper-local-teleop/SKILL.md](.agents/skills/piper-local-teleop/SKILL.md).
- Camera window: [.agents/skills/piper-camera/SKILL.md](.agents/skills/piper-camera/SKILL.md).
- Optional Portal: [.agents/skills/piper-portal-teleop/SKILL.md](.agents/skills/piper-portal-teleop/SKILL.md).

## Project invariants

- Resolve physical left/right identity from confirmed adapter serials, not
  current `can0`/`can1` numbering. Config templates are not calibrated hardware.
- Native role commands are broadcast. Assign opposite roles only on isolated
  buses. Do not run Quest/SpaceMouse/Portal motion while a native leader drives
  the same follower. A PC process exit does not stop native firmware linkage.
- Read current health and controller ownership before motion. Never clear an
  active collision, alter encoder zeros, or weaken joint limits just to make a
  check pass. A saved home offset is not encoder calibration or a native offset.
- Preserve the user's selected firmware profile, gripper behavior and speed;
  explain when a mode ignores a speed setting. Native high-follow ignores the
  normal speed percentage and includes gripper commands.
- Never claim following from sent commands alone. Verify both live frame sets,
  fault-free follower feedback, and actual joint/gripper tracking during a small
  attended manual movement. Record unresolved checks explicitly.
- Keep changes to robot code separate from running hardware actions. Publishing
  or testing this repository does not authorize starting motors or cloud streams.

## Development

Use `./piperpilot test` for offline tests. The pinned pyAgxArm SDK is required for
kinematics tests; no CAN/motor enable is needed. `local_setup/` is source code and
must be committed. Its `reports/`, datasets, local configs, `.env` files, `.sfu/`,
virtual environments, and recordings are runtime data and must stay ignored.
Retain upstream licensing. Optional Portal source is in `integrations/portal/`;
its native dependency build is separate from ordinary local teleop.
