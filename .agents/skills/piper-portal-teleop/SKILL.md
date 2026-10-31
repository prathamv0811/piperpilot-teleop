---
name: piper-portal-teleop
description: Set up or operate the optional PiPER Portal robot/operator integration
  when the user selects an SFU or remote teleop workflow. Do not use for native CAN
  following or direct local Quest operation.
---

# Optional Portal robot/operator

Read `integrations/portal/README.md`. All integration source and tests are in
that directory; the external Portal native dependency is built under ignored
`vendor/portal` at the documented revision. This is optional. Do not start an
SFU just because local CAN or Quest operation is failing.

Confirm the intended endpoint and data path from the request. Local/private
network hosting uses `configure_sfu.py` and `run_sfu.sh`; it does not authorize
cloud telemetry, new public firewall exposure, or sending credentials elsewhere.
Generated `.sfu/` files and `.env` files must remain private and ignored.

Install/build the documented native dependency and run the integration's
hardware-free tests. Prove the selected robot/operator schemas and transport
with simulated robot and synthetic operator before a new hardware deployment.
The same schema must be used on both sides; `portal.video.yaml` adds a camera.

For physical control, disconnect native leader–follower CAN linkage and use
separate verified USB adapters. Populate `robot.local.yaml` from its example.
Inspect health, existing process ownership and actual workspace readiness.
The default hardware robot launcher is state-only. Starting motion requires
`--allow-motion` on the robot and the authorized operator identity/claim on the
operator. A request to start real teleop supplies that intent; do not confuse
observation-only startup or simulated tests with successful physical control.

Use a scoped operator token for a separate host. Do not commit `.env.operator`
or print token contents. Select a workspace camera by stable device path.
Stop other PC controllers before starting Portal motion. Preserve fault/age
checks in `hardware.py`; never automatically clear a collision, widen limits,
or reset the arm to overcome missing feedback.

Verify live state, action freshness, clutch release/engagement, correct left/right
routing and camera delivery separately. Report the actual endpoint used, which
processes are running, and any hardware or transport evidence still missing.
