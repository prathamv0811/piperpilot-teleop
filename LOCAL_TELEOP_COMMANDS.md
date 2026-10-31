# Direct local teleop with top-camera video

For the separate **left-leader/right-follower native CAN setup**, see
[NATIVE_LEADER_FOLLOWER.md](NATIVE_LEADER_FOLLOWER.md). Its speed behavior and
wiring differ from the Quest commands below.

No SFU or LiveKit connection is used. Quest poses go directly over USB/ADB to
the dual-arm controller. The top camera runs in a separate process and streams
directly to the headset and local Rerun viewer.

Copy `dual.example.yaml` to `dual.local.yaml` and verify its physical mapping.
The example uses 50% position-control speeds. Preserve your own measured home
alignment; verify feedback rate on the connected hardware.

## Home

```bash
cd /path/to/piperpilot-teleop
./piperpilot dual-stop
./piperpilot dual-home
```

Homing now confirms fresh CAN joint-position mode feedback before sending the
home target. It stops if that acknowledgement is missing, feedback or motor
health fails, or no joint motion occurs within three seconds. It does not
automatically reset faults. A successful home must be confirmed by fresh measured joint feedback.

## Start teleop

```bash
cd /path/to/piperpilot-teleop
./piperpilot connect
./piperpilot dual-teleop
```

Keep PiperQuestTeleop focused and both controllers visible to the headset for
preflight. Release grips and triggers, then hold each grip to move its arm.
Triggers control grippers; menu pauses both arms. Stop with Ctrl+C or
`./piperpilot dual-stop`.

## Top camera and Rerun

Check for existing processes first. To start them, use separate
terminals. Do not launch a second camera process or a second viewer server.

```bash
cd /path/to/piperpilot-teleop
./piperpilot camera-viewer
```

```bash
cd /path/to/piperpilot-teleop
./piperpilot camera --camera \
  /dev/v4l/by-id/YOUR_UVC_CAMERA-video-index0
```

Open [Rerun](http://127.0.0.1:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy).
Select `piperpilot_direct_local` if an older recording is selected. Camera video
is 640×480, approximately 15 fps to Quest and up to 10 fps in Rerun. Joint plots
read the local controller's status file at 1 Hz; that display rate does not
change the control loop or hardware feedback rate. The camera process never
opens CAN or the Quest pose connection.

Runtime reports: `local_setup/reports/dual-live.json`,
`local_setup/reports/dual-preflight.json`, and
`local_setup/reports/local-camera.json`.
