---
name: piper-camera
description: Open or restore the PiPER workspace UVC camera in the local Rerun window,
  optionally with existing Quest video streaming. Use for camera startup, stale capture
  after USB reconnect, or local video troubleshooting.
---

# Local UVC camera window

Read the camera section of `LOCAL_TELEOP_COMMANDS.md`. Camera operation is
independent of CAN and can be completed even when robot activation is blocked.

1. Inspect current camera/Rerun processes and `/dev/v4l/by-id`. Select the
   workspace camera by stable identity; do not substitute a laptop-facing webcam.
2. Check live capture counts and freshness in `local_setup/reports/local-camera.json`.
   A running PID alone does not prove capture is live. After USB reconnection,
   a process can still hold a dead device even though the stable symlink now
   points to a new `/dev/videoN`.
3. Reuse healthy processes. If capture is stale, verify the PID belongs to
   `local_setup.camera_stream`, stop it gracefully, confirm exit, and reopen the
   selected device. Do not launch a competing capture process.
4. If no viewer server is running, start `./piperpilot camera-viewer`. In another
   terminal/session start `./piperpilot camera --camera /dev/v4l/by-id/ACTUAL_CAMERA-video-index0`.
5. Open `http://127.0.0.1:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy`.
   Select the latest `piperpilot_direct_local` recording if an older one is shown.
   Verify the image is visibly changing and capture counts increase. Keep the
   window open as the requested output.

The current stream uses 640×480 MJPEG capture and raw RGB to Rerun. Do not
switch to an untested encoded-image path merely to reduce bandwidth. Requested
camera FPS is not measured FPS. Rerun display latency is not CAN command latency.

The camera process never owns CAN or the Quest pose stream. It may feed an
already configured local Quest video port, but no SFU/cloud connection is needed.
Native following ignores the saved PC speed percentage; do not show a live
native session as stopped merely because the PC dual controller is absent.
