"""Local UVC camera to Quest and Rerun; no SFU, CAN ownership, or pose connection."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from local_setup.dual import REPORTS, write_report
from piper_teleop.cameras import CameraBase, Frame
from piper_teleop.quest_video import QuestVideoStreamer


class UVCCamera(CameraBase):
    def __init__(self, path):
        self.name = 'top'
        self.capture = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f'Cannot open top camera: {path}')
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.thread = threading.Thread(target=self.grab, daemon=True)
        self.thread.start()

    def grab(self):
        count = 0
        while self.running:
            ok, bgr = self.capture.read()
            if not ok:
                time.sleep(.01)
                continue
            count += 1
            frame = Frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), time.monotonic(), count)
            with self.lock:
                self.frame = frame

    def latest(self):
        with self.lock:
            frame = self.frame
        return frame if frame and time.monotonic() - frame.mono < .25 else None

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        self.capture.release()


def run(args):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    with ExitStack() as stack:
        lock = stack.enter_context(Path(f'/tmp/piperpilot-{os.getuid()}-camera.lock').open('a+'))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        camera = UVCCamera(args.camera)
        stack.callback(camera.stop)
        video = QuestVideoStreamer([camera])
        video.start()
        stack.callback(video.stop)
        blueprint = rrb.Blueprint(rrb.Vertical(
            rrb.Spatial2DView(origin='cameras/top', name='Top camera — direct local stream'),
            rrb.Horizontal(*(rrb.TimeSeriesView(origin=f'joints/{side}', name=f'{side.title()} joints (degrees)')
                             for side in ('left', 'right'))),
            rrb.TextDocumentView(origin='status', name='Local teleop status'),
            row_shares=[6, 2, 1]))
        rr.init('piperpilot_direct_local', default_blueprint=blueprint, strict=True)
        rec = rr.get_global_data_recording()
        rec.connect_grpc('rerun+http://127.0.0.1:9876/proxy', default_blueprint=blueprint)
        started = time.monotonic()
        last_frame = last_report = None
        next_status = started
        report = {'running': True, 'pid': os.getpid(), 'sfu': False,
                  'camera': args.camera, 'rerun_frames': 0}
        try:
            while not stop.is_set():
                now = time.monotonic()
                rec.set_time('time', duration=now-started)
                frame = camera.latest()
                if frame and frame.frame_id != last_frame:
                    last_frame = frame.frame_id
                    rec.log('cameras/top', rr.Image(frame.rgb))
                    report['rerun_frames'] += 1
                if now >= next_status:
                    try:
                        state = json.loads((REPORTS/'dual-live.json').read_text())
                    except (OSError, ValueError):
                        state = {}
                    live = state.get('running', False) and 0 <= time.time()-state.get('timestamp', 0) < 3
                    if live and state['timestamp'] != last_report:
                        last_report = state['timestamp']
                        for side, arm in state.get('arms', {}).items():
                            for i, value in enumerate(arm.get('joints_rad', []), 1):
                                rec.log(f'joints/{side}/j{i}', rr.Scalars([np.degrees(value)]))
                    camera_status = 'live' if frame else 'waiting for fresh camera frames'
                    control_status = state.get('reason', '') if live else 'PC teleop is not running; native CAN following is independent'
                    speed_status = ('Quest arm speeds: 50%. Telemetry display: 1 Hz.' if live else
                                    'Native high-follow does not use the saved 50% speed setting.')
                    rec.log('status', rr.TextDocument(f'Camera: {camera_status}\nArm control: {control_status}\n'
                                                     f'{speed_status}'))
                    report.update(camera_fresh=bool(frame), captured_frames=last_frame,
                                  quest_frames_sent=video.frames_sent, quest_frames_dropped=video.frames_dropped,
                                  teleop_running=live)
                    write_report('local-camera.json', report)
                    rec.flush(timeout_sec=0)
                    next_status = now+1
                stop.wait(.1)
        finally:
            report['running'] = False
            write_report('local-camera.json', report)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--camera', required=True, help='Stable /dev/v4l/by-id/...-video-index0 path')
    run(ap.parse_args())


if __name__ == '__main__':
    main()
