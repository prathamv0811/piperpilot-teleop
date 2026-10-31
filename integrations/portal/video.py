"""Optional video source and existing Quest video-panel adapter."""
import threading
import time
import numpy as np
from piper_teleop.cameras import FakeCamera, Frame


class RemoteCamera:
    """Latest decoded Portal frame for PiperPilot's existing headset panels."""
    def __init__(self, name):
        self.name = name
        self._lock = threading.Lock()
        self._frame = None

    def update(self, video, timestamp):
        rgb = np.frombuffer(bytes(video.data), dtype=np.uint8).reshape(video.height, video.width, 3).copy()
        with self._lock:
            self._frame = Frame(rgb, time.monotonic(), timestamp)

    def latest(self):
        with self._lock:
            if self._frame is None or time.monotonic()-self._frame.mono > .25:
                return None
            return self._frame


class OpenCVCamera:
    def __init__(self, name, path, width=640, height=480, fps=30):
        import cv2
        self.name = name
        self._capture = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f'Cannot open camera {name}: {path}')
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)
        self._lock = threading.Lock()
        self._frame = None
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        import cv2
        count = 0
        while self._running:
            ok, bgr = self._capture.read()
            if not ok:
                time.sleep(.01)
                continue
            count += 1
            with self._lock:
                self._frame = Frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), time.monotonic(), count)

    def latest(self):
        with self._lock:
            return self._frame

    def stop(self):
        self._running = False
        self._capture.release()
        self._thread.join(timeout=2)


def open_cameras(cfg, mappings, *, simulated, stack):
    names = [*cfg.video_tracks, *(s.name for s in cfg.frame_video_tracks)]
    paths = dict(item.split('=', 1) for item in mappings)
    if not simulated and set(paths) != set(names):
        raise ValueError('Supply one --camera NAME=/dev/v4l/by-id/... for every track in the schema')
    cameras = {}
    for name in names:
        camera = FakeCamera(name, 320, 240, cfg.fps) if simulated else OpenCVCamera(name, paths[name], fps=cfg.fps)
        stack.callback(camera.stop)
        cameras[name] = camera
    return cameras
