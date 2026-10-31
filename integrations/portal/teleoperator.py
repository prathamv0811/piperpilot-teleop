"""Portal Operator: local Quest input to a remote dual-PiPER Robot."""
import argparse
import asyncio
import os
import secrets
import signal
import time

import numpy as np
from livekit.portal import Operator, OperatorConfig
from piper_teleop.quest_client import HandState, QuestReader, QuestState

from _common import ROOT, connection, save_report
from contract import encode_quest
from video import RemoteCamera


def synthetic_frame(elapsed):
    # This input is refused whenever the robot declares physical hardware.
    phase = elapsed % 4
    squeeze = 1. if .6 < phase < 3 else 0.
    left = HandState(valid=True, tracked=True, squeeze=squeeze,
                     trigger=.3 if squeeze else 0., pos=np.array([0., .02*np.sin(elapsed), 0.]))
    right = HandState(valid=True, tracked=True, squeeze=squeeze,
                      trigger=.6 if squeeze else 0., pos=np.array([0., 0., -.02*np.cos(elapsed)]))
    return QuestState(session_state='FOCUSED', recv_mono=time.monotonic(), left=left, right=right)


async def run(args):
    if args.synthetic and args.observe:
        raise ValueError('Choose --synthetic or --observe')
    url, room, token = connection(args.identity, 'operator', args.env_file)
    cfg = OperatorConfig.from_yaml_file(args.schema, room)
    op = Operator(cfg)
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    latest = [None, 0.]
    cameras = {name: RemoteCamera(name) for name in [*cfg.video_tracks, *(v.name for v in cfg.frame_video_tracks)]}
    report = {'running': True, 'pid': os.getpid(), 'observations': 0, 'video_frames': 0,
              'sent_actions': 0, 'synthetic_input': args.synthetic}

    def on_observation(obs):
        latest[:] = [obs, time.monotonic()]
        report['observations'] += 1
        report['video_frames'] += len(obs.frames)
        report['state'] = obs.state
        for name, video in obs.frames.items():
            cameras[name].update(video, obs.timestamp_us)

    op.on_observation(on_observation)
    reader = video_streamer = None
    try:
        await op.connect(url, token)
        if op.local_identity() != args.identity:
            raise ValueError('Operator token identity does not match --identity')
        deadline = time.monotonic()+15
        while op.robot_identity() is None and time.monotonic() < deadline:
            await asyncio.sleep(.05)
        if op.robot_identity() != args.robot_identity:
            raise ValueError('Expected robot identity is not present')
        if args.claim_control:
            await op.set_active_operator(op.local_identity())
        if not args.observe and not args.synthetic:
            reader = QuestReader(args.quest_host, args.quest_port)
            reader.start()
        if getattr(args, 'quest_video', False):
            from piper_teleop.quest_video import QuestVideoStreamer
            video_streamer = QuestVideoStreamer(list(cameras.values()), host=args.quest_host,
                                               port=args.quest_video_port)
            video_streamer.start()
        started = time.monotonic()
        last_frame, sequence, epoch = None, 0, secrets.randbits(32)
        next_status = started
        while not stop.is_set() and (not args.seconds or time.monotonic()-started < args.seconds):
            now = time.monotonic()
            obs, received = latest
            if obs is not None and now-received <= .25 and op.active_operator() == args.identity and not args.observe:
                if args.synthetic and not obs.state['simulated']:
                    raise RuntimeError('Synthetic input may only control a simulated robot')
                frame = synthetic_frame(now-started) if args.synthetic else reader.get()
                sample_now = time.monotonic()
                if frame is not None and frame.recv_mono != last_frame and 0 <= sample_now-frame.recv_mono <= .25:
                    values = encode_quest(frame, epoch=epoch, sequence=sequence, now=sample_now)
                    op.send_action(values, in_reply_to_ts_us=obs.timestamp_us)
                    last_frame = frame.recv_mono
                    sequence += 1
                    report['sent_actions'] += 1
            if args.display and obs is not None:
                import cv2
                for name, video in obs.frames.items():
                    rgb = np.frombuffer(bytes(video.data), dtype=np.uint8).reshape(video.height, video.width, 3)
                    cv2.imshow(name, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) == ord('q'):
                    stop.set()
            if now >= next_status:
                if video_streamer:
                    report['quest_video'] = {'frames_sent': video_streamer.frames_sent,
                                             'frames_dropped': video_streamer.frames_dropped}
                if reader:
                    current = reader.get()
                    report['quest'] = {'rate_hz': reader.rate_hz(), 'age_s': reader.age(),
                        'session_state': current.session_state if current else None,
                        'hands': {side: {'valid': bool(current.hand(side).valid),
                                         'tracked': bool(current.hand(side).tracked),
                                         'squeeze': float(current.hand(side).squeeze),
                                         'trigger': float(current.hand(side).trigger)}
                                  for side in ('left', 'right')} if current else {}}
                save_report(args.report, report)
                print(f"[operator] observations={report['observations']} actions={report['sent_actions']} control={op.active_operator()}", flush=True)
                next_status = now+1
            await asyncio.sleep(1/90)
    finally:
        if reader:
            reader.stop()
        if video_streamer:
            video_streamer.stop()
        if args.display:
            import cv2
            cv2.destroyAllWindows()
        if op.active_operator() == args.identity:
            try:
                await op.set_active_operator(None)
            except Exception:
                pass
        await op.disconnect()
        op.close()
        report['running'] = False
        save_report(args.report, report)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--schema', default=str(ROOT/'portal.yaml'))
    ap.add_argument('--env-file')
    ap.add_argument('--identity', default='piperpilot-operator')
    ap.add_argument('--robot-identity', default='piperpilot-robot')
    ap.add_argument('--claim-control', action='store_true')
    ap.add_argument('--observe', action='store_true', help='Receive feedback without opening Quest or sending actions')
    ap.add_argument('--synthetic', action='store_true', help='Exercise both controllers against a SIMULATED robot only')
    ap.add_argument('--quest-host', default='127.0.0.1')
    ap.add_argument('--quest-port', type=int, default=8735)
    ap.add_argument('--quest-video', action='store_true', help='Forward remote camera tracks to Quest panels')
    ap.add_argument('--quest-video-port', type=int, default=8736)
    ap.add_argument('--display', action='store_true', help='Show received camera tracks with OpenCV')
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--report', default=str(ROOT/'reports/operator.json'))
    asyncio.run(run(ap.parse_args()))


if __name__ == '__main__':
    main()
