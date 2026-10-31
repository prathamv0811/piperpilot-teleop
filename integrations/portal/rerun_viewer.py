"""Observe a Portal session in Rerun without claiming control or opening CAN/Quest."""
import argparse
import asyncio
import json
import os
import signal
import threading
import time
from urllib.parse import urlencode

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from livekit.portal import Operator, OperatorConfig
from piper_teleop import xr_math

from _common import ROOT, connection, save_report
from contract import decode_quest


def axes(rec, path, pos, rot, size=.08):
    strips = [[pos, pos+rot[:, i]*size] for i in range(3)]
    rec.log(path, rr.LineStrips3D(strips, colors=[[230,60,60],[60,200,60],[70,110,240]], radii=.003))


async def run(args):
    url, room, token = connection(args.identity, 'viewer', args.env_file)
    cfg = OperatorConfig.from_yaml_file(args.schema, room)
    cfg.set_action_subscription(True)
    op = Operator(cfg)
    latest = {'observation': None, 'action': None}
    lock = threading.Lock()
    counts = {'observations': 0, 'actions': 0, 'video_frames': 0}

    def receive(kind, value):
        with lock:
            latest[kind] = value
            counts['observations' if kind == 'observation' else 'actions'] += 1
            if kind == 'observation':
                counts['video_frames'] += len(value.frames)

    op.on_observation(lambda obs: receive('observation', obs))
    op.on_action(lambda action: receive('action', action))
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    camera_names = [*cfg.video_tracks, *(v.name for v in cfg.frame_video_tracks)]
    camera_views = [rrb.Horizontal(*(rrb.Spatial2DView(origin=f'cameras/{name}', name=f'{name} camera')
                                   for name in camera_names))] if camera_names else []
    blueprint = rrb.Blueprint(rrb.Vertical(
        *camera_views,
        rrb.Horizontal(
            rrb.Spatial3DView(origin='robot/left', name='Left arm — base coordinates'),
            rrb.Spatial3DView(origin='robot/right', name='Right arm — base coordinates'),
            rrb.Spatial3DView(origin='xr', name='Quest controllers')),
        rrb.Horizontal(
            rrb.TimeSeriesView(origin='joints/left', name='Left joints (degrees)'),
            rrb.TimeSeriesView(origin='joints/right', name='Right joints (degrees)'),
            rrb.TimeSeriesView(origin='inputs', name='Grips, triggers and tracking')),
        rrb.TextLogView(origin='status', name='Connection and arm status'),
        row_shares=([3] if camera_views else [])+[3,2,1]))
    rr.init('piperpilot_portal_dual', default_blueprint=blueprint, strict=True)
    rec = rr.get_global_data_recording()
    if rec is None:
        raise RuntimeError('Rerun recording was not initialized')
    # The CLI server is started separately with an explicit loopback binding.
    # The Python serve_* helpers do not expose a bind-address argument.
    uri = f'rerun+http://127.0.0.1:{args.grpc_port}/proxy'
    rec.connect_grpc(uri, default_blueprint=blueprint)
    web_url = f'http://127.0.0.1:{args.web_port}/?{urlencode({"url": uri})}'
    print(f'[rerun] {web_url}', flush=True)
    rec.log('xr', rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
    for side in ('left','right'):
        rec.log(f'robot/{side}', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rec.log(f'robot/{side}/base', rr.Points3D([[0,0,0]], labels=['base'], radii=.015), static=True)
    report = {'running': True, 'pid': os.getpid(), 'viewer_url': web_url, 'control_claimed': False}
    trails = {side: [] for side in ('left','right')}
    last_obs = last_action = None
    last_status = 0.
    started = time.monotonic()
    try:
        await op.connect(url, token)
        if op.local_identity() != args.identity:
            raise ValueError('Viewer token must have its own identity, separate from the controller')
        while not stop.is_set():
            now = time.monotonic()
            rec.set_time('time', duration=now-started)
            with lock:
                obs, action = latest['observation'], latest['action']
            if obs is not None and obs.timestamp_us != last_obs:
                last_obs = obs.timestamp_us
                s = obs.state
                for side in ('left','right'):
                    if s[f'{side}.feedback_valid']:
                        pos = np.array([s[f'{side}.{k}'] for k in ('x','y','z')])
                        rot = xr_math.rpy_to_matrix(*(s[f'{side}.{k}'] for k in ('roll','pitch','yaw')))
                        axes(rec, f'robot/{side}/measured_eef',pos,rot)
                        trail = trails[side]
                        trail.append(pos)
                        del trail[:-300]
                        if len(trail)>1:
                            rec.log(f'robot/{side}/trail',rr.LineStrips3D([trail],colors=[[220,190,60]],radii=.002))
                    for i in range(1,7):
                        rec.log(f'joints/{side}/j{i}',rr.Scalars([np.degrees(s[f'{side}.j{i}'])]))
                    rec.log(f'inputs/{side}/gripper_m',rr.Scalars([s[f'{side}.gripper_m']]))
                for name, frame in obs.frames.items():
                    rgb = np.frombuffer(bytes(frame.data),dtype=np.uint8).reshape(frame.height,frame.width,3)
                    rec.log(f'cameras/{name}',rr.Image(rgb))
            if action is not None and action.timestamp_us != last_action:
                last_action = action.timestamp_us
                try:
                    frame = decode_quest(action.values,received_mono=now,transport_age_s=0)
                except ValueError:
                    frame = None
                if frame:
                    for side in ('left','right'):
                        hand = frame.hand(side)
                        if hand.valid and hand.tracked:
                            axes(rec, f'xr/{side}',hand.pos,xr_math.quat_xyzw_to_matrix(hand.quat_xyzw))
                        else:
                            rec.log(f'xr/{side}',rr.Clear(recursive=True))
                        for name,value in [('grip',hand.squeeze),('trigger',hand.trigger),('tracked',int(hand.tracked))]:
                            rec.log(f'inputs/{side}/{name}',rr.Scalars([value]))
            if now-last_status>=1:
                report.update(counts, active_operator=op.active_operator())
                if obs:
                    report['arms'] = {side:{'status':obs.state[f'{side}.status'],
                        'ready':obs.state[f'{side}.ready'],'engaged':obs.state[f'{side}.engaged']} for side in ('left','right')}
                rec.log('status',rr.TextLog(json.dumps(report.get('arms',{}))+f' operator={op.active_operator()}'))
                rec.flush(timeout_sec=0)
                save_report(args.report,report)
                print(f"[rerun] observations={counts['observations']} actions={counts['actions']} control={op.active_operator()}",flush=True)
                last_status=now
            await asyncio.sleep(1/10)
    finally:
        try:
            await op.disconnect()
        finally:
            op.close()
            report['running']=False
            save_report(args.report,report)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--env-file')
    ap.add_argument('--schema',default=str(ROOT/'portal.yaml'))
    ap.add_argument('--identity',default='piperpilot-viewer')
    ap.add_argument('--web-port',type=int,default=9090)
    ap.add_argument('--grpc-port',type=int,default=9876)
    ap.add_argument('--report',default=str(ROOT/'reports/rerun.json'))
    asyncio.run(run(ap.parse_args()))


if __name__=='__main__':
    main()
