"""Portal Robot for two PiPER arms. Physical hardware defaults to read-only."""
import argparse
import asyncio
from contextlib import ExitStack
import os
import signal
import time

from livekit.portal import Robot, RobotConfig
from piper_teleop.dual_controller import DualController

from _common import ROOT, connection, save_report
from bridge import RemoteQuestReader
from contract import encode_state
from hardware import HardwarePair, configs_for
from video import open_cameras


async def run(args):
    simulated = not args.hardware
    if args.allow_motion and not args.hardware:
        raise ValueError('--allow-motion is only needed with --hardware')
    motion = simulated or args.allow_motion
    url, room, token = connection(args.identity, 'robot', args.env_file)
    pcfg = RobotConfig.from_yaml_file(args.schema, room)
    configs = configs_for(args.config, simulated=simulated)
    reader = RemoteQuestReader(args.operator_identity, min(c.quest.stale_timeout_s for c in configs.values()))
    portal = Robot(pcfg)
    portal.on_action(reader.accept)
    portal.on_active_operator_changed(reader.set_operator)
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    report = {'running': True, 'pid': os.getpid(), 'simulated': simulated, 'motion_allowed': motion, 'sent_states': 0, 'sent_frames': 0,
              'max_gripper_difference_m': 0.}
    pair = None
    try:
        await portal.connect(url, token)
        if portal.local_identity() != args.identity:
            raise ValueError('Robot token identity does not match --identity')
        with ExitStack() as resources:
            cameras = open_cameras(pcfg, args.camera, simulated=simulated, stack=resources)
            pair = resources.enter_context(HardwarePair(configs, simulated=simulated, allow_motion=motion))
            controller = DualController(reader, pair.arms, configs) if motion else None
            if controller:
                resources.callback(controller.reset, 'session stopped')
            started = time.monotonic()
            next_state, next_status = started, started
            sent_frame_ids = {}
            generation = reader.generation
            speeds = {side: cfg.arm.speed_percent for side, cfg in configs.items()}
            print(f'[robot] {"SIMULATED" if simulated else "PHYSICAL"}; motion={motion}; room={room}; speeds={speeds}', flush=True)
            print('[robot] Waiting for selected operator; release both grips and triggers before engaging.', flush=True)
            while not stop.is_set() and (not args.seconds or time.monotonic()-started < args.seconds):
                tick = time.monotonic()
                reader.set_operator(portal.active_operator())
                if controller:
                    # Prevent an identity/session transition between reset and tick.
                    with reader._lock:
                        if generation != reader.generation:
                            controller.reset('operator session changed; release controls')
                            generation = reader.generation
                        controller.tick()
                if tick >= next_state:
                    state = encode_state(pair.arms, controller, simulated=simulated,
                                         motion_allowed=motion, input_fresh=reader.age() <= reader.timeout_s)
                    report['max_gripper_difference_m'] = max(report['max_gripper_difference_m'],
                        abs(state['left.gripper_m']-state['right.gripper_m']))
                    frames = {name: camera.latest() for name, camera in cameras.items()}
                    # Never stamp an old camera image as current visual feedback.
                    if all(frame is not None and 0 <= tick-frame.mono <= .15 for frame in frames.values()):
                        ts = time.time_ns()//1000
                        reader.sent_observation(ts)
                        for name, frame in frames.items():
                            if sent_frame_ids.get(name) != frame.frame_id:
                                capture_ts = ts-int(max(0., time.monotonic()-frame.mono)*1_000_000)
                                portal.send_video_frame(name, frame.rgb, timestamp_us=capture_ts)
                                sent_frame_ids[name] = frame.frame_id
                                report['sent_frames'] += 1
                        portal.send_state(state, timestamp_us=ts)
                        report['sent_states'] += 1
                    next_state = tick + 1/pcfg.fps
                if tick >= next_status:
                    report.update(accepted_actions=reader.accepted, rejected_actions=reader.rejected,
                                  last_rejection=reader.last_rejection,
                                  controller=controller.status() if controller else None,
                                  state=encode_state(pair.arms, controller, simulated=simulated,
                                                     motion_allowed=motion, input_fresh=reader.age() <= reader.timeout_s))
                    save_report(args.report, report)
                    reason = controller.reason if controller else 'read-only; motors are not commanded'
                    print(f'[robot] actions={reader.accepted} rejected={reader.rejected} {reason}', flush=True)
                    next_status = tick+1
                await asyncio.sleep(max(.001, 1/configs['left'].teleop.rate_hz-(time.monotonic()-tick)))
            report.update(accepted_actions=reader.accepted, rejected_actions=reader.rejected,
                          controller=controller.status() if controller else None,
                          state=encode_state(pair.arms, controller, simulated=simulated,
                                             motion_allowed=motion, input_fresh=reader.age() <= reader.timeout_s))
    except Exception as exc:
        report['terminal_error'] = str(exc)
        if pair is not None:
            report['health_issues'] = {side: list(getattr(arm, 'last_health_issues', []))
                                       for side, arm in pair.arms.items()}
        print(f"[robot] stopped: {exc}; health={report.get('health_issues', {})}", flush=True)
        raise
    finally:
        report['running'] = False
        try:
            save_report(args.report, report)
        finally:
            try:
                await portal.disconnect()
            finally:
                portal.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hardware', action='store_true', help='Read physical CAN arms (otherwise simulate)')
    ap.add_argument('--allow-motion', action='store_true', help='Enable physical remote motion after local preflight')
    ap.add_argument('--config', default=str(ROOT/'robot.example.yaml'))
    ap.add_argument('--schema', default=str(ROOT/'portal.yaml'))
    ap.add_argument('--env-file')
    ap.add_argument('--identity', default='piperpilot-robot')
    ap.add_argument('--operator-identity', default='piperpilot-operator')
    ap.add_argument('--camera', action='append', default=[], metavar='NAME=PATH')
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--report', default=str(ROOT/'reports/robot.json'))
    asyncio.run(run(ap.parse_args()))


if __name__ == '__main__':
    main()
