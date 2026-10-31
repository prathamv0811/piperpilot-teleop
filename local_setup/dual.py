"""Local dual PiPER setup, preflight, homing, and Quest teleoperation."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import signal
import time

import numpy as np

from local_setup.check import arm_check, validate_arm_start
from piper_teleop.config import load_config
from piper_teleop.dual_controller import DualController
from piper_teleop.piper_arm import make_arm
from piper_teleop.quest_client import QuestReader

ROOT = Path(__file__).resolve().parents[1]
LOCK = Path(f'/tmp/piperpilot-{os.getuid()}-dual.lock')
REPORTS = ROOT / 'local_setup/reports'


def can_inventory():
    result = {}
    for path in sorted(Path('/sys/class/net').iterdir()):
        if (path / 'type').read_text().strip() != '280':
            continue
        serial = None
        for parent in path.resolve().parents:
            if (parent / 'serial').is_file():
                serial = (parent / 'serial').read_text().strip()
                break
        result[path.name] = {'usb_serial': serial,
                             'up': bool(int((path / 'flags').read_text(), 16) & 1)}
    return result


def configs_for(path, *, sim=False, require_mapping=False):
    base = load_config(str(path))
    if require_mapping and not base.dual.mapping_confirmed:
        raise RuntimeError('Left/right USB mapping is unconfirmed in dual.local.yaml')
    inventory = can_inventory() if not sim else {}
    configs = {}
    serials = []
    for side in ('left', 'right'):
        spec = base.dual.arms.get(side)
        serial = spec.usb_serial
        serials.append(serial)
        matches = [ch for ch, info in inventory.items() if info['usb_serial'] == serial]
        if not sim and len(matches) != 1:
            raise RuntimeError(f'{side}: expected one adapter with serial {serial}; found {matches}')
        channel = f'fake_{side}' if sim else matches[0]
        if not sim and not inventory[channel]['up']:
            raise RuntimeError(f'{channel} is down; run sudo ip link set {channel} up type can bitrate 1000000')
        configs[side] = load_config(str(path), {
            'arm': {'channel': channel, 'firmware': spec.firmware,
                    'backend': 'fake' if sim else 'piper_joint', 'auto_home': False,
                    'speed_percent': spec.get('speed_percent', base.arm.speed_percent),
                    'home_speed_percent': spec.get('home_speed_percent', 10),
                    'home_joints': spec.get('home_joints', base.arm.home_joints)},
            'quest': {'control_hand': side, 'other_hand_gestures': False},
            'teleop': {'yaw_offset_deg': spec.get('yaw_offset_deg', base.teleop.yaw_offset_deg)},
        })
    if not all(serials) or len(set(serials)) != 2:
        raise RuntimeError('Two distinct USB adapter serials are required')
    return configs


def write_report(name, data):
    REPORTS.mkdir(parents=True, exist_ok=True)
    path = REPORTS / name
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2) + '\n')
    temp.replace(path)


def check_quest(reader, seconds=3):
    frames = set()
    tracked = dict.fromkeys(('left', 'right'), 0)
    end = time.monotonic() + seconds
    last = None
    while time.monotonic() < end:
        state = reader.get()
        if state is not None and reader.age() < 0.25 and state.recv_mono not in frames:
            frames.add(state.recv_mono)
            last = state
            if state.session_state == 'FOCUSED' and not state.calib_step:
                for side in tracked:
                    tracked[side] += int(DualController.hand_usable(state.hand(side)))
        time.sleep(0.01)
    result = {'frames': len(frames), 'tracked_frames': tracked,
              'hz': reader.rate_hz(), 'space': last.space if last else None,
              'focused': last is not None and last.session_state == 'FOCUSED',
              'anchor_tracked': last.anchor_tracked if last else False}
    result['ready'] = bool(
        len(frames) >= 20 and min(tracked.values()) >= 10 and last is not None
        and reader.age() < 0.25 and last.session_state == 'FOCUSED' and not last.calib_step
        and all(DualController.hand_usable(last.hand(side)) for side in tracked)
        and (not last.anchor_have or last.anchor_tracked))
    return result


def acquire_lock(stack):
    handle = stack.enter_context(LOCK.open('a+'))
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('A dual-arm session is already running; use piperpilot dual-stop') from None
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()))
    handle.flush()


def stop_session():
    if not LOCK.exists():
        print('No dual-arm session running.')
        return
    with LOCK.open('r') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            pid = int(handle.read())
            proc = Path('/proc') / str(pid)
            if (proc.joinpath('cwd').resolve() != ROOT
                    or b'local_setup.dual' not in proc.joinpath('cmdline').read_bytes().split(b'\0')):
                raise RuntimeError('Process identity changed; no signal sent')
            os.kill(pid, signal.SIGINT)
            print(f'Requested dual-arm shutdown (PID {pid}).')
        else:
            print('No dual-arm session running.')


def home_feedback(arm, side):
    """Require fresh, healthy feedback before selecting a mode or sending a target."""
    st = arm.get_state()
    joints = arm.robot.get_joint_angles()
    status = arm.robot.get_arm_status()
    now = time.time()
    if (joints is None or status is None
            or any(not 0 <= now - msg.timestamp <= .25 for msg in (joints, status))
            or not st.joints_valid or not np.isfinite(st.joints).all()
            or int(status.msg.arm_status) not in (0, 4)):
        raise RuntimeError(f'{side}: homing lost feedback or faulted ({st.arm_status})')
    if int(status.msg.err_code):
        raise RuntimeError(f'{side}: homing blocked by arm error {status.msg.err_code}')
    for i in range(1, 7):
        driver = arm.robot.get_driver_states(i)
        if driver is None or not 0 <= time.time() - driver.timestamp <= .25:
            raise RuntimeError(f'{side}: homing lost J{i} driver feedback')
        for name in ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                     'driver_overheating', 'collision_status', 'driver_error_status', 'stall_status'):
            if getattr(driver.msg.foc_status, name):
                raise RuntimeError(f'{side}: homing blocked by J{i} {name}')
    if arm.robot.get_joints_enable_status_list() != [True] * 6:
        raise RuntimeError(f'{side}: homing blocked; not all six motors are enabled')
    return st, status


def prepare_joint_home(arm, side):
    """Wait for a fresh CAN/MOVE_J acknowledgement before sending home targets."""
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        home_feedback(arm, side)
        requested = time.time()
        arm.robot.set_motion_mode('j')
        time.sleep(.05)
        _, status = home_feedback(arm, side)
        if (status.timestamp >= requested and int(status.msg.ctrl_mode) == 1
                and int(status.msg.mode_feedback) == 1):
            print(f'[dual] {side}: CAN joint-position mode confirmed.', flush=True)
            return
    raise RuntimeError(f'{side}: CAN joint-position mode not acknowledged; no home target sent')


def home_arm(cfg, side):
    from piper_teleop.config import Cfg
    data = cfg.to_dict()
    speed = cfg.arm.get('home_speed_percent', 10)
    if not isinstance(speed, (int, float)) or not 1 <= speed <= 100:
        raise ValueError('Home speed must be between 1 and 100 percent')
    target = np.asarray(cfg.arm.home_joints, dtype=float)
    if target.shape != (6,) or not np.isfinite(target).all():
        raise ValueError(f'{side}: home target must contain six finite joint angles')
    data['arm'].update(backend='piper', speed_percent=speed)
    with ExitStack() as stack:
        arm = make_arm(Cfg(data))
        stack.callback(arm.stop)
        completed = False

        def stop_incomplete_home():
            if not completed:
                arm.emergency_stop()

        stack.callback(stop_incomplete_home)
        print(f'[dual] Homing {side} on {cfg.arm.channel} at {speed:g}% speed.', flush=True)
        st, _ = home_feedback(arm, side)
        if st.arm_status == 0 and np.max(np.abs(st.joints - target)) < .03:
            validate_arm_start(cfg, st.joints, st.eef_pose6, st.arm_status, require_ready=True)
            print(f'[dual] {side} home reached.', flush=True)
            completed = True
            return
        prepare_joint_home(arm, side)
        initial = arm.get_state().joints.copy()
        started = time.monotonic()
        end = started + 30
        next_status = started
        moved = False
        while time.monotonic() < end:
            st, status = home_feedback(arm, side)
            if int(status.msg.ctrl_mode) != 1 or int(status.msg.mode_feedback) != 1:
                raise RuntimeError(f'{side}: joint-position control mode lost during homing')
            if st.arm_status == 0 and np.max(np.abs(st.joints - target)) < 0.03:
                validate_arm_start(cfg, st.joints, st.eef_pose6, st.arm_status, require_ready=True)
                print(f'[dual] {side} home reached.', flush=True)
                completed = True
                return
            now = time.monotonic()
            moved = moved or np.max(np.abs(st.joints - initial)) > 0.005
            if not moved and now - started > 3:
                raise RuntimeError(f'{side}: no joint movement after 3 s; stopping homing')
            # Repeat the same bounded home target, checking feedback before every send.
            arm.move_joints(target.tolist())
            if now >= next_status:
                print(f'[dual] {side}: max joint error '
                      f'{np.degrees(np.max(np.abs(st.joints - target))):.1f} deg; '
                      f'status={st.arm_status}', flush=True)
                next_status = now + 1
            time.sleep(0.05)
        raise RuntimeError(f'{side}: home target not reached; second arm will not be moved')


def run(args):
    if args.mode == 'stop':
        stop_session()
        return 0
    if args.mode == 'inventory':
        print(json.dumps(can_inventory(), indent=2))
        return 0
    sim = args.mode == 'preview'
    configs = configs_for(args.config, sim=sim, require_mapping=args.mode in ('home', 'teleop'))
    with ExitStack() as stack:
        if args.mode != 'check':
            acquire_lock(stack)
        report = {'timestamp': time.time(), 'motion_sent': False, 'mode': args.mode,
                  'simulated_arms': sim, 'arms': {}}
        if not sim:
            for side, cfg in configs.items():
                try:
                    report['arms'][side] = {'result': 'PASS', **arm_check(
                        cfg, for_home=args.mode == 'home', require_ready=args.mode == 'teleop')}
                except Exception as exc:
                    report['arms'][side] = {'result': 'FAIL', 'error': str(exc)}
        reader = None
        if args.mode != 'home' and not args.arm_only:
            reader = QuestReader(configs['left'].quest.host, configs['left'].quest.port)
            reader.start()
            stack.callback(reader.stop)
            report['quest'] = check_quest(reader)
        write_report('dual-preflight.json', report)
        print(json.dumps(report, indent=2), flush=True)
        failed = any(info['result'] == 'FAIL' for info in report['arms'].values())
        if failed or ('quest' in report and not report['quest']['ready']):
            raise RuntimeError('Preflight incomplete; see local_setup/reports/dual-preflight.json')
        if args.mode == 'check':
            return 0
        if args.mode == 'home':
            for side, cfg in configs.items():
                home_arm(cfg, side)
            return 0
        if reader is None:
            raise RuntimeError('Quest stream is required for teleoperation')
        arms = {}
        for side, cfg in configs.items():
            arms[side] = make_arm(cfg)
            stack.callback(arms[side].stop)
        controller = DualController(reader, arms, configs)
        stack.callback(controller.reset, 'session stopped')
        from piper_teleop.apps.collect_data import RawKeyboard
        print('[dual] LEFT grip/trigger -> LEFT arm/gripper; RIGHT -> RIGHT.\n'
              '[dual] Release grips and triggers to arm input. Menu pauses both.\n'
              '[dual] q/Ctrl+C exits; c calibrates forward using LEFT controller.', flush=True)
        started = time.monotonic()
        next_status = started
        next_tick = started
        period = 1 / configs['left'].teleop.rate_hz
        try:
            with RawKeyboard() as keyboard:
                while not args.seconds or time.monotonic() - started < args.seconds:
                    key = keyboard.poll()
                    if key == 'q':
                        break
                    if key == 'c':
                        if any(c.status.engaged for c in controller.controllers.values()):
                            print('Release both grips before calibration.', flush=True)
                        else:
                            controller.reset('calibrating')
                            reader.send_calibrate_x('left')
                    controller.tick()
                    now = time.monotonic()
                    if now >= next_status:
                        status = {'pid': os.getpid(), 'timestamp': time.time(),
                                  'running': True, 'simulated_arms': sim, **controller.status()}
                        write_report('dual-live.json', status)
                        print('[dual] ' + json.dumps(status), flush=True)
                        next_status = now + 1
                    next_tick = max(next_tick + period, now)
                    time.sleep(max(0, next_tick - time.monotonic()))
        finally:
            controller.reset('session stopped')
            write_report('dual-live.json', {'pid': os.getpid(), 'timestamp': time.time(),
                         'running': False, 'simulated_arms': sim, **controller.status()})
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=('inventory', 'check', 'home', 'preview', 'teleop', 'stop'))
    ap.add_argument('--config', type=Path, default=ROOT / 'dual.local.yaml')
    ap.add_argument('--arm-only', action='store_true', help='check robot links without Quest')
    ap.add_argument('--seconds', type=float, default=0, help='exit preview/teleop after this duration')
    args = ap.parse_args()
    if args.arm_only and args.mode != 'check':
        ap.error('--arm-only is only valid with check')

    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        return run(args)
    except KeyboardInterrupt:
        print('[dual] Stopped. Arms retain their last position targets.')
        return 130
    except Exception as exc:
        print(f'[dual] ERROR: {exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
