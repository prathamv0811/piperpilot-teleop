"""Configure AgileX's native CAN leader/follower roles; never relay targets."""
import argparse
from collections import Counter
from contextlib import ExitStack
import fcntl
import json
import math
import os
from pathlib import Path
import selectors
import socket
import struct
import time

from local_setup.dual import ROOT, acquire_lock, can_inventory, configs_for, write_report
from piper_teleop.config import load_config

LEADER_IDS = (0x151, 0x155, 0x156, 0x157, 0x159)
FEEDBACK_IDS = (0x2A1, 0x2A5, 0x2A6, 0x2A7)
FAULTS = ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
          'driver_overheating', 'collision_status', 'driver_error_status', 'stall_status')


def shared_config(path):
    """Read the native pair through one known adapter, independent of its old side."""
    base = load_config(str(path))
    known = {base.dual.arms.get(side).usb_serial for side in ('left', 'right')}
    matches = [(ch, info) for ch, info in can_inventory().items()
               if info['usb_serial'] in known]
    if len(matches) != 1:
        raise RuntimeError('Shared-bus inspection needs exactly one connected, known USB-CAN adapter')
    channel, info = matches[0]
    if not info['up']:
        raise RuntimeError(f'{channel} is down; run sudo ip link set {channel} up type can bitrate 1000000')
    # Native standard status/joint/driver feedback belongs to the follower;
    # leader commands are read independently from their control-frame IDs.
    cfg = load_config(str(path), {'arm': {'channel': channel,
                      'firmware': base.dual.arms.right.firmware}})
    return {'follower': cfg}, {'shared': {'channel': channel, 'usb_serial': info['usb_serial']}}


def capture(channels, seconds):
    """Receive CAN frames only. This function has no transmit path."""
    counts = {ch: Counter() for ch in channels}
    latest = {ch: {} for ch in channels}
    with ExitStack() as stack:
        selector = stack.enter_context(selectors.DefaultSelector())
        for channel in channels:
            bus = stack.enter_context(socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW))
            bus.bind((channel,))
            bus.setblocking(False)
            selector.register(bus, selectors.EVENT_READ, channel)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for key, _ in selector.select(min(.1, max(0, end - time.monotonic()))):
                frame = key.fileobj.recv(16)
                can_id, size, data = struct.unpack('=IB3x8s', frame)
                counts[key.data][can_id] += 1
                latest[key.data][can_id] = (time.time(), data[:size])
    return {ch: summarize(counts[ch], latest[ch]) for ch in channels}


def summarize(counts, latest):
    now = time.time()
    fresh = {i for i, (stamp, _) in latest.items() if 0 <= now - stamp <= .25}
    leader = all(i in fresh for i in LEADER_IDS)
    follower = all(i in fresh for i in FEEDBACK_IDS)
    kind = ('leader and follower traffic' if leader and follower else
            'leader traffic' if leader else 'standard arm feedback' if follower else
            'incomplete traffic' if counts else 'no CAN frames')
    result = {'observed': kind, 'leader_frames_fresh': leader,
              'feedback_frames_fresh': follower,
              'counts': {f'0x{i:03x}': n for i, n in sorted(counts.items())}}
    for label, ids in [('leader_joints_deg', (0x155, 0x156, 0x157)),
                       ('feedback_joints_deg', (0x2A5, 0x2A6, 0x2A7))]:
        if all(i in fresh and len(latest[i][1]) == 8 for i in ids):
            result[label] = [v / 1000 for i in ids for v in struct.unpack('>ii', latest[i][1])]
    if 'leader_joints_deg' in result and 'feedback_joints_deg' in result:
        result['instant_joint_error_deg'] = [b - a for a, b in zip(
            result['leader_joints_deg'], result['feedback_joints_deg'])]
    for label, can_id in [('leader_gripper', 0x159), ('follower_gripper', 0x2A8)]:
        if can_id not in fresh or len(latest[can_id][1]) != 8:
            continue
        value, force, status, extra = struct.unpack('>ihBB', latest[can_id][1])
        # Firmware V183 uses width. Retain mode information so newer angle-mode
        # frames are never mislabeled as millimetres.
        angle_mode = bool(status & 4) if can_id == 0x159 else extra == 1
        grip = {'mode': 'angle' if angle_mode else 'width',
                'status_byte': status, 'force_N': force / 1000}
        grip['angle_deg' if angle_mode else 'width_mm'] = value / 1000
        if can_id == 0x159:
            grip.update(enabled=bool(status & 1), zero_command=extra)
        else:
            grip.update(enabled=bool(status & 64), fault_bits=status & 63)
        result[label] = grip
    if all(result.get(label, {}).get('mode') == 'width'
           for label in ('leader_gripper', 'follower_gripper')):
        result['instant_gripper_error_mm'] = (result['follower_gripper']['width_mm']
                                               - result['leader_gripper']['width_mm'])
    # Traffic on a shared bus does not establish which physical arm sent it,
    # and one matching static pose does not prove that the follower tracks motion.
    return result


def health(robot):
    issues = []
    status = robot.get_arm_status()
    joints = robot.get_joint_angles()
    now = time.time()
    result = {'status': None, 'joints_rad': None, 'issues': issues}
    if status is None or not 0 <= now - status.timestamp <= .25:
        issues.append('missing or stale arm status')
    else:
        result['status'] = {name: int(getattr(status.msg, name)) for name in
                            ('ctrl_mode', 'arm_status', 'mode_feedback', 'err_code')}
        if int(status.msg.arm_status) != 0 or int(status.msg.err_code) != 0:
            issues.append('arm reports a fault or is not idle/normal')
    if (joints is None or not 0 <= now - joints.timestamp <= .25
            or len(joints.msg) != 6 or not all(math.isfinite(v) for v in joints.msg)):
        issues.append('missing, stale, or invalid joint feedback')
    else:
        result['joints_rad'] = list(joints.msg)
    for i in range(1, 7):
        driver = robot.get_driver_states(i)
        if driver is None or not 0 <= time.time() - driver.timestamp <= .25:
            issues.append(f'J{i}: missing or stale driver feedback')
        else:
            issues.extend(f'J{i}: {name}' for name in FAULTS
                          if getattr(driver.msg.foc_status, name))
    return result


def send_roles(robots, *, separate_buses, workspace_clear, traffic, sent):
    """Require physical isolation and healthy arms before either broadcast write."""
    if not separate_buses:
        raise RuntimeError('Roles use broadcast CAN ID 0x470. Keep each arm on its own USB-CAN bus; '
                           'confirm with --separate-buses before assigning opposite roles.')
    if not workspace_clear:
        raise RuntimeError('Confirm both arms have clearance before changing modes: --workspace-clear')
    if any(any(int(key, 16) in LEADER_IDS for key in data['counts']) for data in traffic.values()):
        raise RuntimeError('Control frames are already present; resolve the existing controller/leader first.')
    snapshots = {side: health(robot) for side, robot in robots.items()}
    problems = [f'{side}: {issue}' for side, report in snapshots.items() for issue in report['issues']]
    if problems:
        raise RuntimeError('Role configuration blocked: ' + '; '.join(problems))
    # Writes are intentionally separate, and do not enable, home, reset faults,
    # calibrate encoders, or initialize/configure/command a gripper.
    robots['right'].set_follower_mode()
    sent.append({'side': 'right', 'role': 'follower', 'timestamp': time.time()})
    robots['left'].set_leader_mode()
    sent.append({'side': 'left', 'role': 'leader', 'timestamp': time.time()})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('check', 'configure'))
    parser.add_argument('--config', type=Path, default=ROOT / 'dual.local.yaml')
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--shared', action='store_true',
                        help='Read the connected native pair through one known USB-CAN adapter; check only')
    parser.add_argument('--separate-buses', action='store_true',
                        help='Physical confirmation: the two arms are on separate CAN buses')
    parser.add_argument('--workspace-clear', action='store_true',
                        help='Physical confirmation: arms and cables have clearance')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        parser.error('--seconds must be between 1 and 30')
    if args.shared and args.mode != 'check':
        parser.error('--shared is read-only; opposite roles must be configured on separate buses')
    report = {'timestamp': time.time(), 'mode': args.mode,
              'intended_roles': {'left': 'leader', 'right': 'follower'},
              'role_commands_sent': [], 'native_following_verified': False}
    try:
        if args.shared:
            configs, report['adapters'] = shared_config(args.config)
            report['connection'] = 'shared CAN; standard feedback is from the follower'
        else:
            configs = configs_for(args.config, require_mapping=True)
            report['adapters'] = {side: {'channel': cfg.arm.channel,
                                 'usb_serial': cfg.dual.arms.get(side).usb_serial}
                                  for side, cfg in configs.items()}
        with ExitStack() as stack:
            acquire_lock(stack)
            for cfg in configs.values():
                lock = stack.enter_context(Path(f'/tmp/piperpilot-{os.getuid()}-{cfg.arm.channel}.lock').open('a'))
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            report['traffic_before'] = capture([cfg.arm.channel for cfg in configs.values()], args.seconds)
            from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
            robots = {}
            for side, cfg in configs.items():
                robot = AgxArmFactory.create_arm(create_agx_arm_config(
                    robot=ArmModel.PIPER, firmeware_version=getattr(PiperFW, cfg.arm.firmware),
                    interface='socketcan', channel=cfg.arm.channel, bitrate=1000000))
                stack.callback(robot.disconnect)
                robot.connect()
                robots[side] = robot
            time.sleep(.8)
            report['health_before'] = {side: health(robot) for side, robot in robots.items()}
            if args.mode == 'configure':
                send_roles(robots, separate_buses=args.separate_buses,
                           workspace_clear=args.workspace_clear, traffic=report['traffic_before'],
                           sent=report['role_commands_sent'])
                report['traffic_after'] = capture([cfg.arm.channel for cfg in configs.values()], args.seconds)
                report['next_step'] = ('Check isolated left leader/right feedback traffic, then wire native CAN '
                                       'and power follower first, leader second. Verify tracking with a small '
                                       'manual leader movement. High-follow ignores speed percentage and '
                                       'the native leader also transmits gripper commands.')
    except Exception as exc:
        report['error'] = str(exc)
    write_report(f'leader-follower-{args.mode}.json', report)
    print(json.dumps(report, indent=2))
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())
