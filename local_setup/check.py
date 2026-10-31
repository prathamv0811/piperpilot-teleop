"""Read arm feedback and Quest poses; never enables or commands the robot."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from piper_teleop.config import load_config

ROOT = Path(__file__).resolve().parents[1]


def validate_arm_start(cfg, joints, pose, status, *, for_home=False, require_ready=False):
    """Allow only the Cartesian target-limit fault during explicit joint homing."""
    if (np.shape(joints) != (6,) or not np.isfinite(joints).all()
            or np.shape(pose) != (6,) or not np.isfinite(pose).all()):
        raise RuntimeError('Invalid joint or pose feedback')
    folded = bool(joints[1] < 0 or joints[2] > 0)
    if status == 4 and not for_home:
        raise RuntimeError('TARGET_POS_EXCEEDS_LIMIT: run `piperpilot home` before teleop')
    if status != 0 and not (for_home and status == 4):
        raise RuntimeError(f'Arm fault status={status}; homing is blocked')
    if require_ready:
        if len(joints) != 6 or not np.isfinite(joints).all() or not np.isfinite(pose).all():
            raise RuntimeError('Invalid joint or pose feedback')
        if folded:
            raise RuntimeError('Arm is still folded: run `piperpilot home` before teleop')
        if cfg.arm.backend == 'piper_joint':
            from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
            preset = ROBOT_JOINT_LIMIT_PRESET_RAD[cfg.arm.model.lower()]
            margin = np.radians(cfg.joint_position.joint_margin_deg)
            for i, q in enumerate(joints, 1):
                lo, hi = preset[f'joint{i}']
                if not lo + margin <= q <= hi - margin:
                    raise RuntimeError(f'Joint {i} outside working limits: run `piperpilot home`')
        if not all(lo <= v <= hi for lo, v, hi in zip(
                cfg.teleop.limits.workspace_min, pose[:3], cfg.teleop.limits.workspace_max)):
            raise RuntimeError('Arm is outside the teleop workspace: run `piperpilot home`')
    return {'unfold_before_teleop': folded, 'joint_homing_recovery': status == 4 and for_home}


def arm_check(cfg, *, for_home=False, require_ready=False, detect_firmware=False):
    from pyAgxArm import (
        AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config,
        resolve_firmware_profile,
    )
    model = getattr(ArmModel, cfg.arm.model)
    profile = getattr(PiperFW, cfg.arm.firmware)
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=model, firmeware_version=profile,
        interface=cfg.arm.interface, channel=cfg.arm.channel,
        bitrate=cfg.arm.bitrate,
    ))
    robot.connect()
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            joints = robot.get_joint_angles()
            if (joints is not None and joints.hz > 0 and robot.get_fps() > 0
                    and robot.get_flange_pose() is not None
                    and robot.get_arm_status() is not None):
                break
            time.sleep(0.05)
        firmware = robot.get_firmware(timeout=1.0)
        if not firmware:
            raise RuntimeError('No firmware response from arm')
        detected = resolve_firmware_profile(model, firmware['software_version'])
        if detected != profile and not detect_firmware:
            raise RuntimeError(f'Firmware profile mismatch: {detected} != {profile}')
        joints = robot.get_joint_angles()
        pose = robot.get_flange_pose()
        status = robot.get_arm_status()
        if joints is None or joints.hz <= 0 or len(joints.msg) != 6 or pose is None:
            raise RuntimeError('Missing live six-joint or flange feedback')
        if status is None:
            raise RuntimeError('Missing arm status feedback')
        if any(not 0 <= time.time() - msg.timestamp <= 0.25
               for msg in (joints, pose, status)):
            raise RuntimeError('Stale arm feedback')
        readiness = validate_arm_start(
            cfg, joints.msg, pose.msg, int(status.msg.arm_status),
            for_home=for_home, require_ready=require_ready,
        )
        result = {
            'channel': cfg.arm.channel,
            'model': cfg.arm.model,
            'firmware': firmware,
            'sdk_profile': detected,
            'joint_feedback_hz': float(joints.hz),
            'joints_rad': list(joints.msg),
            'flange_pose': list(pose.msg),
            'arm_status': int(status.msg.arm_status),
            'control_mode': int(status.msg.ctrl_mode),
            'error_code': int(status.msg.err_code),
            'motors_currently_enabled': robot.get_joint_enable_status(255),
            'motion_sent': False,
        }
        result.update(readiness)
        return result
    finally:
        robot.disconnect()


def quest_check(cfg, seconds):
    from piper_teleop.quest_client import QuestReader
    reader = QuestReader(cfg.quest.host, cfg.quest.port)
    reader.start()
    samples = []
    last_frame = None
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = reader.get()
            if state is not None and state.frame != last_frame and reader.age() < 0.25:
                last_frame = state.frame
                hand = state.hand(cfg.quest.control_hand)
                samples.append((time.monotonic(), hand.valid, hand.tracked))
            time.sleep(0.005)
        state = reader.get()
        result = {
            'control_hand': cfg.quest.control_hand,
            'hello': reader.hello,
            'fresh_frames_observed': len(samples),
            'stream_hz': reader.rate_hz(),
            'session_state': state.session_state if state else None,
            'valid_frames': sum(s[1] for s in samples),
            'tracked_frames': sum(s[2] for s in samples),
            'space': state.space if state else None,
            'anchor_saved': state.anchor_have if state else False,
            'anchor_tracked': state.anchor_tracked if state else False,
            'pose_stalls': reader.stall_summary(),
            'motion_sent': False,
        }
        if len(samples) < 10:
            raise RuntimeError(f'No steady Quest pose stream: {result}')
        if not result['tracked_frames']:
            raise RuntimeError(f'Wake the left controller and keep it visible to the headset: {result}')
        return result
    finally:
        reader.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arm-only', action='store_true')
    ap.add_argument('--quest-only', action='store_true')
    ap.add_argument('--seconds', type=float, default=5)
    start = ap.add_mutually_exclusive_group()
    start.add_argument('--for-home', action='store_true',
                       help='permit status 4 only for explicit joint-space homing')
    start.add_argument('--require-ready', action='store_true',
                       help='require an unfolded pose inside the teleop workspace')
    args = ap.parse_args()
    cfg = load_config(str(ROOT / 'left.local.yaml'))
    report = {'mode': 'read-only checks; no enable or robot motion commands'}
    checks = []
    if not args.quest_only:
        checks.append(('arm', lambda: arm_check(
            cfg, for_home=args.for_home, require_ready=args.require_ready)))
    if not args.arm_only:
        checks.append(('quest', lambda: quest_check(cfg, args.seconds)))
    failed = False
    for label, fn in checks:
        try:
            report[label] = {'result': 'PASS', **fn()}
        except Exception as exc:
            report[label] = {'result': 'FAIL', 'error': str(exc)}
            failed = True
    reports = ROOT / 'local_setup' / 'reports'
    reports.mkdir(parents=True, exist_ok=True)
    name = 'arm' if args.arm_only else 'quest' if args.quest_only else 'hardware'
    (reports / f'{name}-check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
