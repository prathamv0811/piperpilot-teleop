"""Local dual-PiPER lifecycle, serial mapping, and motor-level health checks."""
from contextlib import ExitStack
import fcntl
import os
from pathlib import Path
import time

import numpy as np
from piper_teleop.config import load_config
from piper_teleop.piper_arm import ArmBase, ArmState, FakeArm, PiperArmJoint

SIDES = ('left', 'right')
FAULTS = ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
          'driver_overheating', 'collision_status', 'driver_error_status', 'stall_status')


def inventory():
    devices = {}
    for net in Path('/sys/class/net').iterdir():
        if (net/'type').read_text().strip() != '280':
            continue
        for parent in net.resolve().parents:
            if (parent/'serial').is_file():
                devices[net.name] = {'serial': (parent/'serial').read_text().strip(),
                                     'up': bool(int((net/'flags').read_text(), 16) & 1)}
                break
    return devices


def configs_for(path, *, simulated=False):
    base = load_config(str(path))
    devices = {} if simulated else inventory()
    if not simulated and not base.dual.mapping_confirmed:
        raise ValueError('Confirm left/right adapter serials in the local robot configuration')
    result, seen = {}, set()
    for side in SIDES:
        spec = base.dual.arms.get(side)
        serial = spec.usb_serial
        if not simulated and (not serial or serial in seen):
            raise ValueError('Distinct adapter serials are required')
        seen.add(serial)
        channels = [ch for ch, d in devices.items() if d['serial'] == serial and d['up']]
        if not simulated and len(channels) != 1:
            raise RuntimeError(f'{side}: expected one active adapter for its saved serial')
        result[side] = load_config(str(path), {
            'arm': {'channel': f'fake_{side}' if simulated else channels[0],
                    'backend': 'fake' if simulated else 'piper_joint', 'auto_home': False,
                    'firmware': spec.firmware,
                    'speed_percent': spec.get('speed_percent', base.arm.speed_percent),
                    'home_joints': spec.get('home_joints', base.arm.home_joints)},
            'quest': {'control_hand': side, 'other_hand_gestures': False},
            'teleop': {'yaw_offset_deg': spec.get('yaw_offset_deg', base.teleop.yaw_offset_deg)},
        })
    return result


def driver_problems(robot):
    problems = []
    for i in range(1, 7):
        d = robot.get_driver_states(i)
        if d is None or not 0 <= time.time()-d.timestamp <= .25:
            problems.append(f'J{i}: stale driver feedback')
            continue
        problems += [f'J{i}: {name}' for name in FAULTS if getattr(d.msg.foc_status, name)]
    return problems


class CheckedPiperArm(PiperArmJoint):
    def get_state(self):
        st = super().get_state()
        status = self.robot.get_arm_status()
        problems = driver_problems(self.robot)
        if status is None or not 0 <= time.time()-status.timestamp <= .25:
            problems.append('stale arm-status feedback')
        elif int(status.msg.err_code) != 0:
            problems.append(f'arm error code {int(status.msg.err_code)}')
        enabled = self.robot.get_joints_enable_status_list()
        if len(enabled) != 6:
            problems.append('missing motor-enable feedback')
        else:
            problems += [f'J{i}: motor disabled' for i, value in enumerate(enabled, 1) if not value]
        if not st.joints_valid or not st.eef_valid:
            problems.append('invalid or stale joint/pose feedback')
        if st.arm_status != 0:
            problems.append(f'arm status {st.arm_status}')
        self.last_health_issues = problems
        if problems:
            st.arm_status = -2
            st.joints_valid = st.eef_valid = False
        return st


class ReadOnlyArm(ArmBase):
    def __init__(self, cfg):
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=getattr(ArmModel, cfg.arm.model), firmeware_version=getattr(PiperFW, cfg.arm.firmware),
            interface=cfg.arm.interface, channel=cfg.arm.channel, bitrate=cfg.arm.bitrate))
        self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER) if cfg.gripper.enabled else None
        self.robot.connect()
        self.firmware = None

    def get_state(self):
        st = ArmState(mono=time.monotonic())
        j, s = self.robot.get_joint_angles(), self.robot.get_arm_status()
        if j is not None and len(j.msg) == 6 and np.isfinite(j.msg).all():
            st.joints = np.array(j.msg)
            st.joints_valid = 0 <= time.time()-j.timestamp <= .25
            st.eef_pose6 = np.array(self.robot.fk(j.msg))
            st.eef_valid = bool(st.joints_valid and np.isfinite(st.eef_pose6).all())
        if s is not None and 0 <= time.time()-s.timestamp <= .25:
            st.arm_status = int(s.msg.arm_status)
            if int(s.msg.err_code) != 0 or driver_problems(self.robot):
                st.arm_status = -2
        g = self.gripper.get_gripper_status() if self.gripper else None
        if g is not None and 0 <= time.time()-g.timestamp <= .25:
            st.gripper_width = float(g.msg.value)
            st.gripper_force = float(g.msg.force)
            st.gripper_valid = True
        return st

    def stop(self):
        self.robot.disconnect()


def lock_channel(stack, name):
    handle = stack.enter_context(Path(f'/tmp/piperpilot-{os.getuid()}-{name}.lock').open('a+'))
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError(f'Another controller owns {name}; stop it before starting Portal') from None
    return handle


class HardwarePair:
    def __init__(self, configs, *, simulated=False, allow_motion=False):
        self.configs, self.simulated, self.allow_motion = configs, simulated, allow_motion
        self.stack = ExitStack()
        self.arms = {}

    def __enter__(self):
        try:
            if self.simulated:
                for side, cfg in self.configs.items():
                    arm = FakeArm(cfg)
                    arm.move_joints(cfg.arm.home_joints)
                    self.arms[side] = arm
                    self.stack.callback(arm.stop)
                return self
            lock_channel(self.stack, 'dual')
            # Inspect BOTH arms before either constructor can enable a motor.
            with ExitStack() as preflight:
                readers = {}
                for side, cfg in self.configs.items():
                    lock_channel(preflight, cfg.arm.channel)
                    readers[side] = ReadOnlyArm(cfg)
                    preflight.callback(readers[side].stop)
                time.sleep(.5)
                for side, arm in readers.items():
                    from pyAgxArm import ArmModel, PiperFW, resolve_firmware_profile
                    firmware = arm.robot.get_firmware(timeout=1)
                    cfg = self.configs[side]
                    if not firmware or resolve_firmware_profile(getattr(ArmModel, cfg.arm.model), firmware['software_version']) != getattr(PiperFW, cfg.arm.firmware):
                        raise RuntimeError(f'{side}: firmware does not match the configured profile')
                    st = arm.get_state()
                    if not st.joints_valid or not st.eef_valid:
                        raise RuntimeError(f'{side}: missing fresh arm feedback')
                    if self.allow_motion:
                        if st.arm_status != 0:
                            raise RuntimeError(f'{side}: motor/controller fault; recover locally first')
                        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
                        limits = ROBOT_JOINT_LIMIT_PRESET_RAD[cfg.arm.model.lower()]
                        margin = np.radians(cfg.joint_position.joint_margin_deg)
                        if any(not limits[f'joint{i}'][0]+margin <= q <= limits[f'joint{i}'][1]-margin for i, q in enumerate(st.joints, 1)):
                            raise RuntimeError(f'{side}: home locally before remote control')
                        if not all(lo <= q <= hi for lo, q, hi in zip(cfg.teleop.limits.workspace_min, st.eef_pos, cfg.teleop.limits.workspace_max)):
                            raise RuntimeError(f'{side}: outside the configured workspace; home locally')
            for side, cfg in self.configs.items():
                if self.allow_motion:
                    arm = CheckedPiperArm(cfg)
                else:
                    lock_channel(self.stack, cfg.arm.channel)
                    arm = ReadOnlyArm(cfg)
                self.arms[side] = arm
                self.stack.callback(arm.stop)
            time.sleep(.3)
            return self
        except BaseException:
            self.stack.close()
            raise

    def __exit__(self, typ, value, traceback):
        if typ is not None and self.allow_motion and not issubclass(typ, (KeyboardInterrupt,)):
            for arm in self.arms.values():
                try:
                    arm.emergency_stop()
                except Exception:
                    pass
        self.stack.close()
