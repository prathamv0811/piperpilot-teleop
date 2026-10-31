"""Exercise the joint-control boundary without opening CAN or enabling motors."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

from piper_teleop.config import load_config
from piper_teleop.piper_arm import PiperArm, PiperArmJoint
from piper_teleop import xr_math

HOME = np.array([0, 1.1, -1.2, 0, 0.5, 0.0])


@pytest.fixture
def arm(monkeypatch):
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
    from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
    offline = AgxArmFactory.create_arm(create_agx_arm_config(robot='piper'))
    preset = ROBOT_JOINT_LIMIT_PRESET_RAD['piper']

    class Robot:
        q = HOME.copy()
        age = 0
        status = 0

        def __init__(self):
            self.sent = []

        def fk(self, q):
            return offline.fk(q)

        def get_joint_angles(self):
            return SimpleNamespace(msg=self.q.copy(), hz=200,
                                   timestamp=time.time() - self.age)

        def get_flange_pose(self):
            return SimpleNamespace(msg=self.fk(self.q.tolist()))

        def get_arm_status(self):
            return SimpleNamespace(msg=SimpleNamespace(arm_status=self.status))

        def move_j(self, q):
            self.sent.append(np.array(q))

    def initialize(self, cfg):
        self.robot = Robot()
        self.gripper = None
        self.joint_limits = tuple(np.array([preset[f'joint{i}'][k] for i in range(1, 7)])
                                  for k in [0, 1])

    monkeypatch.setattr(PiperArm, '__init__', initialize)
    return PiperArmJoint(load_config())


def test_nearby_target_moves_continuously_within_joint_limits(arm):
    pose = np.array(arm.robot.fk(HOME.tolist()))
    last = HOME.copy()
    for i in range(1, 61):
        target = pose.copy()
        target[0] += i * 0.0005
        arm.command_eef(target)
        assert arm.robot.sent
        q = arm.robot.sent[-1]
        assert np.max(np.abs(q - last)) <= arm._joint_step + 1e-10
        assert np.all(q >= arm.joint_limits[0]) and np.all(q <= arm.joint_limits[1])
        arm.robot.q = q.copy()
        last = q.copy()
    assert np.linalg.norm(last - HOME) > 0.03


def test_unreachable_target_sends_no_joint_command(arm):
    arm.command_eef([2, 0, 2, 0, 0, 0])
    assert not arm.robot.sent
    assert 'ik' in arm.limits.snapshot()


@pytest.mark.parametrize('age,status', [(0.5, 0), (0, 4)])
def test_stale_feedback_or_arm_fault_stops_commands(arm, age, status):
    arm.robot.age, arm.robot.status = age, status
    with pytest.raises(RuntimeError, match='feedback'):
        arm.command_eef(arm.robot.fk(HOME.tolist()))
    assert not arm.robot.sent


def test_out_of_range_measured_wrist_requires_homing(arm):
    arm.robot.q = HOME.copy()
    arm.robot.q[4] = np.radians(-89)
    with pytest.raises(RuntimeError, match='home first'):
        arm.command_eef(arm.robot.fk(HOME.tolist()))
    assert not arm.robot.sent


def test_invalid_target_does_not_reach_hardware(arm):
    with pytest.raises(ValueError, match='finite'):
        arm.command_eef([np.nan, 0, 0, 0, 0, 0])
    assert not arm.robot.sent


def test_joint_target_cannot_run_ahead_of_stationary_arm(arm):
    arm._ik.solve = lambda target, previous: (HOME + 0.3, True)
    for _ in range(100):
        arm.command_eef(arm.robot.fk((HOME + 0.3).tolist()))
    assert arm.robot.sent
    assert max(np.max(np.abs(q - HOME)) for q in arm.robot.sent) <= arm._joint_lag
    assert 'tracking' in arm.limits.snapshot()


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_six_axis_paths_keep_joint_steps_bounded_and_follow_target(arm, axis, sign):
    start = np.asarray(arm.robot.fk(HOME.tolist()))
    start_rot = xr_math.rpy_to_matrix(*start[3:])
    previous = HOME.copy()
    for i in range(1, 101):
        target = start.copy()
        if axis < 3:
            target[axis] += sign * i * .0003
        else:
            rotvec = np.zeros(3)
            rotvec[axis - 3] = sign * i * .001
            target[3:] = xr_math.matrix_to_rpy(xr_math.rotvec_to_matrix(rotvec) @ start_rot)
        arm.command_eef(target)
        q = arm.robot.sent[-1]
        assert np.max(np.abs(q - previous)) <= arm._joint_step + 1e-10
        assert np.all(q >= arm.joint_limits[0]) and np.all(q <= arm.joint_limits[1])
        arm.robot.q = previous = q.copy()
    achieved = np.asarray(arm.robot.fk(previous.tolist()))
    assert np.linalg.norm(achieved[:3] - target[:3]) < .002
    dr = xr_math.rpy_to_matrix(*achieved[3:]) @ xr_math.rpy_to_matrix(*target[3:]).T
    assert np.arccos(np.clip((np.trace(dr) - 1) / 2, -1, 1)) < .02


def test_solver_false_success_cannot_send_wrong_orientation(arm):
    target = np.asarray(arm.robot.fk(HOME.tolist()))
    target[3:] = xr_math.matrix_to_rpy(
        xr_math.rotvec_to_matrix(np.array([np.pi, 0, 0])) @ xr_math.rpy_to_matrix(*target[3:]))
    arm._ik.solve = lambda target, previous: (HOME.copy(), True)
    arm.command_eef(target)
    assert not arm.robot.sent


def test_only_one_real_arm_controller_can_hold_channel(monkeypatch):
    from uuid import uuid4
    cfg = load_config(overrides={'arm': {'channel': 'test-' + uuid4().hex}})
    monkeypatch.setattr(PiperArm, '_connect', lambda self, cfg: None)
    first = PiperArm(cfg)
    try:
        with pytest.raises(RuntimeError, match='Another PiperPilot process'):
            PiperArm(cfg)
    finally:
        first.stop()
    replacement = PiperArm(cfg)
    replacement.stop()
