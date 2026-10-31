"""Bimanual routing and loss-of-input checks without physical hardware."""
from dataclasses import replace
import time

import numpy as np
import pytest
import yaml

from local_setup.dual import configs_for
from piper_teleop.config import load_config
from piper_teleop.dual_controller import DualController
from piper_teleop.piper_arm import ArmState
from piper_teleop.quest_client import HandState, QuestState


class Reader:
    def __init__(self):
        self.state = QuestState(session_state='FOCUSED', recv_mono=time.monotonic(),
                               left=HandState(valid=True, tracked=True),
                               right=HandState(valid=True, tracked=True))

    def get(self):
        return self.state

    def rate_hz(self):
        return 90.0

    def send_haptic(self, *args, **kwargs):
        pass

    def advance(self):
        self.state.recv_mono = time.monotonic()
        self.state.frame += 1


class Arm:
    def __init__(self):
        self.state = ArmState(joints=np.array([0., 1.1, -1.2, 0., .5, 0.]),
                              joints_valid=True, eef_valid=True, arm_status=0,
                              eef_pose6=np.array([.3, 0., .25, 0., 0., 0.]))
        self.commands = []
        self.grips = []
        self.homes = []

    def get_state(self):
        return self.state

    def held_target_pose6(self):
        return None

    def command_eef(self, pose):
        self.commands.append(pose)
        self.state.eef_pose6 = np.array(pose)

    def command_gripper(self, width):
        self.grips.append(width)

    def move_joints(self, joints):
        self.homes.append(joints)


@pytest.fixture
def rig():
    reader = Reader()
    arms = {side: Arm() for side in ('left', 'right')}
    configs = {side: load_config(overrides={
        'quest': {'control_hand': side, 'other_hand_gestures': False},
        'teleop': {'filter': {'min_cutoff': 1000000}},
    }) for side in arms}
    return reader, arms, DualController(reader, arms, configs)


def engage_both(reader, ctl):
    ctl.tick()
    reader.state.left.squeeze = reader.state.right.squeeze = 1.
    reader.advance()
    ctl.tick()
    assert all(c.status.engaged for c in ctl.controllers.values())


def test_hands_route_independently_and_grippers_are_separate(rig):
    reader, arms, ctl = rig
    ctl.tick()
    reader.state.right.squeeze = 1.
    reader.state.right.trigger = .75
    reader.advance()
    ctl.tick()
    reader.state.right.pos = np.array([0., 0., -.02])
    reader.advance()
    ctl.tick()
    assert not arms['left'].commands
    assert arms['right'].commands[-1][0] > .3
    assert arms['right'].grips[-1] == pytest.approx(.07 * .25)
    assert arms['left'].grips[-1] == pytest.approx(.07)
    reader.state.right.squeeze = 0.
    reader.state.left.squeeze = 1.
    reader.advance()
    ctl.tick()
    right_count = len(arms['right'].commands)
    reader.state.left.pos = np.array([0., .02, 0.])
    reader.advance()
    ctl.tick()
    assert arms['left'].commands[-1][2] > .25
    assert len(arms['right'].commands) == right_count


def test_start_requires_release_of_both_inputs_per_hand(rig):
    reader, arms, ctl = rig
    reader.state.left.squeeze = 1.
    reader.state.right.trigger = .8
    ctl.tick()
    assert not any(ctl.ready.values())
    assert not any(a.grips or a.commands for a in arms.values())


@pytest.mark.parametrize('loss', ['stale', 'focus', 'anchor', 'menu', 'calibration'])
def test_global_loss_stops_both_and_held_grips_cannot_resume(rig, loss):
    reader, arms, ctl = rig
    engage_both(reader, ctl)
    if loss == 'stale':
        reader.state.recv_mono -= 1
    elif loss == 'focus':
        reader.state.session_state = 'VISIBLE'
    elif loss == 'anchor':
        reader.state.anchor_have = True
    elif loss == 'menu':
        reader.state.left.menu = True
    else:
        reader.state.calib_step = 1
    ctl.tick()
    assert not any(c.status.engaged for c in ctl.controllers.values())
    assert not any(ctl.ready.values())
    reader.state = replace(reader.state, session_state='FOCUSED', anchor_have=False, calib_step=0)
    reader.state.left.menu = False
    reader.advance()
    ctl.tick()
    assert not any(c.status.engaged for c in ctl.controllers.values())
    reader.state.left.squeeze = reader.state.right.squeeze = 0.
    reader.advance()
    ctl.tick()
    reader.state.left.squeeze = reader.state.right.squeeze = 1.
    reader.advance()
    ctl.tick()
    assert all(c.status.engaged for c in ctl.controllers.values())


def test_untracked_hand_stops_its_gripper_and_requires_release(rig):
    reader, arms, ctl = rig
    engage_both(reader, ctl)
    reader.state.left.tracked = False
    reader.advance()
    count = len(arms['left'].grips)
    ctl.tick()
    assert not ctl.controllers['left'].status.engaged
    assert ctl.controllers['right'].status.engaged
    assert len(arms['left'].grips) == count
    reader.state.left.tracked = True
    reader.advance()
    ctl.tick()
    assert not ctl.controllers['left'].status.engaged


def test_fault_on_second_arm_blocks_first_arm_before_commands(rig):
    reader, arms, ctl = rig
    engage_both(reader, ctl)
    counts = {side: (len(a.commands), len(a.grips)) for side, a in arms.items()}
    arms['right'].state.arm_status = 7
    with pytest.raises(RuntimeError, match='right'):
        ctl.tick()
    assert counts == {side: (len(a.commands), len(a.grips)) for side, a in arms.items()}


def test_opposite_hand_buttons_never_home_either_arm(rig):
    reader, arms, ctl = rig
    reader.state.left.secondary = reader.state.right.secondary = True
    for c in ctl.controllers.values():
        c._home_hold_start = time.monotonic() - 2
    for _ in range(3):
        reader.advance()
        ctl.tick()
    assert not any(a.homes for a in arms.values())


def test_serial_mapping_survives_interface_order_and_requires_confirmation(tmp_path, monkeypatch):
    path = tmp_path / 'dual.yaml'
    data = {'dual': {'mapping_confirmed': False, 'arms': {
        'left': {'usb_serial': 'L', 'firmware': 'V183'},
        'right': {'usb_serial': 'R', 'firmware': 'V183'}}}}
    path.write_text(yaml.safe_dump(data))
    monkeypatch.setattr('local_setup.dual.can_inventory', lambda: {
        'can9': {'usb_serial': 'R', 'up': True}, 'can4': {'usb_serial': 'L', 'up': True}})
    configs = configs_for(path)
    assert configs['left'].arm.channel == 'can4'
    assert configs['right'].arm.channel == 'can9'
    with pytest.raises(RuntimeError, match='unconfirmed'):
        configs_for(path, require_mapping=True)
    data['dual']['mapping_confirmed'] = True
    path.write_text(yaml.safe_dump(data))
    assert configs_for(path, require_mapping=True)['right'].arm.channel == 'can9'


def test_per_arm_home_preserves_right_wrist_alignment_without_changing_left(tmp_path):
    path = tmp_path / 'dual.yaml'
    left_home = [0., 1.1, -1.2, 0., .5, 0.]
    right_home = [0., 1.1, -1.2, .7698298264696589, .5, 0.]
    path.write_text(yaml.safe_dump({
        'arm': {'home_joints': left_home},
        'dual': {'mapping_confirmed': True, 'arms': {
            'left': {'usb_serial': 'L', 'firmware': 'V183'},
            'right': {'usb_serial': 'R', 'firmware': 'V183', 'home_joints': right_home},
        }},
    }))
    configs = configs_for(path, sim=True)
    assert configs['left'].arm.home_joints == left_home
    assert configs['right'].arm.home_joints == right_home
    assert configs['left'].arm.auto_home is False
    assert configs['right'].arm.auto_home is False


def home_robot(arm, *, stale=False, acknowledge_after=1):
    from types import SimpleNamespace

    foc = SimpleNamespace(**dict.fromkeys((
        'voltage_too_low', 'motor_overheating', 'driver_overcurrent',
        'driver_overheating', 'collision_status', 'driver_error_status', 'stall_status'), False))
    status = SimpleNamespace(arm_status=0, err_code=0, ctrl_mode=0, mode_feedback=0)
    robot = SimpleNamespace(
        mode_calls=0, status=status, foc=foc,
        get_joint_angles=lambda: SimpleNamespace(timestamp=time.time()-(1 if stale else 0)),
        get_arm_status=lambda: SimpleNamespace(timestamp=time.time(), msg=status),
        get_driver_states=lambda _: SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(foc_status=foc)),
        get_joints_enable_status_list=lambda: [True]*6,
    )

    def set_mode(mode):
        assert mode == 'j'
        robot.mode_calls += 1
        if robot.mode_calls >= acknowledge_after:
            status.ctrl_mode = status.mode_feedback = 1

    robot.set_motion_mode = set_mode
    return robot


@pytest.mark.parametrize('stale', [False, True])
def test_homing_retains_successful_hold_but_stops_on_lost_feedback(monkeypatch, stale):
    from local_setup.dual import home_arm

    arm = Arm()
    stopped = []
    emergency = []
    arm.stop = lambda: stopped.append(True)
    arm.emergency_stop = lambda: emergency.append(True)
    arm.robot = home_robot(arm, stale=stale)
    monkeypatch.setattr('local_setup.dual.make_arm', lambda cfg: arm)
    cfg = load_config(overrides={'arm': {'home_joints': arm.state.joints.tolist()}})
    if stale:
        with pytest.raises(RuntimeError, match='lost feedback'):
            home_arm(cfg, 'left')
    else:
        home_arm(cfg, 'left')
    assert bool(emergency) == stale
    assert stopped == [True]


def test_homing_repeats_same_target_when_firmware_drops_initial_commands(monkeypatch):
    from local_setup.dual import home_arm

    arm = Arm()
    target = arm.state.joints.copy()
    arm.state.joints = target + .1
    arm.robot = home_robot(arm)
    arm.stop = lambda: None
    arm.emergency_stop = lambda: pytest.fail('Successful home must retain hold')

    def command(joints):
        arm.homes.append(joints)
        if len(arm.homes) == 3:
            arm.state.joints = np.array(joints)

    arm.move_joints = command
    monkeypatch.setattr('local_setup.dual.make_arm', lambda cfg: arm)
    monkeypatch.setattr('local_setup.dual.time.sleep', lambda _: None)
    cfg = load_config(overrides={'arm': {'home_joints': target.tolist()}})
    home_arm(cfg, 'left')
    assert len(arm.homes) == 3
    assert all(joints == target.tolist() for joints in arm.homes)


def test_homing_stops_early_if_repeated_targets_produce_no_motion(monkeypatch):
    from local_setup.dual import home_arm

    arm = Arm()
    target = arm.state.joints.copy()
    arm.state.joints = target + .1
    arm.robot = home_robot(arm)
    emergency, stopped = [], []
    arm.emergency_stop = lambda: emergency.append(True)
    arm.stop = lambda: stopped.append(True)
    clock = [0.]
    monkeypatch.setattr('local_setup.dual.make_arm', lambda cfg: arm)
    monkeypatch.setattr('local_setup.dual.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('local_setup.dual.time.sleep', lambda dt: clock.__setitem__(0, clock[0] + dt))
    cfg = load_config(overrides={'arm': {'home_joints': target.tolist()}})
    with pytest.raises(RuntimeError, match='no joint movement'):
        home_arm(cfg, 'left')
    assert emergency == [True] and stopped == [True]
    assert 3.05 <= clock[0] < 3.2
    assert all(joints == target.tolist() for joints in arm.homes)


@pytest.mark.parametrize('acknowledge_after', [3, 1000])
def test_homing_waits_for_mode_ack_and_never_sends_targets_without_it(monkeypatch, acknowledge_after):
    from local_setup.dual import home_arm

    arm = Arm()
    target = arm.state.joints.copy()
    arm.state.joints = target + .1
    arm.robot = home_robot(arm, acknowledge_after=acknowledge_after)
    emergency, stopped = [], []
    arm.emergency_stop = lambda: emergency.append(True)
    arm.stop = lambda: stopped.append(True)
    clock = [0.]
    monkeypatch.setattr('local_setup.dual.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('local_setup.dual.time.sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))

    def command(joints):
        assert arm.robot.mode_calls >= acknowledge_after
        arm.homes.append(joints)
        arm.state.joints = np.array(joints)

    arm.move_joints = command
    monkeypatch.setattr('local_setup.dual.make_arm', lambda cfg: arm)
    cfg = load_config(overrides={'arm': {'home_joints': target.tolist()}})
    if acknowledge_after == 3:
        home_arm(cfg, 'left')
        assert arm.robot.mode_calls == 3 and arm.homes == [target.tolist()]
        assert not emergency
    else:
        with pytest.raises(RuntimeError, match='mode not acknowledged'):
            home_arm(cfg, 'left')
        assert not arm.homes and emergency == [True]
        assert 1 <= clock[0] < 1.1
    assert stopped == [True]


@pytest.mark.parametrize('failure', ['driver_fault', 'disabled', 'lost_mode', 'stale_ack'])
def test_homing_stops_for_driver_or_control_mode_failure(monkeypatch, failure):
    from types import SimpleNamespace
    from local_setup.dual import home_arm

    arm = Arm()
    target = arm.state.joints.copy()
    arm.state.joints = target + .1
    arm.robot = home_robot(arm)
    emergency = []
    arm.emergency_stop = lambda: emergency.append(True)
    arm.stop = lambda: None
    clock = [0.]
    monkeypatch.setattr('local_setup.dual.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('local_setup.dual.time.sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    if failure == 'driver_fault':
        arm.robot.foc.collision_status = True
    elif failure == 'disabled':
        arm.robot.get_joints_enable_status_list = lambda: [True]*5+[False]
    elif failure == 'stale_ack':
        arm.robot.get_arm_status = lambda: SimpleNamespace(timestamp=time.time()-.1, msg=arm.robot.status)
    else:
        def command(joints):
            arm.homes.append(joints)
            arm.robot.status.ctrl_mode = 0
        arm.move_joints = command
    monkeypatch.setattr('local_setup.dual.make_arm', lambda cfg: arm)
    cfg = load_config(overrides={'arm': {'home_joints': target.tolist()}})
    expected = {'driver_fault': 'collision_status', 'disabled': 'not all six motors',
                'lost_mode': 'control mode lost', 'stale_ack': 'mode not acknowledged'}
    with pytest.raises(RuntimeError, match=expected[failure]):
        home_arm(cfg, 'left')
    assert emergency == [True]
    assert len(arm.homes) == (1 if failure == 'lost_mode' else 0)
