"""Native role assignment must refuse ambiguous wiring and unhealthy feedback."""
from collections import Counter
import struct
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from local_setup import leader_follower as lf


def robot():
    now = time.time()
    arm = Mock()
    arm.get_arm_status.return_value = NS(timestamp=now, msg=NS(
        ctrl_mode=1, arm_status=0, mode_feedback=1, err_code=0))
    arm.get_joint_angles.return_value = NS(timestamp=now, msg=[0., 1., -1., 0., .5, 0.])
    arm.get_driver_states.return_value = NS(timestamp=now, msg=NS(
        foc_status=NS(**dict.fromkeys(lf.FAULTS, False))))
    return arm


def assign(robots, **kwargs):
    options = dict(separate_buses=True, workspace_clear=True,
                   traffic={'can0': {'counts': {'0x2a1': 100}},
                            'can1': {'counts': {'0x2a1': 100}}}, sent=[])
    options.update(kwargs)
    lf.send_roles(robots, **options)
    return options['sent']


@pytest.mark.parametrize('condition', ['shared', 'unclear', 'active', 'stale_status',
                                     'collision', 'estop', 'stale_driver', 'invalid_joints'])
def test_neither_role_is_written_when_preflight_fails(condition):
    arms = {side: robot() for side in ('left', 'right')}
    kwargs = {}
    if condition == 'shared':
        kwargs['separate_buses'] = False
    elif condition == 'unclear':
        kwargs['workspace_clear'] = False
    elif condition == 'active':
        kwargs['traffic'] = {'can1': {'counts': {'0x155': 1}}}
    elif condition == 'stale_status':
        arms['right'].get_arm_status.return_value.timestamp -= 1
    elif condition == 'collision':
        arms['left'].get_driver_states.return_value.msg.foc_status.collision_status = True
    elif condition == 'estop':
        arms['left'].get_arm_status.return_value.msg.arm_status = 1
    elif condition == 'stale_driver':
        arms['right'].get_driver_states.return_value.timestamp -= 1
    elif condition == 'invalid_joints':
        arms['left'].get_joint_angles.return_value.msg[3] = float('nan')
    with pytest.raises(RuntimeError):
        assign(arms, **kwargs)
    for arm in arms.values():
        arm.set_leader_mode.assert_not_called()
        arm.set_follower_mode.assert_not_called()


def test_correct_roles_only_no_motion_gripper_or_calibration_calls():
    arms = {side: robot() for side in ('left', 'right')}
    sent = assign(arms)
    assert [(item['side'], item['role']) for item in sent] == [('right', 'follower'), ('left', 'leader')]
    arms['left'].set_leader_mode.assert_called_once_with()
    arms['right'].set_follower_mode.assert_called_once_with()
    allowed = {'get_arm_status', 'get_joint_angles', 'get_driver_states'}
    for side, arm in arms.items():
        assert {call[0] for call in arm.mock_calls} <= allowed | {
            'set_leader_mode' if side == 'left' else 'set_follower_mode'}


def test_partial_write_is_recorded_if_second_role_write_fails():
    arms = {side: robot() for side in ('left', 'right')}
    arms['left'].set_leader_mode.side_effect = RuntimeError('CAN transmit failed')
    sent = []
    with pytest.raises(RuntimeError):
        assign(arms, sent=sent)
    assert [(item['side'], item['role']) for item in sent] == [('right', 'follower')]


def test_shared_bus_traffic_is_not_claimed_as_verified_following():
    now = time.time()
    ids = lf.LEADER_IDS + lf.FEEDBACK_IDS
    latest = {i: (now, struct.pack('>ii', 1000, -2000)) for i in ids}
    report = lf.summarize(Counter({i: 20 for i in ids}), latest)
    assert report['observed'] == 'leader and follower traffic'
    assert report['leader_joints_deg'] == [1, -2] * 3
    assert report['instant_joint_error_deg'] == [0] * 6
    assert 'verified' not in report


def test_old_or_incomplete_leader_frames_are_not_fresh():
    now = time.time()
    latest = {i: (now, bytes(8)) for i in lf.LEADER_IDS}
    latest[0x157] = (now - 1, bytes(8))
    report = lf.summarize(Counter({i: 10 for i in latest}), latest)
    assert not report['leader_frames_fresh']
    assert 'leader_joints_deg' not in report


def test_gripper_command_and_measured_opening_are_distinguished():
    now = time.time()
    latest = {0x159: (now, struct.pack('>ihBB', 30000, 2000, 1, 0)),
              0x2A8: (now, struct.pack('>ihBB', 29500, 100, 64, 0))}
    report = lf.summarize(Counter({i: 100 for i in latest}), latest)
    assert report['leader_gripper']['width_mm'] == 30
    assert report['follower_gripper']['width_mm'] == 29.5
    assert report['follower_gripper']['enabled']
    assert report['follower_gripper']['fault_bits'] == 0
    assert report['instant_gripper_error_mm'] == -.5


def test_angle_mode_is_not_reported_as_width():
    now = time.time()
    latest = {0x159: (now, struct.pack('>ihBB', 30000, 2000, 5, 0)),
              0x2A8: (now, struct.pack('>ihBB', 29500, 100, 64, 1))}
    report = lf.summarize(Counter({i: 100 for i in latest}), latest)
    assert report['leader_gripper']['angle_deg'] == 30
    assert 'width_mm' not in report['leader_gripper']
    assert 'instant_gripper_error_mm' not in report


def test_shared_check_resolves_right_adapter_renamed_to_can0(monkeypatch):
    base = lf.load_config(str(lf.ROOT / 'dual.example.yaml'))
    serial = base.dual.arms.right.usb_serial
    monkeypatch.setattr(lf, 'can_inventory', lambda: {'can0': {'usb_serial': serial, 'up': True}})
    configs, adapters = lf.shared_config(lf.ROOT / 'dual.example.yaml')
    assert configs['follower'].arm.channel == 'can0'
    assert adapters['shared']['usb_serial'] == serial


def test_shared_check_rejects_down_adapter(monkeypatch):
    base = lf.load_config(str(lf.ROOT / 'dual.example.yaml'))
    monkeypatch.setattr(lf, 'can_inventory', lambda: {
        'can0': {'usb_serial': base.dual.arms.right.usb_serial, 'up': False}})
    with pytest.raises(RuntimeError, match='can0 is down'):
        lf.shared_config(lf.ROOT / 'dual.example.yaml')


def test_shared_mode_never_configures_roles():
    with pytest.raises(SystemExit) as exc:
        lf.main(['configure', '--shared', '--separate-buses', '--workspace-clear'])
    assert exc.value.code == 2
