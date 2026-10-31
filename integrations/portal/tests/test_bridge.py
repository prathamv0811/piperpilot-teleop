from types import SimpleNamespace
import json
import time

import numpy as np
import pytest
from livekit.portal import OperatorConfig, RobotConfig
from piper_teleop.dual_controller import DualController
from piper_teleop.piper_arm import FakeArm
from piper_teleop.quest_client import HandState, QuestState

from _common import ROOT
from bridge import RemoteQuestReader
from contract import action_fields, decode_quest, encode_quest, encode_state, state_fields
from hardware import configs_for


def frame(now):
    return QuestState(session_state='FOCUSED', recv_mono=now,
                      left=HandState(valid=True, tracked=True), right=HandState(valid=True, tracked=True))


def packet(f, *, sequence=1, epoch=1, now=None, echo=123, sender='operator'):
    return SimpleNamespace(values=encode_quest(f, epoch=epoch, sequence=sequence, now=now),
                           sender=sender, in_reply_to_ts_us=echo)


def test_shared_schemas_use_real_portal_parser():
    for name in ('portal.yaml', 'portal.video.yaml', 'portal.top.yaml'):
        robot = RobotConfig.from_yaml_file(ROOT/name, 'test')
        operator = OperatorConfig.from_yaml_file(ROOT/name, 'test')
        assert robot.state_schema == operator.state_schema
        assert robot.action_schema == operator.action_schema
        assert len(robot.state_schema) == len(state_fields())
        assert len(robot.action_schema) == len(action_fields())


def test_wire_keeps_both_hands_and_anchor_state_separate():
    f = frame(10)
    f.right.trigger, f.left.squeeze = .8, 1.
    f.right.pos = np.array([1., 2., 3.])
    f.space = 'anchor'
    f.anchor_have = f.anchor_tracked = True
    values = encode_quest(f, epoch=1, sequence=12, now=10.02)
    restored = decode_quest(values, received_mono=30, transport_age_s=.04)
    assert restored.right.trigger == .8 and restored.left.trigger == 0
    assert restored.left.squeeze == 1 and restored.right.squeeze == 0
    assert restored.space == 'anchor' and restored.anchor_tracked
    assert restored.recv_mono == pytest.approx(29.94)
    assert restored.frame == 12


def test_hardware_numpy_flags_produce_serializable_state():
    configs = configs_for(ROOT/'robot.example.yaml', simulated=True)
    arm = FakeArm(configs['left'])
    st = arm.get_state()
    st.joints_valid = st.eef_valid = st.gripper_valid = np.bool_(True)
    source = SimpleNamespace(get_state=lambda: st)
    values = encode_state({'left': source, 'right': source}, None,
                          simulated=False, motion_allowed=False, input_fresh=False)
    assert json.loads(json.dumps(values))['left.feedback_valid'] is True


@pytest.fixture
def mailbox():
    now = [10.]
    reader = RemoteQuestReader('operator', clock=lambda: now[0])
    reader.set_operator('operator')
    reader.sent_observation(123)
    return reader, now


def test_only_current_allowed_operator_is_accepted(mailbox):
    reader, now = mailbox
    p = packet(frame(10), now=10, sender='intruder')
    assert not reader.accept(p)
    p.sender = 'operator'
    reader.set_operator('intruder')
    assert not reader.accept(p)
    reader.set_operator('operator')
    assert reader.accept(p)


def test_unknown_and_expired_robot_echo_are_rejected(mailbox):
    reader, now = mailbox
    assert not reader.accept(packet(frame(10), now=10, echo=999))
    now[0] += .26
    assert not reader.accept(packet(frame(now[0]), now=now[0]))
    assert reader.get() is None


def test_source_age_and_transport_age_are_added(mailbox):
    reader, now = mailbox
    now[0] = 10.2
    assert not reader.accept(packet(frame(10.1), now=10.2))
    assert 'round-trip age' in reader.last_rejection


def test_replay_and_reordering_do_not_refresh_freshness(mailbox):
    reader, now = mailbox
    p = packet(frame(10), now=10, sequence=8)
    assert reader.accept(p)
    received = reader.get().recv_mono
    now[0] += .1
    assert not reader.accept(p)
    assert not reader.accept(packet(frame(now[0]), now=now[0], sequence=7))
    assert reader.get().recv_mono == received


@pytest.mark.parametrize('key,value', [('left.x', float('nan')), ('right.qw', 9),
                                      ('left.trigger', 1.5), ('space', 12),
                                      ('focused', 1), ('epoch', -1)])
def test_malformed_actions_are_rejected(mailbox, key, value):
    reader, now = mailbox
    p = packet(frame(10), now=10)
    p.values[key] = value
    assert not reader.accept(p)
    assert reader.get() is None


def test_restarted_operator_retires_old_session(mailbox):
    reader, now = mailbox
    assert reader.accept(packet(frame(10), now=10, sequence=8, epoch=1))
    generation = reader.generation
    assert reader.accept(packet(frame(10), now=10, sequence=0, epoch=2))
    assert reader.generation > generation
    assert not reader.accept(packet(frame(10), now=10, sequence=9, epoch=1))


def test_clutches_release_after_network_loss_and_hands_stay_independent():
    configs = configs_for(ROOT/'robot.example.yaml', simulated=True)
    arms = {side: FakeArm(cfg) for side, cfg in configs.items()}
    reader = RemoteQuestReader('operator')
    reader.set_operator('operator')
    ctl = DualController(reader, arms, configs)

    def send(f, seq):
        reader.sent_observation(seq)
        assert reader.accept(packet(f, sequence=seq, echo=seq))
        ctl.tick()

    f = frame(time.monotonic())
    f.right.squeeze = 1
    send(f, 1)
    assert not ctl.ready['right']
    f.right.squeeze = 0
    f.recv_mono = time.monotonic()
    send(f, 2)
    f.right.squeeze, f.right.trigger = 1, .6
    f.recv_mono = time.monotonic()
    send(f, 3)
    assert ctl.controllers['right'].status.engaged
    assert not ctl.controllers['left'].status.engaged
    assert arms['right']._grip_target == pytest.approx(.028)
    reader.get().recv_mono -= .3
    ctl.tick()
    assert not any(ctl.ready.values())
    f.recv_mono = time.monotonic()
    send(f, 4)
    assert not ctl.ready['right']
    reader.set_operator(None)
    ctl.tick()
    assert not any(c.status.engaged for c in ctl.controllers.values())


@pytest.mark.parametrize('lost_side', ['left', 'right'])
@pytest.mark.parametrize('lost_flag', ['valid', 'tracked'])
def test_untracked_zero_quaternion_only_pauses_its_own_arm(lost_side, lost_flag):
    configs = configs_for(ROOT/'robot.example.yaml', simulated=True)
    arms = {side: FakeArm(cfg) for side, cfg in configs.items()}
    reader = RemoteQuestReader('operator')
    reader.set_operator('operator')
    ctl = DualController(reader, arms, configs)
    f = frame(time.monotonic())
    seq = 0

    def send():
        nonlocal seq
        seq += 1
        f.recv_mono = time.monotonic()
        reader.sent_observation(seq)
        assert reader.accept(packet(f, sequence=seq, echo=seq))
        ctl.tick()

    send()
    f.left.squeeze = f.right.squeeze = 1.
    send()
    assert all(c.status.engaged for c in ctl.controllers.values())
    lost = f.hand(lost_side)
    other_side = 'right' if lost_side == 'left' else 'left'
    setattr(lost, lost_flag, False)
    lost.quat_xyzw = np.zeros(4)
    f.hand(other_side).trigger = .6
    send()
    assert not ctl.ready[lost_side]
    assert not ctl.controllers[lost_side].status.engaged
    assert ctl.controllers[other_side].status.engaged
    assert arms[other_side]._grip_target == pytest.approx(.028)
    assert reader.rejected == 0
    # Recovering tracking while still squeezing must not resume that arm.
    setattr(lost, lost_flag, True)
    lost.quat_xyzw = np.array([0., 0., 0., 1.])
    send()
    assert not ctl.ready[lost_side]
    lost.squeeze = 0.
    send()
    assert ctl.ready[lost_side]
    lost.squeeze = 1.
    send()
    assert all(c.status.engaged for c in ctl.controllers.values())
    # Menu remains a global pause even if its hand's pose is unavailable.
    setattr(lost, lost_flag, False)
    lost.quat_xyzw = np.zeros(4)
    lost.menu = True
    send()
    assert not any(c.status.engaged for c in ctl.controllers.values())
