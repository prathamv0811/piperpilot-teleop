"""Typed Quest actions and dual-PiPER state for the shared Portal schema."""
from __future__ import annotations

import math
import time
import numpy as np
from piper_teleop.quest_client import HandState, QuestState

SIDES = ('left', 'right')
SPACES = ('local', 'stage', 'anchor')
HAND_FLOATS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'trigger', 'squeeze')
HAND_BOOLS = ('valid', 'tracked', 'menu')


def action_fields():
    fields = [('epoch', 'u32'), ('sequence', 'u32'), ('source_age_s', 'f32'),
              ('focused', 'bool'), ('space', 'u8'), ('anchor_have', 'bool'),
              ('anchor_tracked', 'bool'), ('calibrating', 'bool')]
    for side in SIDES:
        fields += [(f'{side}.{key}', 'f32') for key in HAND_FLOATS]
        fields += [(f'{side}.{key}', 'bool') for key in HAND_BOOLS]
    return fields


def state_fields():
    fields = [('simulated', 'bool'), ('motion_allowed', 'bool'), ('input_fresh', 'bool')]
    for side in SIDES:
        fields += [(f'{side}.j{i}', 'f32') for i in range(1, 7)]
        fields += [(f'{side}.{key}', 'f32') for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper_m')]
        fields += [(f'{side}.status', 'i16'), (f'{side}.feedback_valid', 'bool'),
                   (f'{side}.gripper_valid', 'bool'),
                   (f'{side}.ready', 'bool'), (f'{side}.engaged', 'bool')]
    return fields


def encode_quest(frame, *, epoch, sequence, now=None):
    now = time.monotonic() if now is None else now
    values = dict(epoch=epoch, sequence=sequence, source_age_s=max(0., now-frame.recv_mono),
                  focused=frame.session_state == 'FOCUSED', space=SPACES.index(frame.space),
                  anchor_have=frame.anchor_have, anchor_tracked=frame.anchor_tracked,
                  calibrating=bool(frame.calib_step))
    for side in SIDES:
        hand = frame.hand(side)
        for key, value in zip(HAND_FLOATS, [*hand.pos, *hand.quat_xyzw, hand.trigger, hand.squeeze]):
            values[f'{side}.{key}'] = float(value)
        for key in HAND_BOOLS:
            values[f'{side}.{key}'] = bool(getattr(hand, key))
    return values


def decode_quest(values, *, received_mono, transport_age_s):
    if set(values) != {name for name, _ in action_fields()}:
        raise ValueError('Action fields differ from portal.yaml')
    for name, dtype in action_fields():
        value = values[name]
        if dtype == 'bool':
            if not isinstance(value, (bool, np.bool_)):
                raise ValueError(f'{name} must be boolean')
        elif isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)) or not math.isfinite(value):
            raise ValueError(f'{name} must be finite numeric data')
        elif dtype in ('u32', 'u8') and (int(value) != value or not 0 <= value <= (2**32-1 if dtype == 'u32' else 255)):
            raise ValueError(f'{name} is outside its unsigned integer range')
    space = int(values['space'])
    if space >= len(SPACES) or not 0 <= values['source_age_s'] <= .25:
        raise ValueError('Invalid coordinate space or stale Quest sample')
    hands = {}
    for side in SIDES:
        v = lambda key: values[f'{side}.{key}']
        if not (v('valid') and v('tracked')):
            # OpenXR supplies a zero quaternion when a hand has no valid pose.
            # Keep that hand inert without discarding the other hand's input.
            hands[side] = HandState(menu=v('menu'))
            continue
        pos = np.array([v(k) for k in ('x', 'y', 'z')])
        quat = np.array([v(k) for k in ('qx', 'qy', 'qz', 'qw')])
        if np.max(np.abs(pos)) > 100 or not .9 <= np.linalg.norm(quat) <= 1.1:
            raise ValueError('Invalid controller pose')
        if not all(0 <= v(k) <= 1 for k in ('trigger', 'squeeze')):
            raise ValueError('Invalid controller trigger or squeeze')
        hands[side] = HandState(pos=pos, quat_xyzw=quat, trigger=v('trigger'), squeeze=v('squeeze'),
                               valid=v('valid'), tracked=v('tracked'), menu=v('menu'))
    return QuestState(frame=int(values['sequence']),
                      session_state='FOCUSED' if values['focused'] else 'UNKNOWN',
                      space=SPACES[space], anchor_have=values['anchor_have'],
                      anchor_tracked=values['anchor_tracked'], calib_step=int(values['calibrating']),
                      recv_mono=received_mono-values['source_age_s']-transport_age_s, **hands)


def encode_state(arms, controller, *, simulated, motion_allowed, input_fresh):
    values = {'simulated': simulated, 'motion_allowed': motion_allowed, 'input_fresh': input_fresh}
    for side, arm in arms.items():
        st = arm.get_state()
        for i, value in enumerate(st.joints, 1):
            values[f'{side}.j{i}'] = float(value)
        for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), st.eef_pose6):
            values[f'{side}.{key}'] = float(value)
        values.update({f'{side}.gripper_m': float(st.gripper_width), f'{side}.status': st.arm_status,
                       f'{side}.feedback_valid': bool(st.joints_valid and st.eef_valid),
                       f'{side}.gripper_valid': bool(st.gripper_valid),
                       f'{side}.ready': bool(controller and controller.ready[side]),
                       f'{side}.engaged': bool(controller and controller.controllers[side].status.engaged)})
    return values
