"""Two independent arms driven from one timestamped Quest frame.

The caller owns the synchronous tick loop and all resource cleanup. There
are no controller threads and no opposite-hand homing shortcuts.
"""
from dataclasses import replace
import time

import numpy as np

from .quest_client import HandState
from .teleop_controller import TeleopController


class FrameInput:
    """One immutable frame for both controllers in a control-loop iteration."""

    def __init__(self, reader):
        self.reader = reader
        self.state = None

    def get(self):
        return self.state

    def age(self):
        return float('inf') if self.state is None else time.monotonic() - self.state.recv_mono

    def rate_hz(self):
        return self.reader.rate_hz()

    def send_haptic(self, *args, **kwargs):
        self.reader.send_haptic(*args, **kwargs)


class DualController:
    def __init__(self, reader, arms, configs):
        self.reader = reader
        self.arms = arms
        self.configs = configs
        self.input = FrameInput(reader)
        self.controllers = {}
        for side in ('left', 'right'):
            cfg = configs[side]
            if cfg.quest.control_hand != side or cfg.quest.get('other_hand_gestures', True):
                raise ValueError('Dual control requires matching hands and other_hand_gestures=false')
            self.controllers[side] = TeleopController(self.input, arms[side], cfg)
        self.ready = dict.fromkeys(arms, False)
        self.reason = 'release both grips and triggers'
        self._space = None
        self._frame_time = None
        self.engaged_ticks = dict.fromkeys(arms, 0)

    def reset(self, reason):
        self.ready = dict.fromkeys(self.arms, False)
        self.reason = reason
        self.input.state = None
        for ctl in self.controllers.values():
            ctl._disengage(reason)

    @staticmethod
    def hand_usable(hand):
        return (hand.valid and hand.tracked
                and np.shape(hand.pos) == (3,) and np.isfinite(hand.pos).all()
                and np.shape(hand.quat_xyzw) == (4,) and np.isfinite(hand.quat_xyzw).all()
                and 0.9 <= np.linalg.norm(hand.quat_xyzw) <= 1.1
                and np.isfinite(hand.trigger) and 0 <= hand.trigger <= 1
                and np.isfinite(hand.squeeze) and 0 <= hand.squeeze <= 1)

    def tick(self):
        # Check BOTH arms before either receives this tick's commands.
        for side, arm in self.arms.items():
            st = arm.get_state()
            if (st.arm_status != 0 or not st.joints_valid or not st.eef_valid
                    or not np.isfinite(st.joints).all() or not np.isfinite(st.eef_pose6).all()):
                self.reset(f'{side}: missing feedback or fault {st.arm_status}')
                raise RuntimeError(self.reason)

        frame = self.reader.get()
        timeout = min(cfg.quest.stale_timeout_s for cfg in self.configs.values())
        if frame is None or not 0 <= time.monotonic() - frame.recv_mono <= timeout:
            self.reset('Quest stream stale; release controls to resume')
            return
        if frame.session_state != 'FOCUSED' or frame.calib_step:
            self.reset('Quest not focused or calibration active')
            return
        if frame.anchor_have and not frame.anchor_tracked:
            self.reset('saved room anchor not tracked')
            return
        space = (frame.space, frame.anchor_have)
        if self._space is not None and space != self._space:
            self.reset('coordinate frame changed; release controls to resume')
        self._space = space
        if self._frame_time is not None and frame.recv_mono - self._frame_time > timeout:
            self.reset('Quest frame gap; release controls to resume')
        new_frame = frame.recv_mono != self._frame_time
        self._frame_time = frame.recv_mono
        if frame.left.menu or frame.right.menu:
            self.reset('menu pressed; release controls to resume')
            return

        hands = {}
        for side in self.arms:
            hand = frame.hand(side)
            if not self.hand_usable(hand):
                self.ready[side] = False
                self.reason = f'{side} controller not tracked'
            elif (not self.ready[side] and new_frame
                  and hand.squeeze < self.configs[side].teleop.clutch_release
                  and hand.trigger < 0.1):
                self.ready[side] = True
            hands[side] = hand if self.ready[side] else HandState()
        self.input.state = replace(frame, **hands)
        for side, ctl in self.controllers.items():
            ctl._tick()
            self.engaged_ticks[side] += int(ctl.status.engaged)
        if all(self.ready.values()):
            self.reason = 'ready; hold each grip to move its arm'

    def status(self):
        states = {side: arm.get_state() for side, arm in self.arms.items()}
        return {
            'input_hz': self.reader.rate_hz(), 'reason': self.reason,
            'arms': {side: {
                'ready': self.ready[side], 'engaged': ctl.status.engaged,
                'engaged_ticks': self.engaged_ticks[side],
                'last_error': ctl.status.last_error,
                'joints_rad': states[side].joints.tolist(),
                'eef_pose6': states[side].eef_pose6.tolist(),
                'gripper_width': states[side].gripper_width,
                'arm_status': states[side].arm_status,
            } for side, ctl in self.controllers.items()},
        }
