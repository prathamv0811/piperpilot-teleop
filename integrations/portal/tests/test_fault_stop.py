"""A hardware-style fault must stop both simulated arms and persist its cause."""
import json
import os
import time
from types import SimpleNamespace
import uuid

import pytest
from piper_teleop.piper_arm import FakeArm

from _common import ROOT
from robot import run


@pytest.mark.skipif(not os.environ.get('LIVEKIT_URL'), reason='Set LIVEKIT_URL for real transport tests')
async def test_fault_stop_records_driver_cause_and_stops_both(tmp_path, monkeypatch):
    monkeypatch.setenv('LIVEKIT_ROOM', 'piperpilot-fault-test-'+uuid.uuid4().hex[:12])
    original_get = FakeArm.get_state
    first = [None]
    stops = []

    def measured(arm):
        if first[0] is None:
            first[0] = time.monotonic()
        state = original_get(arm)
        if time.monotonic()-first[0] > .3:
            state.arm_status = -2
            state.joints_valid = state.eef_valid = False
            arm.last_health_issues = ['J6: collision_status', 'J6: driver_error_status']
        return state

    monkeypatch.setattr(FakeArm, 'get_state', measured)
    monkeypatch.setattr(FakeArm, 'emergency_stop', lambda arm: stops.append(arm))
    args = SimpleNamespace(hardware=False, allow_motion=False, identity='piperpilot-robot',
                           operator_identity='piperpilot-operator', env_file=None,
                           schema=ROOT/'portal.yaml', config=ROOT/'robot.example.yaml',
                           camera=[], seconds=3, report=tmp_path/'fault.json')
    with pytest.raises(RuntimeError, match='missing feedback or fault -2'):
        await run(args)
    report = json.loads(args.report.read_text())
    assert report['running'] is False
    assert 'left' in report['terminal_error']
    assert report['health_issues']['left'] == ['J6: collision_status', 'J6: driver_error_status']
    assert len(stops) == 2 and stops[0] is not stops[1]
