"""Opt-in integration tests against a real LiveKit SFU, using fake motors."""
import asyncio
import json
import os
from types import SimpleNamespace
import uuid

import pytest

from _common import ROOT
from robot import run as run_robot
from teleoperator import run as run_operator

pytestmark = pytest.mark.skipif(not os.environ.get('LIVEKIT_URL'), reason='Set LIVEKIT_URL for real transport tests')


@pytest.mark.parametrize('schema', ['portal.yaml', 'portal.video.yaml'])
async def test_dual_arm_apps_over_livekit(tmp_path, monkeypatch, schema):
    monkeypatch.setenv('LIVEKIT_ROOM', f'piperpilot-test-{uuid.uuid4().hex[:12]}')
    robot_report, operator_report = tmp_path/'robot.json', tmp_path/'operator.json'
    robot = SimpleNamespace(hardware=False, allow_motion=False, identity='piperpilot-robot',
                            operator_identity='piperpilot-operator', env_file=None,
                            schema=ROOT/schema, config=ROOT/'robot.example.yaml', camera=[],
                            seconds=6, report=robot_report)
    operator = SimpleNamespace(identity='piperpilot-operator', robot_identity='piperpilot-robot',
                               env_file=None, schema=ROOT/schema, claim_control=True,
                               synthetic=True, observe=False, display=False, seconds=4,
                               quest_host='127.0.0.1', quest_port=8735, report=operator_report)
    task = asyncio.create_task(run_robot(robot))
    try:
        await asyncio.sleep(.2)
        await run_operator(operator)
        await task
    finally:
        if not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
    r, o = json.loads(robot_report.read_text()), json.loads(operator_report.read_text())
    assert r['accepted_actions'] > 60
    assert o['observations'] > 30
    assert r['controller']['arms']['left']['engaged_ticks'] > 30
    assert r['controller']['arms']['right']['engaged_ticks'] > 30
    # The operator exits two seconds before the robot: both must release.
    assert not r['state']['left.engaged'] and not r['state']['right.engaged']
    assert not r['state']['input_fresh']
    if 'video' in schema:
        assert o['video_frames'] > 30
        assert r['sent_frames'] > 30
    assert r['max_gripper_difference_m'] > .01
