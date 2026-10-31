"""Exercise the motor feedback guard without opening CAN or enabling hardware."""
import time
from types import SimpleNamespace as NS

import pytest
from piper_teleop.piper_arm import ArmState, PiperArmJoint

from hardware import CheckedPiperArm, FAULTS


@pytest.fixture
def arm(monkeypatch):
    drivers = {i: NS(timestamp=time.time(), msg=NS(foc_status=NS(**dict.fromkeys(FAULTS, False))))
               for i in range(1, 7)}
    status = NS(timestamp=time.time(), msg=NS(err_code=0))
    enabled = [True]*6
    arm = CheckedPiperArm.__new__(CheckedPiperArm)
    arm.robot = NS(get_arm_status=lambda: status, get_driver_states=drivers.get,
                   get_joints_enable_status_list=lambda: enabled)
    monkeypatch.setattr(PiperArmJoint, 'get_state', lambda _: ArmState(
        joints_valid=True, eef_valid=True, arm_status=0))
    return arm, drivers, status, enabled


def test_healthy_feedback_is_usable(arm):
    a, _, _, _ = arm
    st = a.get_state()
    assert st.arm_status == 0 and st.joints_valid and st.eef_valid
    assert not a.last_health_issues


@pytest.mark.parametrize('fault', FAULTS)
def test_j6_fault_stops_and_identifies_joint(arm, fault):
    a, drivers, _, _ = arm
    setattr(drivers[6].msg.foc_status, fault, True)
    st = a.get_state()
    assert st.arm_status != 0 and not st.joints_valid and not st.eef_valid
    assert f'J6: {fault}' in a.last_health_issues


@pytest.mark.parametrize('problem', ['stale_driver', 'stale_status', 'error_code', 'disabled', 'missing_enable'])
def test_feedback_failure_remains_blocked_and_explained(arm, problem):
    a, drivers, status, enabled = arm
    if problem == 'stale_driver':
        drivers[3].timestamp -= 1
    elif problem == 'stale_status':
        status.timestamp -= 1
    elif problem == 'error_code':
        status.msg.err_code = 1
    elif problem == 'disabled':
        enabled[1] = False
    else:
        enabled.clear()
    st = a.get_state()
    assert st.arm_status != 0 and not st.joints_valid and not st.eef_valid
    assert a.last_health_issues
