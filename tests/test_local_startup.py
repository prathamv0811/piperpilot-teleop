"""Local start checks must permit joint recovery without permitting Cartesian motion."""
import pytest

from local_setup.check import validate_arm_start
from piper_teleop.config import load_config


@pytest.fixture
def cfg():
    return load_config()


FOLDED = [-0.0965, -0.0161, 0.0413, -0.0173, 0.2842, 0.0589]
HOME = [0.0, 0.85, -0.75, 0.0, 0.0, 0.0]
POSE = [0.1846, 0.0, 0.3783, 3.1416, 1.558, 3.1416]


def test_target_limit_allows_only_explicit_joint_home(cfg):
    assert validate_arm_start(cfg, FOLDED, POSE, 4, for_home=True)['joint_homing_recovery']
    with pytest.raises(RuntimeError, match='TARGET_POS_EXCEEDS_LIMIT'):
        validate_arm_start(cfg, FOLDED, POSE, 4, require_ready=True)
    with pytest.raises(RuntimeError, match='TARGET_POS_EXCEEDS_LIMIT'):
        validate_arm_start(cfg, FOLDED, POSE, 4)


@pytest.mark.parametrize('fault', [1, 2, 3, 5, 6, 7, -1])
def test_homing_does_not_bypass_other_faults(cfg, fault):
    with pytest.raises(RuntimeError, match='homing is blocked'):
        validate_arm_start(cfg, HOME, POSE, fault, for_home=True)


def test_folded_pose_cannot_start_teleop_even_without_fault(cfg):
    with pytest.raises(RuntimeError, match='still folded'):
        validate_arm_start(cfg, FOLDED, POSE, 0, require_ready=True)


def test_teleop_requires_workspace_and_accepts_ready_arm(cfg):
    with pytest.raises(RuntimeError, match='outside the teleop workspace'):
        validate_arm_start(cfg, HOME, [0.9, 0, 0.3, 0, 0, 0], 0, require_ready=True)
    assert not validate_arm_start(cfg, HOME, POSE, 0, require_ready=True)['unfold_before_teleop']


def test_joint_backend_requires_measured_wrist_inside_model_limits():
    cfg = load_config(overrides={'arm': {'backend': 'piper_joint', 'model': 'PIPER'}})
    q = [0, 1.1, -1.2, 0, -1.55, 0]
    with pytest.raises(RuntimeError, match='Joint 5 outside working limits'):
        validate_arm_start(cfg, q, POSE, 0, require_ready=True)
    q[4] = .5
    validate_arm_start(cfg, q, POSE, 0, require_ready=True)
