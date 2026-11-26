import numpy as np
import pytest

from piper_control.model import RobotModel, pose_from_matrix
from scripts.analyze_tcp_performance import reconstructed_stop_metrics


def record(axis, anchored):
    q = np.array([.01, 1.3, -1.286, 1.08, -.01, 0.])
    q[axis] = -.003
    model = RobotModel()
    inferred, anchors = [False]*6, [True]*6
    inferred[axis], anchors[axis] = True, anchored
    rows = [dict(timestamp=1000.+i*.025, positions=q.tolist(),
                 extra={'feedback_reconstruction': dict(inferred=inferred, anchored=anchors)})
            for i in range(80)]
    return model, q, dict(samples=rows, motion_finished_unix=1000.5,
                          accepted_unix=999.9, motion={'duration': .3},
                          target_pose=pose_from_matrix(model.forward(q)))


def test_censored_raw_position_is_not_scored_as_the_actual_stop():
    model, q, report = record(4, True)
    result = reconstructed_stop_metrics(report, model, q, True)
    raw = q.copy()
    raw[4] = 0.
    assert np.linalg.norm(model.forward(raw)[:3, 3]-model.forward(q)[:3, 3]) > .0005
    assert result['final_position_error_mm'] < 1e-8
    assert result['endpoint_inferred_joints'] == [5]
    assert result['endpoint_position_estimate_anchored']


@pytest.mark.parametrize('axis,position_anchored', [(4, False), (5, True)])
def test_unanchored_estimate_remains_explicit_and_checks_its_tcp_effect(axis, position_anchored):
    model, q, report = record(axis, False)
    result = reconstructed_stop_metrics(report, model, q, False)
    assert result['endpoint_unanchored_joints'] == [axis+1]
    assert result['endpoint_position_estimate_anchored'] == position_anchored
    assert result['post_success_max_distance_from_final_mm'] is None


def test_missing_stop_samples_are_not_a_zero_motion_measurement():
    model, q, report = record(4, True)
    report['samples'] = report['samples'][::10]
    with pytest.raises(ValueError, match='Insufficient'):
        reconstructed_stop_metrics(report, model, q, True)


def test_motion_between_completion_and_client_poll_is_included():
    model, q, report = record(4, True)
    report['result'] = dict(finished_timestamp=1000.3)
    report['samples'][13]['positions'] = (q+[.003, 0, 0, 0, 0, 0]).tolist()
    result = reconstructed_stop_metrics(report, model, q, True)
    assert result['post_success_max_distance_from_final_mm'] > .5
    del report['result']
    assert reconstructed_stop_metrics(report, model, q, True)['post_success_max_distance_from_final_mm'] < 1e-8


def test_position_settle_time_includes_a_late_excursion_and_never_assumes_success():
    model, q, report = record(4, True)
    report['samples'][52]['positions'] = (q+[.01, 0, 0, 0, 0, 0]).tolist()
    result = reconstructed_stop_metrics(report, model, q, True)
    assert result['after_plan_1mm_position_settle_s'] == pytest.approx(1.125)
    report['samples'][-1]['positions'] = report['samples'][52]['positions']
    assert reconstructed_stop_metrics(report, model, q, False)['after_plan_1mm_position_settle_s'] is None


def test_gap_before_terminal_does_not_invalidate_a_complete_final_hold():
    model, q, report = record(4, True)
    del report['samples'][1:15]
    result = reconstructed_stop_metrics(report, model, q, True)
    assert result['final_position_error_mm'] < 1e-8
    assert not result['position_settle_data_complete']
    assert result['after_plan_1mm_position_settle_s'] is None
    # Missing observations after completion still invalidate drift measurements.
    model, q, report = record(4, True)
    del report['samples'][21:35]
    with pytest.raises(ValueError, match='Insufficient'):
        reconstructed_stop_metrics(report, model, q, True)
