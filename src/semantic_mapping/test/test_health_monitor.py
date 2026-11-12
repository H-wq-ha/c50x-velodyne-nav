import json

from semantic_mapping.health_monitor import is_successful_semantic_heartbeat


def test_empty_detection_frame_is_a_healthy_semantic_heartbeat():
    assert is_successful_semantic_heartbeat(json.dumps({
        'status': 'ok', 'detections': [],
    }))


def test_unmapped_or_failed_observation_is_not_a_healthy_heartbeat():
    for status in ('processing_error', 'tf_expired', 'tf_unavailable_camera_observations_only'):
        assert not is_successful_semantic_heartbeat(json.dumps({'status': status}))
    assert not is_successful_semantic_heartbeat('{invalid json')
