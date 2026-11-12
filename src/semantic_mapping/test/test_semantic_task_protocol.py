import json

import pytest

from semantic_mapping.semantic_task_protocol import (
    ProtocolError, decode_request, normalize_class_label, task_status,
)


def _start(**changes):
    request = {
        "schema_version": 1, "request_id": "req-1", "op": "start",
        "client_id": "client-1", "task_id": "task-1", "map_id": "map-1",
        "server_session_id": "session-1", "class_label": "瓶子", "object_id": 5,
    }
    request.update(changes)
    return request


def test_decode_start_normalizes_alias_and_preserves_exact_ids():
    request = decode_request(json.dumps(_start(), ensure_ascii=False))
    assert request["class_label"] == "bottle"
    assert request["class_id"] == 39
    assert request["object_id"] == 5
    assert normalize_class_label(" Water   Bottle ") == (39, "bottle")


@pytest.mark.parametrize("payload", [
    _start(schema_version=True),
    _start(object_id=True),
    _start(object_id=0),
    _start(class_label="chair"),
    _start(extra="unexpected"),
    _start(request_id="bad id"),
    {"schema_version": 1, "request_id": "req-2", "op": "list_objects", "extra": 1},
])
def test_decode_rejects_bad_schema_types_aliases_and_extra_fields(payload):
    with pytest.raises(ProtocolError):
        decode_request(json.dumps(payload, ensure_ascii=False, allow_nan=False))


def test_decode_rejects_malformed_and_nonfinite_json():
    with pytest.raises(ProtocolError):
        decode_request('{"schema_version":1,"request_id":"x","op":"start","n":NaN}')
    with pytest.raises(ProtocolError):
        decode_request("not-json")


def test_decode_preview_and_explicit_list_pagination():
    preview = decode_request(json.dumps({
        "schema_version": 1, "request_id": "preview-1",
        "op": "preview_navigation", "client_id": "client-1",
        "map_id": "map-1", "server_session_id": "session-1",
        "class_label": "水瓶", "object_id": 3}, ensure_ascii=False))
    assert preview["class_label"] == "bottle"
    assert preview["class_id"] == 39
    page = decode_request(json.dumps({
        "schema_version": 1, "request_id": "list-1", "op": "list_objects",
        "client_id": "client-1", "offset": 100, "limit": 50}))
    assert (page["offset"], page["limit"]) == (100, 50)
    for offset, limit in ((-1, 10), (0, 0), (0, 101), (True, 10)):
        with pytest.raises(ProtocolError):
            decode_request(json.dumps({
                "schema_version": 1, "request_id": "list-bad", "op": "list_objects",
                "client_id": "client-1", "offset": offset, "limit": limit}))


def test_task_status_has_full_task_map_client_session_and_sequence_identity():
    status = task_status(client_id="client-1", task_id="task-1", map_id="map-1",
                         server_session_id="session-1", state="canceling",
                         reason_code="cancel_requested", target={"object_id": 2}, seq=9,
                         timestamp=123.5)
    assert status["client_id"] == "client-1"
    assert status["task_id"] == "task-1"
    assert status["map_id"] == "map-1"
    assert status["server_session_id"] == "session-1"
    assert status["seq"] == 9
    assert status["message"] is None


def test_task_status_carries_a_bounded_navigation_error_message():
    status = task_status(client_id="client-1", task_id="task-1", map_id="map-1",
                         server_session_id="session-1", state="failed",
                         reason_code="navigation_failed", target={"object_id": 2},
                         seq=10, message="waiting for calibrated color CameraInfo")
    assert status["message"] == "waiting for calibrated color CameraInfo"
