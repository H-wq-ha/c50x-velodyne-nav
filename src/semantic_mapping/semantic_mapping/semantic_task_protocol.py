"""Versioned JSON protocol shared by semantic navigation clients."""

import json
import re
import time
import uuid


SCHEMA_VERSION = 1
MAX_ID_LENGTH = 96
MAX_LABEL_LENGTH = 48
ALIASES = {
    "bottle": 39, "water bottle": 39, "瓶子": 39, "水瓶": 39, "矿泉水瓶": 39,
    "cup": 41, "杯子": 41,
    "laptop": 63, "笔记本": 63,
}
CLASS_LABELS = {39: "bottle", 41: "cup", 63: "laptop"}
TASK_STATES = {
    "planning", "navigating", "canceling", "canceled", "succeeded",
    "failed", "rejected", "unknown",
}


class ProtocolError(ValueError):
    """A request did not satisfy the wire protocol."""


def _identifier(value, field):
    if not isinstance(value, str) or not value or len(value) > MAX_ID_LENGTH:
        raise ProtocolError(f"{field}_invalid")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", value):
        raise ProtocolError(f"{field}_invalid")
    return value


def normalize_class_label(value):
    if not isinstance(value, str) or len(value) > MAX_LABEL_LENGTH:
        raise ProtocolError("class_label_invalid")
    label = " ".join(value.strip().lower().split())
    class_id = ALIASES.get(label)
    if class_id is None:
        raise ProtocolError("class_label_unknown")
    return class_id, CLASS_LABELS[class_id]


def decode_request(raw):
    if not isinstance(raw, str) or len(raw) > 4096:
        raise ProtocolError("request_too_large")
    try:
        request = json.loads(raw, parse_constant=lambda _x: (_ for _ in ()).throw(
            ProtocolError("non_finite_number")))
    except (json.JSONDecodeError, TypeError):
        raise ProtocolError("json_invalid") from None
    if not isinstance(request, dict):
        raise ProtocolError("request_invalid")
    if type(request.get("schema_version")) is not int or request["schema_version"] != SCHEMA_VERSION:
        raise ProtocolError("schema_version_unsupported")
    request_id = _identifier(request.get("request_id"), "request_id")
    op = request.get("op")
    if op not in ("list_objects", "preview_navigation", "start", "status", "cancel"):
        raise ProtocolError("operation_invalid")
    common = {"schema_version", "request_id", "op"}
    expected = {
        "list_objects": common,
        "preview_navigation": common | {"client_id", "map_id", "server_session_id",
                                        "class_label", "object_id"},
        "start": common | {"client_id", "task_id", "map_id", "server_session_id", "class_label", "object_id"},
        "status": common | {"client_id", "task_id", "map_id", "server_session_id"},
        "cancel": common | {"client_id", "task_id", "map_id", "server_session_id"},
    }[op]
    list_fields = {"client_id", "offset", "limit"}
    if set(request) != expected and not (
            op == "list_objects" and set(request) in (
                common, common | {"client_id"}, common | {"offset", "limit"},
                common | list_fields)):
        raise ProtocolError("request_fields_invalid")
    request["request_id"] = request_id
    if op == "list_objects" and "client_id" in request:
        request["client_id"] = _identifier(request["client_id"], "client_id")
    if op == "list_objects":
        if "offset" in request:
            if type(request["offset"]) is not int or request["offset"] < 0:
                raise ProtocolError("offset_invalid")
            if type(request["limit"]) is not int or not 1 <= request["limit"] <= 100:
                raise ProtocolError("limit_invalid")
    else:
        identity_fields = (("client_id", "map_id", "server_session_id")
                           if op == "preview_navigation" else
                           ("client_id", "task_id", "map_id", "server_session_id"))
        for field in identity_fields:
            request[field] = _identifier(request[field], field)
    if op in ("start", "preview_navigation"):
        class_id, label = normalize_class_label(request["class_label"])
        object_id = request["object_id"]
        if type(object_id) is not int or object_id < 1 or object_id > 2**31 - 1:
            raise ProtocolError("object_id_invalid")
        request["class_id"] = class_id
        request["class_label"] = label
    return request


def encode_response(request_id, *, ok, result=None, reason_code=None, message=None,
                    op=None, server_session_id=None, map_id=None,
                    client_id=None, task_id=None):
    response = {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id,
        "ok": bool(ok),
        "timestamp": time.time(),
        "op": op,
        "server_session_id": server_session_id,
        "map_id": map_id,
        "client_id": client_id,
        "task_id": task_id,
    }
    if result is not None:
        response["result"] = result
    if reason_code is not None:
        response["reason_code"] = reason_code
    if message is not None:
        response["message"] = str(message)[:240]
    return json.dumps(response, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def new_server_session_id():
    return uuid.uuid4().hex


def task_status(*, client_id, task_id, map_id, server_session_id, state, reason_code,
                target, seq, timestamp=None, message=None):
    if state not in TASK_STATES:
        raise ProtocolError("task_state_invalid")
    if type(seq) is not int or seq < 0:
        raise ProtocolError("sequence_invalid")
    return {
        "schema_version": SCHEMA_VERSION,
        "client_id": client_id,
        "task_id": task_id,
        "map_id": map_id,
        "server_session_id": server_session_id,
        "state": state,
        "reason_code": reason_code,
        "message": str(message)[:240] if message is not None else None,
        "target": target,
        "timestamp": time.time() if timestamp is None else timestamp,
        "seq": seq,
    }
