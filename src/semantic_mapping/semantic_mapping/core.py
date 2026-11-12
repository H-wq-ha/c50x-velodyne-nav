"""Deterministic geometry, detector postprocessing, and persistent semantic map core."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import sqlite3
from threading import RLock
from typing import Iterable, Optional, Sequence

import numpy as np

LAPTOP_CLASS_ID = 63
SEMANTIC_CLASS_IDS = (LAPTOP_CLASS_ID, 39, 41)
CLASS_NAMES = {63: "laptop", 39: "bottle", 41: "cup"}


def normalize_class_ids(class_ids: Iterable[int], *, class_count: int = 80) -> tuple[int, ...]:
    """Validate and normalize a non-empty list of supported COCO class IDs."""
    if isinstance(class_ids, (str, bytes)):
        raise ValueError("class_ids must be a non-empty sequence of integer class IDs")
    try:
        values = tuple(class_ids)
    except TypeError as exc:
        raise ValueError("class_ids must be a non-empty sequence of integer class IDs") from exc
    if not values:
        raise ValueError("class_ids must not be empty")
    normalized = []
    for value in values:
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
            raise ValueError(f"invalid class ID {value!r}; class IDs must be integers")
        class_id = int(value)
        if class_id < 0 or class_id >= int(class_count):
            raise ValueError(f"unsupported class ID {class_id}; valid COCO IDs are 0..{class_count - 1}")
        if class_id not in CLASS_NAMES:
            raise ValueError(f"unsupported semantic class ID {class_id}")
        if class_id in normalized:
            raise ValueError(f"duplicate semantic class ID {class_id}")
        normalized.append(class_id)
    return tuple(normalized)


def observation_class_fields(active_class_ids: Iterable[int]) -> dict:
    """Build truthful observation-level class metadata while retaining a legacy hint."""
    class_ids = normalize_class_ids(active_class_ids)
    return {
        "class_id": class_ids[0] if len(class_ids) == 1 else None,
        "class_ids": list(class_ids),
        "legacy_primary_class_id": LAPTOP_CLASS_ID,
    }


@dataclass(frozen=True)
class LetterboxTransform:
    scale: float
    pad_x: float
    pad_y: float
    source_width: int
    source_height: int
    target_width: int
    target_height: int

    def restore_boxes(self, boxes: np.ndarray) -> np.ndarray:
        """Map Nx4 xyxy boxes from letterboxed pixels to source pixels."""
        out = np.asarray(boxes, dtype=np.float32).copy().reshape((-1, 4))
        out[:, [0, 2]] = (out[:, [0, 2]] - self.pad_x) / self.scale
        out[:, [1, 3]] = (out[:, [1, 3]] - self.pad_y) / self.scale
        out[:, [0, 2]] = np.clip(out[:, [0, 2]], 0, self.source_width)
        out[:, [1, 3]] = np.clip(out[:, [1, 3]], 0, self.source_height)
        return out


def letterbox_image(image: np.ndarray, size: int | tuple[int, int] = 256,
                    color=(114, 114, 114)) -> tuple[np.ndarray, LetterboxTransform]:
    """Resize and pad HWC image; size is (width,height) when tuple."""
    import cv2
    h, w = image.shape[:2]
    tw, th = (size, size) if isinstance(size, int) else size
    scale = min(tw / w, th / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    left, top = (tw - nw) // 2, (th - nh) // 2
    canvas = np.full((th, tw, image.shape[2]), color, dtype=image.dtype)
    canvas[top:top + nh, left:left + nw] = resized
    return canvas, LetterboxTransform(scale, float(left), float(top), w, h, tw, th)


def box_iou(box: Sequence[float], boxes: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], boxes[:, 0]); y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2]); y2 = np.minimum(box[3], boxes[:, 3])
    inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    a = max(0., box[2]-box[0]) * max(0., box[3]-box[1])
    b = np.maximum(0, boxes[:, 2]-boxes[:, 0]) * np.maximum(0, boxes[:, 3]-boxes[:, 1])
    return inter / np.maximum(a + b - inter, 1e-9)


def postprocess_yolov8(output: np.ndarray, transform: LetterboxTransform,
                       confidence_threshold: float = .35, iou_threshold: float = .45,
                       class_id: int = LAPTOP_CLASS_ID,
                       class_ids: Optional[Iterable[int]] = None) -> list[dict]:
    """YOLOv8 detect head output [1,4+classes,N] or [1,N,4+classes]."""
    pred = np.asarray(output)
    while pred.ndim > 2 and pred.shape[0] == 1:
        pred = pred[0]
    if pred.ndim != 2:
        raise ValueError(f"unexpected detector output shape {np.asarray(output).shape}")
    # Standard COCO exports have 84 channels. Handle both common layouts, even
    # when a test or a tiny model has fewer than 84 candidate boxes.
    if pred.shape[0] == 4 + 80 and pred.shape[1] != 4 + 80:
        pred = pred.T
    class_scores = pred[:, 4:]
    requested = normalize_class_ids((class_id,) if class_ids is None else class_ids,
                                    class_count=class_scores.shape[1])
    if not np.isfinite(confidence_threshold) or not 0 <= confidence_threshold <= 1:
        raise ValueError("confidence_threshold must be finite and in [0, 1]")
    if not np.isfinite(iou_threshold) or not 0 <= iou_threshold <= 1:
        raise ValueError("iou_threshold must be finite and in [0, 1]")
    best_class = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(len(class_scores)), best_class]
    indices = np.flatnonzero(np.isin(best_class, requested) &
                             np.isfinite(scores) & (scores >= confidence_threshold))
    if not len(indices):
        return []
    rows = pred[indices]
    # YOLOv8 exports center-x, center-y, width, height followed by class scores.
    boxes = rows[:, :4].astype(np.float32).copy()
    boxes[:, :2] -= boxes[:, 2:4] / 2
    boxes[:, 2:4] += boxes[:, :2]
    kept = []
    for candidate_class in requested:
        local = np.flatnonzero(best_class[indices] == candidate_class)
        order = local[np.argsort(scores[indices[local]])[::-1]]
        while len(order):
            selected = int(order[0]); kept.append(selected)
            remaining = order[1:]
            order = remaining[box_iou(boxes[selected], boxes[remaining]) <= iou_threshold]
    kept.sort(key=lambda i: (-float(scores[indices[i]]), int(best_class[indices[i]]), i))
    original = transform.restore_boxes(boxes[kept])
    return [{"box": b.tolist(), "class_id": int(best_class[indices[k]]),
             "score": float(scores[indices[k]])} for b, k in zip(original, kept)]


def sample_depth_m(depth: np.ndarray, u: float, v: float, encoding: str,
                   window_px: int = 5, min_m: float = .15,
                   max_m: float = 6.0, min_valid_samples: int = 5,
                   min_valid_ratio: float = .4) -> tuple[Optional[float], dict]:
    """Robust local depth estimate; 16UC1 is mm, 32FC1 is meters."""
    arr = np.asarray(depth)
    x, y = int(round(u)), int(round(v)); radius = max(0, int(window_px) // 2)
    h, w = arr.shape[:2]
    if x < 0 or y < 0 or x >= w or y >= h:
        return None, {"valid": False, "reason": "pixel_out_of_bounds", "samples": 0}
    patch = arr[max(0, y-radius):min(h, y+radius+1), max(0, x-radius):min(w, x+radius+1)]
    values = patch.astype(np.float64)
    if encoding in ("16UC1", "mono16") or arr.dtype == np.uint16:
        values *= .001
    elif encoding not in ("32FC1", "64FC1", "" ) and arr.dtype.kind != "f":
        return None, {"valid": False, "reason": "unsupported_encoding", "samples": 0}
    valid = values[np.isfinite(values) & (values >= min_m) & (values <= max_m)]
    valid_ratio = float(valid.size / patch.size) if patch.size else 0.0
    if min_valid_samples < 1 or not 0.0 < min_valid_ratio <= 1.0:
        raise ValueError("min_valid_samples must be >= 1 and min_valid_ratio must be in (0, 1]")
    if valid.size < min_valid_samples or valid_ratio < min_valid_ratio:
        return None, {"valid": False, "reason": "insufficient_valid_depth",
                      "samples": int(valid.size), "valid_ratio": valid_ratio}
    if valid.size == 0:
        return None, {"valid": False, "reason": "no_valid_depth", "samples": 0}
    # Reject isolated foreground/background returns using median/MAD when enough samples exist.
    med = float(np.median(valid))
    mad = float(np.median(np.abs(valid - med)))
    inliers = valid[np.abs(valid-med) <= max(.04, 3.5 * 1.4826 * mad)]
    if not inliers.size:
        inliers = valid
    return float(np.median(inliers)), {"valid": True, "reason": "ok", "samples": int(inliers.size),
        "valid_ratio": valid_ratio, "mad_m": mad}


def backproject_pixel(u: float, v: float, depth_m: float, intrinsics: Sequence[float],
                     distortion: Sequence[float] = (), distortion_model: str = "plumb_bob") -> np.ndarray:
    """Convert pixel+depth to optical-frame xyz using K=(fx,fy,cx,cy) and distortion."""
    fx, fy, cx, cy = map(float, intrinsics)
    if fx <= 0 or fy <= 0 or depth_m <= 0:
        raise ValueError("positive focal lengths and depth are required")
    xy = np.array([(u-cx)/fx, (v-cy)/fy], dtype=np.float64)
    coeff = np.asarray(distortion, dtype=np.float64)
    if coeff.size and distortion_model in ("plumb_bob", "rational_polynomial", "equidistant"):
        import cv2
        model = cv2.fisheye if distortion_model == "equidistant" else cv2
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        pt = np.array([[[u, v]]], dtype=np.float64)
        if model is cv2.fisheye:
            norm = cv2.fisheye.undistortPoints(pt, K, coeff[:4])
        else:
            norm = cv2.undistortPoints(pt, K, coeff)
        xy = norm.reshape(2)
    return np.array([xy[0]*depth_m, xy[1]*depth_m, depth_m], dtype=np.float64)


def transform_point(matrix: np.ndarray, point: Sequence[float]) -> np.ndarray:
    """Apply validated homogeneous transform to a 3D point."""
    T = np.asarray(matrix, dtype=np.float64)
    p = np.asarray(point, dtype=np.float64).reshape(3)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError("transform must be finite homogeneous 4x4")
    return (T @ np.r_[p, 1.])[:3]


class SemanticMapStore:
    """SQLite observation log and simple confirmed static-object map."""
    def __init__(self, path: str, map_id: str, confirmations: int = 3,
                 association_gate_m: float = .35, map_frame: str = "map",
                 supported_class_ids: Iterable[int] = SEMANTIC_CLASS_IDS,
                 active_class_ids: Optional[Iterable[int]] = None):
        if not str(map_id).strip() or not str(map_frame).strip():
            raise ValueError("map_id and map_frame must be non-empty")
        if int(confirmations) < 1 or not np.isfinite(association_gate_m) or association_gate_m <= 0:
            raise ValueError("confirmations must be positive and association_gate_m must be positive")
        self.supported_class_ids = normalize_class_ids(supported_class_ids)
        self.active_class_ids = normalize_class_ids(
            self.supported_class_ids if active_class_ids is None else active_class_ids)
        if not set(self.active_class_ids).issubset(self.supported_class_ids):
            raise ValueError("active_class_ids must be included in supported_class_ids")
        self.path, self.map_id, self.map_frame = path, str(map_id), str(map_frame)
        self.confirmations = int(confirmations); self.gate = float(association_gate_m)
        self._lock = RLock()
        parent = os.path.dirname(os.path.abspath(os.path.expanduser(path)))
        os.makedirs(parent, exist_ok=True)
        self.db = sqlite3.connect(os.path.expanduser(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute("CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        row = self.db.execute("SELECT value FROM metadata WHERE key='map_id'").fetchone()
        if row is not None and row['value'] != self.map_id:
            self.db.close()
            raise ValueError(f"database map_id {row['value']!r} does not match requested {self.map_id!r}")
        self.db.execute("INSERT OR IGNORE INTO metadata VALUES ('map_id', ?)", (self.map_id,))
        manifest_row = self.db.execute("SELECT value FROM metadata WHERE key='manifest'").fetchone()
        if manifest_row is None:
            manifest_data = {"map_id": self.map_id, "map_version": 1,
                             "coordinate_frame": self.map_frame,
                             "semantic_class_id": LAPTOP_CLASS_ID}
        else:
            try:
                manifest_data = json.loads(manifest_row['value'])
                if not isinstance(manifest_data, dict):
                    manifest_data = {}
            except (TypeError, json.JSONDecodeError):
                manifest_data = {}
            manifest_data.setdefault("map_id", self.map_id)
            manifest_data.setdefault("map_version", 1)
            manifest_data.setdefault("coordinate_frame", self.map_frame)
            manifest_data.setdefault("semantic_class_id", LAPTOP_CLASS_ID)
        legacy_primary_class_id = manifest_data.get("legacy_primary_class_id", LAPTOP_CLASS_ID)
        prior_ids = manifest_data.get("supported_class_ids",
                                      manifest_data.get("semantic_class_ids", []))
        if not isinstance(prior_ids, list):
            prior_ids = []
        class_ids = list(dict.fromkeys([*prior_ids, *self.supported_class_ids]))
        manifest_data["supported_class_ids"] = class_ids
        manifest_data["semantic_class_ids"] = list(self.active_class_ids)
        manifest_data["active_class_ids"] = list(self.active_class_ids)
        manifest_data["semantic_class_id"] = (
            self.active_class_ids[0] if len(self.active_class_ids) == 1 else None)
        manifest_data["legacy_primary_class_id"] = legacy_primary_class_id
        manifest_data["semantic_classes"] = [
            {"class_id": class_id, "name": CLASS_NAMES[class_id]}
            for class_id in class_ids if class_id in CLASS_NAMES]
        manifest_data["active_semantic_classes"] = [
            {"class_id": class_id, "name": CLASS_NAMES[class_id]}
            for class_id in self.active_class_ids]
        manifest = json.dumps(manifest_data, sort_keys=True)
        self.db.execute("INSERT OR IGNORE INTO metadata VALUES ('map_version', '1')")
        self.db.execute("INSERT INTO metadata VALUES ('manifest', ?) ON CONFLICT(key) DO UPDATE SET value=excluded.value", (manifest,))
        self.db.execute("CREATE TABLE IF NOT EXISTS observations (id INTEGER PRIMARY KEY, stamp_ns INTEGER NOT NULL, frame TEXT NOT NULL, detection_index INTEGER NOT NULL DEFAULT 0, class_id INTEGER NOT NULL, score REAL NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL, payload TEXT NOT NULL)")
        columns = {r['name'] for r in self.db.execute("PRAGMA table_info(observations)")}
        if 'detection_index' not in columns:
            self.db.execute("ALTER TABLE observations ADD COLUMN detection_index INTEGER NOT NULL DEFAULT 0")
        self.db.execute("CREATE UNIQUE INDEX IF NOT EXISTS observations_frame_detection ON observations(stamp_ns,frame,detection_index)")
        self.db.execute("CREATE TABLE IF NOT EXISTS objects (object_id INTEGER PRIMARY KEY, class_id INTEGER NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL, count INTEGER NOT NULL, first_stamp_ns INTEGER NOT NULL, last_stamp_ns INTEGER NOT NULL)")
        self.db.commit()

    def add(self, stamp_ns: int, frame: str, point: Sequence[float], score: float,
            class_id: int = LAPTOP_CLASS_ID, payload: Optional[dict] = None,
            detection_index: int = 0) -> dict:
        p = np.asarray(point, dtype=float).reshape(3)
        try:
            normalized_class_id = normalize_class_ids([class_id])[0]
        except ValueError as exc:
            raise ValueError("observation has an unsupported class ID or invalid data") from exc
        if (normalized_class_id not in self.supported_class_ids or not np.isfinite(p).all() or int(stamp_ns) <= 0
                or not str(frame).strip() or not np.isfinite(score) or not 0 <= score <= 1):
            raise ValueError("observation has an unsupported class ID or invalid data")
        class_id = normalized_class_id
        stamp_ns = int(stamp_ns)
        with self._lock, self.db:
            try:
                self.db.execute("INSERT INTO observations(stamp_ns,frame,detection_index,class_id,score,x,y,z,payload) VALUES(?,?,?,?,?,?,?,?,?)",
                    (stamp_ns, frame, int(detection_index), class_id, float(score), *p.tolist(), json.dumps(payload or {}, sort_keys=True)))
            except sqlite3.IntegrityError as exc:
                if 'UNIQUE constraint failed' not in str(exc):
                    raise
                return {"object_id": None, "count": 0, "confirmed": False, "status": "duplicate_detection"}
            objs = self.db.execute("SELECT * FROM objects WHERE class_id=?", (class_id,)).fetchall()
            nearby = [(float(np.linalg.norm(p - np.array([o['x'],o['y'],o['z']]))), o) for o in objs]
            nearby = [item for item in nearby if item[0] <= self.gate and item[1]['last_stamp_ns'] != stamp_ns]
            if nearby:
                _, obj = min(nearby, key=lambda pair: pair[0]); n = obj['count'] + 1
                q = (np.array([obj['x'],obj['y'],obj['z']])*obj['count'] + p)/n
                self.db.execute("UPDATE objects SET x=?,y=?,z=?,count=?,last_stamp_ns=? WHERE object_id=?", (*q.tolist(), n, stamp_ns, obj['object_id']))
                oid = obj['object_id']
            else:
                cur = self.db.execute("INSERT INTO objects(class_id,x,y,z,count,first_stamp_ns,last_stamp_ns) VALUES(?,?,?,?,?,?,?)",
                    (class_id,*p.tolist(),1,stamp_ns,stamp_ns)); oid = cur.lastrowid; n = 1
        return {"object_id": int(oid), "count": int(n), "confirmed": n >= self.confirmations, "status": "added"}

    def objects(self, confirmed_only: bool = False) -> list[dict]:
        with self._lock:
            rows = self.db.execute("SELECT * FROM objects ORDER BY object_id").fetchall()
            return [dict(object_id=int(r['object_id']), class_id=r['class_id'], position=[r['x'],r['y'],r['z']],
                         observations=r['count'], confirmed=r['count'] >= self.confirmations,
                         first_stamp_ns=r['first_stamp_ns'],last_stamp_ns=r['last_stamp_ns'])
                    for r in rows if not confirmed_only or r['count'] >= self.confirmations]

    def close(self):
        with self._lock:
            try:
                self.db.close()
            except sqlite3.Error:
                pass
