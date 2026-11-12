import numpy as np
import json
import pytest
import sqlite3
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

from semantic_mapping.core import (
    CLASS_NAMES, SEMANTIC_CLASS_IDS, SemanticMapStore, backproject_pixel,
    letterbox_image, normalize_class_ids, observation_class_fields,
    postprocess_yolov8,
    sample_depth_m, transform_point,
)
from semantic_mapping.node import select_latest_ready_frame


def test_letterbox_restores_source_coordinates():
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    boxed, tf = letterbox_image(image, (256, 256))
    assert boxed.shape == (256, 256, 3)
    assert tf.scale == pytest.approx(1.28)
    restored = tf.restore_boxes(np.array([[64, 96, 192, 160]], dtype=np.float32))
    np.testing.assert_allclose(restored[0], [50, 25, 150, 75], atol=1e-5)


def test_yolo_postprocess_keeps_only_laptop_class_and_restores_box():
    # Head layout [1, 4+80, N], matching YOLOv8 ONNX export.
    pred = np.zeros((1, 84, 2), dtype=np.float32)
    pred[0, :4, 0] = [128, 128, 64, 64]
    pred[0, 4+63, 0] = .9
    pred[0, :4, 1] = [128, 128, 60, 60]
    pred[0, 4+2, 1] = .99
    _, tf = letterbox_image(np.zeros((100,200,3), np.uint8), (256,256))
    detections = postprocess_yolov8(pred, tf, .3)
    assert len(detections) == 1
    assert detections[0]['class_id'] == 63
    assert detections[0]['score'] == pytest.approx(.9)
    np.testing.assert_allclose(detections[0]['box'], [75,25,125,75], atol=1e-4)


def test_yolo_postprocess_supports_laptop_bottle_and_cup_with_class_aware_nms():
    pred = np.zeros((1, 84, 4), dtype=np.float32)
    pred[0, :4, :] = np.array([
        [128, 128, 64, 64], [128, 128, 60, 60],
        [128, 128, 58, 58], [128, 128, 50, 50],
    ], dtype=np.float32).T
    pred[0, 4 + 63, 0] = .91
    pred[0, 4 + 63, 1] = .84  # Same-class overlap is suppressed.
    pred[0, 4 + 39, 2] = .88
    pred[0, 4 + 41, 3] = .86  # Different classes survive even at the same location.
    _, tf = letterbox_image(np.zeros((256, 256, 3), np.uint8), (256, 256))

    detections = postprocess_yolov8(pred, tf, .3, .45, class_ids=[63, 39, 41])

    assert [(d['class_id'], d['score']) for d in detections] == [
        (63, pytest.approx(.91)), (39, pytest.approx(.88)), (41, pytest.approx(.86))]
    assert {CLASS_NAMES[d['class_id']] for d in detections} == {'laptop', 'bottle', 'cup'}


@pytest.mark.parametrize('class_ids', [[], [63, 63], [80], [-1], [39.0], '39'])
def test_class_id_validation_rejects_empty_duplicate_or_unsupported_values(class_ids):
    with pytest.raises(ValueError):
        normalize_class_ids(class_ids)


def test_observation_metadata_uses_real_single_class_and_null_for_multiple():
    assert observation_class_fields([39]) == {
        'class_id': 39, 'class_ids': [39], 'legacy_primary_class_id': 63}
    assert observation_class_fields([63, 39, 41]) == {
        'class_id': None, 'class_ids': [63, 39, 41], 'legacy_primary_class_id': 63}


def test_postprocess_rejects_semantic_class_absent_from_detector_output():
    pred = np.zeros((1, 43, 1), dtype=np.float32)
    _, tf = letterbox_image(np.zeros((256, 256, 3), np.uint8), (256, 256))
    with pytest.raises(ValueError, match='unsupported class ID'):
        postprocess_yolov8(pred, tf, class_ids=[63, 39, 41])


def test_depth_units_filtering_and_invalid_samples():
    depth_mm = np.full((9,9), 1200, dtype=np.uint16)
    depth_mm[4,4] = 0
    z, quality = sample_depth_m(depth_mm, 4, 4, '16UC1', 5)
    assert z == pytest.approx(1.2)
    assert quality['valid'] and quality['samples'] > 0
    depth_m = np.full((9,9), np.nan, dtype=np.float32)
    assert sample_depth_m(depth_m, 4, 4, '32FC1')[0] is None
    assert sample_depth_m(depth_mm, 20, 4, '16UC1')[1]['reason'] == 'pixel_out_of_bounds'


def test_depth_requires_minimum_pixel_support_and_ratio():
    sparse = np.zeros((5, 5), dtype=np.uint16)
    sparse[2, 2] = 1000
    z, quality = sample_depth_m(sparse, 2, 2, '16UC1', 5)
    assert z is None
    assert quality['reason'] == 'insufficient_valid_depth'
    assert quality['samples'] == 1
    supported = np.full((5, 5), 1000, dtype=np.uint16)
    z, quality = sample_depth_m(supported, 2, 2, '16UC1', 5)
    assert z == pytest.approx(1.0)
    assert quality['samples'] == 25 and quality['valid_ratio'] == pytest.approx(1.0)


def test_backprojection_and_transform_matrix():
    point = backproject_pixel(330, 250, 2.0, [500, 500, 320, 240])
    np.testing.assert_allclose(point, [.04, .04, 2.])
    angle = np.pi/2
    T = np.array([[np.cos(angle),-np.sin(angle),0,1],
                  [np.sin(angle), np.cos(angle),0,2],
                  [0,0,1,3],[0,0,0,1]])
    np.testing.assert_allclose(transform_point(T, [1,0,0]), [1,3,3], atol=1e-8)
    with pytest.raises(ValueError): transform_point(np.eye(3), [1,2,3])


def test_fusion_dedup_confirmation_and_separate_objects(tmp_path):
    path = str(tmp_path / 'map.sqlite3')
    store = SemanticMapStore(path, 'map-a', confirmations=3, association_gate_m=.25)
    manifest = store.db.execute("SELECT value FROM metadata WHERE key='manifest'").fetchone()[0]
    manifest_json = __import__('json').loads(manifest)
    assert manifest_json['map_id'] == 'map-a' and manifest_json['map_version'] == 1
    assert manifest_json['semantic_class_id'] is None
    assert manifest_json['legacy_primary_class_id'] == 63
    assert manifest_json['supported_class_ids'] == list(SEMANTIC_CLASS_IDS)
    assert manifest_json['semantic_class_ids'] == list(SEMANTIC_CLASS_IDS)
    assert manifest_json['active_class_ids'] == list(SEMANTIC_CLASS_IDS)
    a1 = store.add(1, 'map', [1,0,0], .9)
    a2 = store.add(2, 'map', [1.05,0,0], .8)
    a3 = store.add(3, 'map', [.95,0,0], .85)
    b1 = store.add(4, 'map', [3,0,0], .9)
    assert not a1['confirmed'] and not a2['confirmed'] and a3['confirmed']
    assert a1['object_id'] == a3['object_id'] != b1['object_id']
    assert len(store.objects(confirmed_only=True)) == 1
    store.close()
    store = SemanticMapStore(path, 'map-a', confirmations=3)
    assert len(store.objects()) == 2
    assert store.objects()[0]['observations'] == 3
    store.close()
    with pytest.raises(ValueError, match='does not match'):
        SemanticMapStore(path, 'map-b')


def test_store_maps_each_class_and_never_associates_across_classes(tmp_path):
    store = SemanticMapStore(str(tmp_path / 'multi_class.sqlite3'), 'multi', confirmations=2)
    laptop = store.add(1, 'map', [1, 2, 3], .9, class_id=63)
    bottle = store.add(2, 'map', [1, 2, 3], .8, class_id=39)
    cup = store.add(3, 'map', [1, 2, 3], .7, class_id=41)
    bottle_confirmed = store.add(4, 'map', [1.02, 2, 3], .85, class_id=39)
    objects = store.objects()

    assert laptop['object_id'] != bottle['object_id'] != cup['object_id']
    assert bottle_confirmed['object_id'] == bottle['object_id']
    assert bottle_confirmed['confirmed']
    assert {(obj['class_id'], obj['observations']) for obj in objects} == {
        (63, 1), (39, 2), (41, 1)}
    with pytest.raises(ValueError, match='unsupported class ID'):
        store.add(5, 'map', [1, 2, 3], .9, class_id=40)
    store.close()


def test_manifest_tracks_single_and_multiple_active_classes_without_false_primary(tmp_path):
    path = str(tmp_path / 'active_classes.sqlite3')
    store = SemanticMapStore(path, 'active', active_class_ids=[41])
    manifest = json.loads(store.db.execute(
        "SELECT value FROM metadata WHERE key='manifest'").fetchone()[0])
    assert manifest['semantic_class_id'] == 41
    assert manifest['semantic_class_ids'] == [41]
    assert manifest['active_class_ids'] == [41]
    assert manifest['legacy_primary_class_id'] == 63
    assert manifest['supported_class_ids'] == list(SEMANTIC_CLASS_IDS)
    store.add(10, 'map', [1, 2, 3], .9, class_id=41)
    store.close()

    store = SemanticMapStore(path, 'active', active_class_ids=[39, 41])
    manifest = json.loads(store.db.execute(
        "SELECT value FROM metadata WHERE key='manifest'").fetchone()[0])
    assert manifest['semantic_class_id'] is None
    assert manifest['semantic_class_ids'] == [39, 41]
    assert manifest['active_class_ids'] == [39, 41]
    assert manifest['legacy_primary_class_id'] == 63
    assert len(store.objects()) == 1  # Changing active classes never clears old observations.
    store.close()


def test_legacy_manifest_migrates_fields_and_preserves_unknown_metadata(tmp_path):
    path = tmp_path / 'legacy.sqlite3'
    db = sqlite3.connect(path)
    db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
    db.execute("INSERT INTO metadata VALUES ('map_id', 'legacy')")
    db.execute("INSERT INTO metadata VALUES ('map_version', '1')")
    db.execute("INSERT INTO metadata VALUES ('manifest', ?)", (json.dumps({
        'map_id': 'legacy', 'map_version': 1, 'coordinate_frame': 'map',
        'semantic_class_id': 63, 'custom_field': 'preserved',
    }),))
    db.commit()
    db.close()

    store = SemanticMapStore(str(path), 'legacy', active_class_ids=[39])
    manifest = json.loads(store.db.execute(
        "SELECT value FROM metadata WHERE key='manifest'").fetchone()[0])
    assert manifest['map_id'] == 'legacy'
    assert manifest['custom_field'] == 'preserved'
    assert manifest['semantic_class_id'] == 39
    assert manifest['legacy_primary_class_id'] == 63
    assert manifest['semantic_class_ids'] == [39]
    store.close()


def test_same_stamp_duplicate_and_distinct_boxes_do_not_fake_confirmation(tmp_path):
    store = SemanticMapStore(str(tmp_path / 'same_frame' / 'map.sqlite3'), 'map-c',
                             confirmations=3, association_gate_m=.3)
    first = store.add(100, 'map', [1, 0, 0], .9, payload={'source_frame':'camera'}, detection_index=0)
    duplicate = store.add(100, 'map', [1, 0, 0], .9, payload={}, detection_index=0)
    second_box = store.add(100, 'map', [1.02, 0, 0], .8, payload={}, detection_index=1)
    assert first['count'] == 1 and not first['confirmed']
    assert duplicate['status'] == 'duplicate_detection'
    assert second_box['object_id'] != first['object_id']
    assert second_box['count'] == 1 and not second_box['confirmed']
    assert len(store.objects()) == 2
    assert store.add(200, 'map', [1.01, 0, 0], .9, detection_index=0)['object_id'] == first['object_id']
    confirmed = store.add(300, 'map', [.99, 0, 0], .9, detection_index=0)
    assert confirmed['object_id'] == first['object_id'] and confirmed['confirmed']
    store.close()


def test_store_rejects_invalid_configuration(tmp_path):
    with pytest.raises(ValueError, match='confirmations'):
        SemanticMapStore(str(tmp_path / 'invalid.sqlite3'), 'map', confirmations=0)


def test_store_serializes_cross_thread_reads_and_writes(tmp_path):
    store = SemanticMapStore(str(tmp_path / 'threaded.sqlite3'), 'threaded',
                             confirmations=3, association_gate_m=.5)

    def worker(i):
        result = store.add(i + 1, 'map', [1 + (i % 4) * .01, 0, 0], .9,
                           payload={'source_frame': 'camera'}, detection_index=0)
        snapshot = store.objects()
        return result, snapshot

    with ThreadPoolExecutor(max_workers=6) as pool:
        results = list(pool.map(worker, range(40)))
    assert len(results) == 40
    objects = store.objects()
    assert len(objects) == 1
    assert objects[0]['observations'] == 40
    assert objects[0]['confirmed']
    store.close()


def test_tf_ready_selector_chooses_newest_fresh_pose_and_keeps_pending():
    def pair(frame_id):
        return (SimpleNamespace(header=SimpleNamespace(frame_id=frame_id)), object())
    frames = [(8_000_000_000, pair('older-ready')),
              (9_000_000_000, pair('newer-pending')),
              (7_000_000_000, pair('expired-ready')),
              (10_100_000_000, pair('future'))]
    ready = {'older-ready', 'expired-ready'}
    def has_tf(stamp, frame_pair):
        frame_id = frame_pair[1][0].header.frame_id
        return frame_id in ready
    selected = select_latest_ready_frame(frames, 10_000_000_000, 2.0,
        has_tf)
    assert selected == 0
    ready.add('newer-pending')
    selected = select_latest_ready_frame(frames, 10_000_000_000, 2.0,
        has_tf)
    assert selected == 1
