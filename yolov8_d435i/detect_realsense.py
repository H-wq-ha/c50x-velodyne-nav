#!/usr/bin/env python3
"""Run live YOLO ONNX detection and print each object's D435i camera XYZ."""
from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/yolov8n_256.onnx")
    parser.add_argument("--serial", default="327122073765", help="D435i serial; empty selects the first camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--imgsz", type=int, default=256, help="Must match the fixed ONNX model input size")
    parser.add_argument("--ort-threads", type=int, default=2,
                        help="ONNX Runtime CPU threads; 1-4 often avoids CPU oversubscription")
    parser.add_argument("--detect-every", type=int, default=2,
                        help="Run detection every N camera frames; 1 gives the highest recognition rate")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--max-seconds", type=float, default=0, help="0 runs until Q/Esc or Ctrl+C")
    parser.add_argument("--headless", action="store_true", help="Do not open the preview window")
    return parser.parse_args()


def object_depth(depth_image: np.ndarray, box: np.ndarray) -> tuple[float, int] | None:
    """Robust median depth from the central part of a detection box, in metres."""
    h, w = depth_image.shape
    x1, y1, x2, y2 = np.rint(box).astype(int)
    x1, x2 = np.clip([x1, x2], 0, w)
    y1, y2 = np.clip([y1, y2], 0, h)
    bw, bh = max(1, x2 - x1), max(1, y2 - y1)
    # Ignore the box edges, which are more likely to include background pixels.
    xa, xb = max(x1, x1 + int(bw * 0.30)), min(x2, x1 + int(bw * 0.70))
    ya, yb = max(y1, y1 + int(bh * 0.30)), min(y2, y1 + int(bh * 0.70))
    roi = depth_image[ya:yb, xa:xb]
    valid = roi[(roi > 0) & (roi < 8000)]
    if valid.size < 5:
        # Small objects can have a very small central area; try a 5x5 center patch.
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        roi = depth_image[max(0, cy - 2):min(h, cy + 3), max(0, cx - 2):min(w, cx + 3)]
        valid = roi[(roi > 0) & (roi < 8000)]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0, int(valid.size)


def main() -> None:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {model_path}; run prepare_model.py first")

    if args.ort_threads < 1:
        raise ValueError("--ort-threads must be at least 1")
    if args.detect_every < 1:
        raise ValueError("--detect-every must be at least 1")

    # Limit CPU thread fan-out so camera capture and preview remain responsive.
    torch.set_num_threads(min(4, torch.get_num_threads()))
    import onnxruntime as ort
    from ultralytics.nn.backends.onnx import ONNXBackend

    # Pass explicit thread-pool settings to the ONNX backend used by Ultralytics.
    original_onnx_init = ONNXBackend.__init__

    def configured_onnx_init(self, weight, device, fp16=False, format="onnx", session_options=None):
        if session_options is None:
            session_options = ort.SessionOptions()
            session_options.intra_op_num_threads = args.ort_threads
            session_options.inter_op_num_threads = 1
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        original_onnx_init(self, weight, device, fp16=fp16, format=format,
                           session_options=session_options)

    ONNXBackend.__init__ = configured_onnx_init
    model = YOLO(str(model_path), task="detect")
    print(f"Model: {model_path}; ONNX Runtime CPU threads={args.ort_threads}", flush=True)

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    align = rs.align(rs.stream.color)
    profile = None
    frame_count = 0
    inference_count = 0
    detected_count = 0
    started = time.monotonic()
    last_empty_notice = 0.0
    cached_boxes = np.empty((0, 4), dtype=np.float32)
    cached_class_ids = np.empty((0,), dtype=np.int32)
    cached_confidences = np.empty((0,), dtype=np.float32)
    names = model.names

    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        print(
            f"Camera: {device.get_info(rs.camera_info.name)} "
            f"serial={device.get_info(rs.camera_info.serial_number)}; "
            f"color/depth={args.width}x{args.height}@{args.fps}; ONNX input={args.imgsz}",
            flush=True,
        )
        print("Coordinates: color optical frame; X right, Y down, Z forward; units=m", flush=True)
        if not args.headless:
            cv2.namedWindow("YOLOv8n + D435i", cv2.WINDOW_NORMAL)

        # Warm up the sensor and model before measuring live processing.
        for _ in range(5):
            frames = pipeline.wait_for_frames(timeout_ms=5000)
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        if not color_frame:
            raise RuntimeError("D435i did not return a color frame")
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            frame_count += 1
            run_inference = (frame_count - 1) % args.detect_every == 0
            if run_inference:
                results = model.predict(
                    source=color,
                    imgsz=args.imgsz,
                    conf=args.confidence,
                    device="cpu",
                    verbose=False,
                    max_det=50,
                )
                result = results[0]
                names = result.names
                if result.boxes is not None and len(result.boxes):
                    cached_boxes = result.boxes.xyxy.cpu().numpy()
                    cached_class_ids = result.boxes.cls.cpu().numpy().astype(int)
                    cached_confidences = result.boxes.conf.cpu().numpy()
                else:
                    cached_boxes = np.empty((0, 4), dtype=np.float32)
                    cached_class_ids = np.empty((0,), dtype=np.int32)
                    cached_confidences = np.empty((0,), dtype=np.float32)
                inference_count += 1

            overlay = color.copy()
            found = 0
            frame_messages = []

            if len(cached_boxes):
                for box, class_id, confidence in zip(cached_boxes, cached_class_ids, cached_confidences):
                    estimate = object_depth(depth, box)
                    x1, y1, x2, y2 = np.rint(box).astype(int)
                    class_name = names[int(class_id)]
                    if estimate is None:
                        text = f"{class_name} {confidence:.2f} | depth unavailable"
                        frame_messages.append(f"{datetime.now().strftime('%H:%M:%S')} {text}")
                    else:
                        z, samples = estimate
                        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                        xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], z)
                        x, y, z = map(float, xyz)
                        found += 1
                        text = f"{class_name} {confidence:.2f} X{x:+.2f} Y{y:+.2f} Z{z:.2f}m"
                        frame_messages.append(
                            f"{datetime.now().strftime('%H:%M:%S')} {class_name:<16} "
                            f"conf={confidence:.2f} XYZ=({x:+.3f}, {y:+.3f}, {z:.3f}) m "
                            f"depth_samples={samples}"
                        )
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(overlay, text, (x1, max(22, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if run_inference and frame_messages:
                print("\n".join(frame_messages), flush=True)

            if run_inference:
                detected_count += found
            elapsed = max(time.monotonic() - started, 1e-6)
            fps_text = (f"Camera loop: {frame_count / elapsed:.1f} | "
                        f"YOLO: {inference_count / elapsed:.1f} FPS | detections: {found}")
            cv2.putText(overlay, fps_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 220, 255), 2)
            if run_inference and found == 0 and time.monotonic() - last_empty_notice >= 2.0:
                print("当前画面未检测到有有效深度的 COCO 物体。", flush=True)
                last_empty_notice = time.monotonic()

            if not args.headless:
                cv2.imshow("YOLOv8n + D435i", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            if args.max_seconds > 0 and time.monotonic() - started >= args.max_seconds:
                break

    except KeyboardInterrupt:
        print("Stopped by user.", flush=True)
    finally:
        if profile is not None:
            pipeline.stop()
        if not args.headless:
            cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - started, 1e-6)
    print(f"Finished: {frame_count} camera frames, {frame_count / elapsed:.2f} frame-loop FPS, "
          f"{inference_count} YOLO inferences, {inference_count / elapsed:.2f} inference FPS, "
          f"{detected_count} valid object-coordinate estimates.", flush=True)


if __name__ == "__main__":
    main()
