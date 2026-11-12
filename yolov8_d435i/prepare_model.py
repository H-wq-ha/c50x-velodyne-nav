#!/usr/bin/env python3
"""Download the small COCO YOLOv8 model and export a fixed-size ONNX model."""
import argparse
from pathlib import Path

import onnx
from ultralytics import YOLO
from ultralytics.utils.downloads import attempt_download_asset


ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "models"
WEIGHTS = MODELS / "yolov8n.pt"
BASE_IMGSZ = 256


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imgsz", type=int, default=BASE_IMGSZ,
                        help="Fixed square ONNX input size (smaller is faster but may miss small objects)")
    args = parser.parse_args()
    onnx_path = MODELS / f"yolov8n_{args.imgsz}.onnx"
    MODELS.mkdir(parents=True, exist_ok=True)
    resolved = Path(attempt_download_asset(WEIGHTS)).resolve()
    if resolved != WEIGHTS.resolve():
        resolved.replace(WEIGHTS)
    print(f"PyTorch weights: {WEIGHTS} ({WEIGHTS.stat().st_size / 1e6:.1f} MB)", flush=True)

    model = YOLO(str(WEIGHTS), task="detect")
    exported = Path(model.export(
        format="onnx",
        imgsz=args.imgsz,
        dynamic=False,
        simplify=False,
        opset=12,
        nms=False,
    )).resolve()
    if exported != onnx_path.resolve():
        exported.replace(onnx_path)

    checked = onnx.load(str(onnx_path))
    onnx.checker.check_model(checked)
    print(f"ONNX model: {onnx_path} ({args.imgsz}x{args.imgsz} input, {onnx_path.stat().st_size / 1e6:.1f} MB)", flush=True)
    print(f"Input: {checked.graph.input[0].name}; output: {checked.graph.output[0].name}", flush=True)


if __name__ == "__main__":
    main()
