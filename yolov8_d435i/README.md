# YOLOv8n + Intel RealSense D435i

This standalone demo downloads the COCO-pretrained YOLOv8 nano detector, aligns D435i depth to color, and prints each detection's 3D center in the D435i color optical frame. Fixed 640x640, 416x416, 320x320, and 256x256 ONNX models are included. The live demo defaults to 640x480 color/depth at 60 FPS and the 256x256 model for faster inference; `prepare_model.py` exports a 256x256 model by default.

Coordinates are in meters: **X right, Y down, Z forward**. The reported depth is the median of valid depth pixels near the center of the detection box. These are camera-frame coordinates; transform them into `base_link` with the camera mounting TF if robot-frame coordinates are needed. Smaller inputs trade small-object detail for speed. Use `--model models/yolov8n_416.onnx --imgsz 416` for a balanced option or `--model models/yolov8n_640.onnx --imgsz 640` for higher image detail.

The ONNX model runs on CPU through ONNX Runtime. The included Python environment reuses the machine's PyTorch, Ultralytics, OpenCV, NumPy, and RealSense Python SDK, and installs the ONNX Runtime dependency locally. The default ONNX Runtime thread count is two, measured faster here than one, four, or eight. `--detect-every N` controls recognition cadence: the default `2` reuses the latest detection boxes on alternating frames and refreshes their depth-derived coordinates; set it to `1` for the highest recognition rate. The preview displays both camera-loop and YOLO inference rates.

## Prepare and run

```bash
cd /home/y/arm_car/lidar/yolov8_d435i
source .venv/bin/activate
python prepare_model.py
python detect_realsense.py
```

Press **Q** or **Esc** in the preview to stop. To run without a preview, use `python detect_realsense.py --headless`. Use `--max-seconds 20` for a bounded test, `--confidence 0.5` to reduce weak detections, or `--serial SERIAL` to select another RealSense device. For a higher-detail/slower model, use `--model models/yolov8n_320.onnx --imgsz 320`; use `--detect-every 1` to run inference on each processed frame. To export a different model size, run `python prepare_model.py --imgsz 320` (valid sizes include 256, 320, 416, and 640).

The model recognizes the 80 COCO classes. It cannot label custom objects that were absent from training.
