# D435i + YOLOv8n 快速测试

- Camera: Intel RealSense D435i, serial `327122073765`, firmware `5.17.0.10`
- All tests used aligned 640×480 color and depth streams configured at 60 Hz, CPU ONNX Runtime, and a bounded live run.
- 640×640 input: 8.92 inference FPS at a 30 Hz camera configuration.
- 416×416 input: 14.33 inference FPS.
- 320×320 input: 17.32 inference FPS.
- 256×256 input, one inference per frame: 21.44 FPS with four ONNX threads; 23.76 FPS with two threads; 23.50 FPS with one thread; 18.71 FPS with eight threads. The two-thread setting is the best measured choice. The live scene consistently produced person and laptop detections with valid depth/XYZ estimates.
- 256×256 input, inference every other processed frame: 34.12 camera-loop FPS and 17.11 YOLO inferences per second. This is the default cadence for a smoother live preview; use `--detect-every 1` for maximum recognition frequency.

Coordinates are in the color optical frame (X right, Y down, Z forward). Depth is the median of valid aligned depth pixels in the center region of each detection box. The camera stream is configured at 60 Hz; application frame-loop FPS includes alignment, inference scheduling, and drawing.
