# Runtime Models

`tools/train_detector.py` writes:

- `ball_cone.onnx`: end-to-end YOLO26 detection graph for ONNX Runtime.
- `ball_cone.json`: input dimensions, class map, format, and model checksum.

The generated files are intentionally not committed until a reviewed model has
been trained. The robot keeps using the classical detector unless
`VISION_BACKEND=ml` is set.
