# Ball And Cone ML Workflow

This workflow replaces only ball/cone proposals. Horizon gating, IMU
compensation, temporal tracking, target priority, drive smoothing, actuator
gating, and the classical detector remain available.

The runtime defaults to `VISION_BACKEND=classical`. Nothing in this workflow
enables actuator output.

## Licensing Check

Ultralytics states that private use, embedded robotics, and custom-trained
models require either releasing the complete project under AGPL-3.0 or
obtaining an Enterprise license:

https://www.ultralytics.com/license

This project owner has said licensing is already handled. Confirm the
school/program's chosen license before using the trained model outside this
experiment. Keeping the repository private does not by itself satisfy the
AGPL option described by Ultralytics.

## 1. Collect Frames On The Pi

No ML package is needed for capture:

```bash
cd ~/Coding/bobot
git pull
source .venv/bin/activate
ROBOT_START_STATE=capture CAPTURE_SESSION=course-lighting-01 python3 main.py
```

Capture mode is always motor-neutral. Hold Y and point either stick down to
select it from the radial menu, or start there with `ROBOT_START_STATE`.
Press X for each snapshot. The default X event code is `308`; override it if
the controller reports another code:

```bash
CONTROLLER_CAPTURE_BUTTON=308
```

Captured files are written under:

```text
datasets/captures/<session>/frame_<timestamp>.jpg
datasets/captures/<session>/metadata.jsonl
```

The JPG is the corrected camera frame before any horizon or detection overlay.
The JSONL record includes IMU attitude and horizon context.

Collect at least three sessions; five to ten is better. Restart the runtime
with a new `CAPTURE_SESSION` for each location, lighting condition, or test
arrangement. A useful first model usually needs a few hundred deliberately
varied images, not thousands of nearly identical burst frames.

Include:

- Every ball color, near and far, singly and in piles.
- Partial balls at image edges and balls partly hidden behind other objects.
- Cones at every useful distance and angle.
- Bright, dim, gradient, and shadowed lighting.
- Wood, yellow planks, people, clothing, buckets, tools, and other historical
  false positives.
- Frames with no ball or cone. These become negative examples.

Do not capture only easy centered objects. Move the robot and camera through
the poses it will actually encounter.

## 2. Transfer Captures To The Laptop

`datasets/` is intentionally ignored by Git. Transfer it over the local
network, Tailscale, USB storage, or Raspberry Pi Connect. For SSH/rsync:

```bash
cd ~/Coding/bobot
rsync -av pi@PI_ADDRESS:~/Coding/bobot/datasets/captures/ datasets/captures/
```

Do not force hundreds of training images into the normal Git history.

## 3. Create A Laptop Training Environment

The reliable Arch path is CPU training:

```bash
cd ~/Coding/bobot
python3 -m venv .venv-train
source .venv-train/bin/activate
python -m pip install --upgrade pip
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements-train.txt
```

The Ryzen AI 9 HX 370 CPU can train the small `yolo26n` model. It will be
slower than a supported GPU setup but requires much less system work.

### Optional Radeon 890M Training

The training script automatically selects device `0` when
`torch.cuda.is_available()` is true; ROCm exposes its GPU through PyTorch's
`torch.cuda` API.

AMD's current production support for the HX 370 lists Ubuntu 24.04.3,
ROCm 7.2, Python 3.12, and PyTorch 2.9. Arch is not in that support matrix.
AMD also currently lists native Windows ML training as unsupported. Do not
replace a working Arch install merely to accelerate the first experiment.

Use AMD's current instructions if CPU time becomes a real bottleneck:

https://rocm.docs.amd.com/projects/radeon-ryzen/en/docs-7.2/docs/compatibility/compatibilityryz/native_linux/native_linux_compatibility.html

https://rocm.docs.amd.com/projects/radeon-ryzen/en/latest/docs/install/installryz/native_linux/install-pytorch.html

Verify a ROCm environment before installing the remaining requirements:

```bash
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

## 4. Annotate Every Frame

Run:

```bash
python tools/annotate.py
```

Controls:

- `1`: boxes drawn next are balls.
- `2`: boxes drawn next are cones.
- Left-drag: add a box.
- Right-click: delete the containing or nearest box.
- `u`: remove the most recently added box.
- `n`, Enter, or Space: save and move forward.
- `p`: save and move backward.
- `q`: save and exit.

The tool starts at the first image without a label file. Moving past a frame
with no boxes writes an empty label file, which intentionally marks a reviewed
negative.

Annotation rules:

- Label every visible ball and cone, not only the intended target.
- Draw a tight box around the visible object.
- Label distinguishable touching balls separately.
- For a severely occluded object that cannot be classified confidently, leave
  it unlabeled rather than guessing.
- Never label a false positive as a ball merely to make it disappear; leave it
  as background so the model learns the distinction.

Labels use normalized YOLO rows:

```text
class_id x_center y_center width height
```

Class `0` is ball and class `1` is cone. Labels are stored under
`datasets/labels/` with the same session and filename layout as the captures.

## 5. Validate And Split The Dataset

```bash
python tools/prepare_dataset.py
```

If rebuilding an existing generated split:

```bash
python tools/prepare_dataset.py --replace
```

The command refuses to continue when an image has not been reviewed or a
label is malformed. With at least three capture sessions, complete sessions
are assigned to train, validation, or test so adjacent near-duplicate frames
do not leak across the evaluation boundary.

Generated files:

```text
datasets/ball_cone/data.yaml
datasets/ball_cone/manifest.json
datasets/ball_cone/images/{train,val,test}/
datasets/ball_cone/labels/{train,val,test}/
```

Review the split counts printed by the tool. Try to keep capture sessions
similar in size; a single huge session makes session-level balancing difficult.

## 6. Train, Evaluate, And Export

Start with:

```bash
python tools/train_detector.py --epochs 100 --imgsz 512 --batch 8
```

The script:

1. Fine-tunes `yolo26n.pt`.
2. Uses moderate brightness, saturation, scale, translation, rotation, mosaic,
   and horizontal-flip augmentation.
3. Evaluates the best checkpoint against the held-out test split.
4. Exports a fixed-size end-to-end ONNX graph.
5. Writes a checksum and runtime settings manifest.

Outputs:

```text
runs/ball_cone/train/
models/ball_cone.onnx
models/ball_cone.json
```

If memory is a problem, reduce `--batch` to `4` or `2`. If far-away balls are
still weak, try `--imgsz 640`; expect slower Pi inference. Resume an interrupted
run using its last checkpoint:

```bash
python tools/train_detector.py --resume runs/ball_cone/train/weights/last.pt
```

Training configuration is visible in `tools/train_detector.py`; commit tuning
changes so every model can be reproduced.

## 7. Commit And Deploy The Model

The model files are not ignored:

```bash
git add models/ball_cone.onnx models/ball_cone.json
git commit -m "Add trained ball and cone detector"
git push
```

On the Pi:

```bash
cd ~/Coding/bobot
git pull
source .venv/bin/activate
python -m pip install -r requirements-ml-pi.txt
ENABLE_ACTUATORS=false ROBOT_START_STATE=detector VISION_BACKEND=ml python3 main.py
```

The Pi does not need PyTorch or Ultralytics. It runs only `onnxruntime`.

The detector TUI reports:

- Backend and load/error state.
- Inference latency and effective inference rate.
- Age of the latest result.
- Frames replaced in the one-frame inference queue.
- Current model result sequence.
- Tracked and IMU-predicted candidates.

Inference runs on a separate latest-frame worker. Dropped inference frames are
normal when the camera is faster than the model. The worker always processes
the newest pending frame. Results older than `ML_MAX_RESULT_AGE` are discarded,
and only currently observed, temporally confirmed balls may acquire a new
steering target.

Keep actuators disabled until the model has been tested across the course.
Then enable detector throttle using the existing two-stage safety process:
`ENABLE_ACTUATORS=true` at startup and A after entering detector mode.

## 8. Runtime Tuning

Defaults:

```bash
VISION_BACKEND=ml
ML_MODEL_PATH=models/ball_cone.onnx
ML_MODEL_MANIFEST=models/ball_cone.json
ML_VERIFY_MODEL_HASH=true
ML_BALL_CONFIDENCE=0.35
ML_CONE_CONFIDENCE=0.35
ML_CERTAIN_CONFIDENCE=0.55
ML_MIN_BOX_AREA_RATIO=0.00002
ML_MAX_BOX_AREA_RATIO=0.45
ML_MAX_DETECTIONS=100
ML_INTRA_THREADS=4
ML_INTER_THREADS=1
ML_MAX_RESULT_AGE=0.75
```

Raise the class confidence threshold to reduce false positives. Lower it to
recover missed objects, but prefer collecting and retraining on failure cases
over forcing a poor model with a very low threshold. `ML_CERTAIN_CONFIDENCE`
controls whether a persistent track becomes a solid circle/triangle.

`ML_MAX_BOX_AREA_RATIO` is a final guard against screen-sized detections.
Horizon allowances still come from `HORIZON_BALL_ALLOWANCE_RATIO` and
`HORIZON_CONE_ALLOWANCE_RATIO`.

## 9. Active-Learning Loop

After each course test:

1. Enter capture mode and save frames containing misses and false positives.
2. Give the new run a distinct `CAPTURE_SESSION`.
3. Transfer and annotate the new session.
4. Rebuild the dataset with `--replace`.
5. Train a new named run, for example `--name train-02`.
6. Compare held-out test results and real course behavior.
7. Commit the model only when it is better.

Keep some course arrangements and lighting sessions out of training. A model
that memorizes one floor layout can report excellent validation numbers and
still fail after the cones move.

## 10. Rollback

Immediate runtime rollback needs no Git operation:

```bash
VISION_BACKEND=classical python3 main.py
```

The implementation is divided into independent commits:

- Motor-neutral frame capture.
- Laptop annotation/training/export tools.
- Asynchronous ONNX runtime.
- Documentation.

Use `git revert` from newest to oldest if a code-level rollback is needed. Raw
captures are unaffected because `datasets/` is ignored.

## 11. When To Consider An AI HAT

Do not buy one before measuring the trained model on the Pi. Ultralytics'
published Pi 5 result for YOLO26n ONNX at 640 input is around 128 ms per image,
excluding pre/post-processing. This project uses 512 by default and tolerates a
lower model rate by tracking between results.

Consider acceleration if actual TUI measurements show that:

- Inference is consistently too slow to react safely.
- Model CPU load degrades camera, IMU, or control timing.
- Accuracy requires a substantially larger input/model.

For this vision-only use, the Raspberry Pi AI HAT+ 13 TOPS is the sensible
first Hailo option; the AI HAT+ 2 is aimed partly at generative models and is
unnecessary here. A Hailo device is not a drop-in ONNX Runtime accelerator.
It requires compiling a supported model to Hailo's format and adding a new
runtime backend. The current `DetectorState` and tracker interfaces were kept
separate so that backend can be added later without rewriting autonomy.

References:

- https://docs.ultralytics.com/guides/end2end-detection
- https://docs.ultralytics.com/guides/yolo-data-augmentation
- https://docs.ultralytics.com/guides/raspberry-pi
- https://onnxruntime.ai/docs/get-started/with-python.html
- https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html
