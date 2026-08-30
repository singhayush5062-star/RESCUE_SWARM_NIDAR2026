# Detection weights

`nidar_person.pt` is the path every detection node loads. It is the **only**
path referenced by `nidar_detection`'s launch file and node defaults, so
swapping models is a file copy — no code, config, or launch change:

```bash
cp /path/to/retrained.pt project_gazebo/models/detection/nidar_person.pt
```

Ultralytics dispatches on what that path points at, so it accepts `.pt`
(PyTorch), `.onnx` (ONNX Runtime), `.engine` (TensorRT), or an ncnn export
**directory** without any change to `detector.py`.

## Current contents

| file | what it is |
|---|---|
| `nidar_person.pt` | active weights — PERSON_DETECTION_MODEL_V3, YOLO26n fine-tuned on MANNEQUIN_PERSON_V3, single class `person` |
| `nidar_person_ncnn_model/` | ncnn export of the same weights (the CPU backend) |
| `nidar_person.onnx` | ONNX export of the same weights, kept for other runtimes |
| `yolo26n.pt` | stock COCO-pretrained YOLO26-nano, kept as a fallback |

V3's own validation figures: precision 0.961, recall 0.950, mAP50 0.974,
mAP50-95 0.917, trained at 640×640.

For contrast, the stock COCO weights this replaced scored a top-down person
at **0.03–0.12 confidence even when the figure filled the frame** — COCO
contains essentially no overhead people, so that was a viewpoint gap, not a
resolution one. V3 scores the same nadir sim survivors at **0.86–0.95**.

## The three exports are the same weights, not three models

`detector.resolve_model_path()` picks between them by device, so nothing
downstream has to care:

| where inference runs | what loads | measured, 640×480 sim frame |
|---|---|---|
| CUDA available | `nidar_person.pt` | 13.2 ms (76 FPS) |
| CPU only | `nidar_person_ncnn_model/` | 122.2 ms (8.2 FPS) |

PyTorch on CPU is 379 ms (2.6 FPS) at the same input size, so on a machine
with no usable GPU the ncnn export is **3.1× faster for identical output** —
which is why it is picked automatically rather than offered as an option.
That is also the backend the competition companion computer will run, so it
is worth testing deliberately even on a box that has a GPU:

```bash
NIDAR_DETECTION_DEVICE=cpu ./scripts/run_simulation.sh
```

Set the node parameter `prefer_ncnn_on_cpu:=false` to force the literal
`model_path` regardless of device.

## Re-exporting after retraining

Dropping in a new `.pt` alone works — it just runs on the slower backend on
CPU, and the node logs a warning saying so. To keep the fast CPU path, export
alongside it:

```bash
yolo export model=project_gazebo/models/detection/nidar_person.pt format=ncnn
```

That writes `nidar_person_ncnn_model/` right beside the weights, which is
exactly the name `resolve_model_path()` looks for.

Loading an ncnn export needs the `ncnn` runtime, which is not a rosdep
dependency and is not installed by `scripts/setup_nidar_ros.sh` (that script
sets up the environment, it does not install packages):

```bash
python3 -m pip install --user ncnn
```

Without it the node falls back to the `.pt` and logs the slow-CPU warning.
