# Detection weights

`nidar_person.pt` is the weights file every detection node loads. It is the
**only** path referenced by `nidar_detection`'s launch file and node
defaults, so swapping models is a file copy — no code, config, or launch
change:

```bash
cp /path/to/retrained.pt project_gazebo/models/detection/nidar_person.pt
```

Ultralytics dispatches on the file extension, so this path accepts `.pt`
(PyTorch, what sim uses), `.onnx` (ONNX Runtime), or `.engine` (TensorRT,
the Phase 8 Jetson target) without any change to `detector.py`.

## Current contents

| file | what it is |
|---|---|
| `nidar_person.pt` | active weights — currently a copy of `yolo26n.pt` |
| `yolo26n.pt` | stock COCO-pretrained YOLO26-nano, kept as a fallback |

## Why the stock weights are a placeholder, not a solution

Measured live against this simulator's nadir camera (see
`DOCUMENTS/Phase2_Detection_Notes.md`): stock COCO weights score a
top-down person at **0.03–0.12 confidence even when the figure fills the
frame**. COCO contains essentially no overhead people, so this is a
viewpoint gap, not a resolution one — flying lower does not fix it.

Stock weights are therefore only good enough to prove the *pipeline*
(camera → inference → `DetectionResult` → GCS) runs end to end. Real
detection needs weights retrained on overhead human imagery. Drop those in
at `nidar_person.pt` and the whole pipeline picks them up.
