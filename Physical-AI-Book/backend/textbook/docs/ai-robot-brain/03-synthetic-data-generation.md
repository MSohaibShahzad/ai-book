---
title: "Chapter 3: Synthetic Data Generation"
slug: /ai-robot-brain/synthetic-data-generation
sidebar_label: "3. Synthetic Data Generation"
sidebar_position: 4
toc: true
description: "Master Isaac Sim's Replicator API for automated generation of labeled training data with bounding boxes, segmentation masks, and domain randomization for perception models."
---

# Chapter 3: Synthetic Data Generation

## Introduction

Training modern perception models (YOLO, Mask R-CNN, PointNet++) requires **massive labeled datasets**: 10,000-1,000,000 images with bounding boxes, segmentation masks, or 3D point annotations. Manually labeling this data costs $0.10-$1.00 per image (total: $1K-$1M) and takes weeks to months. Real-world data collection for humanoid-specific tasks (stair climbing, door opening, picking varied household objects) is even harder—many scenarios are rare or dangerous.

**Synthetic data generation** solves this: Isaac Sim's **Replicator API** automatically generates **photorealistic labeled data** overnight. This chapter teaches you to:
1. Use Replicator to create diverse scenes (randomized lighting, objects, poses).
2. Automatically annotate data (bounding boxes, segmentation, depth).
3. Export datasets in standard formats (COCO, KITTI, custom).
4. Validate synthetic data quality (domain gap analysis).

---

## The Synthetic Data Pipeline

### Workflow Overview

```
[Scene Setup] → [Randomization] → [Capture] → [Annotation] → [Export]
```

**1. Scene Setup**: Create base environment (warehouse, kitchen, office) with objects.
**2. Randomization**: Vary lighting, textures, object poses, camera parameters (domain randomization).
**3. Capture**: Render RGB, depth, segmentation images from robot's camera view.
**4. Annotation**: Automatically generate labels (2D bounding boxes, 3D bounding boxes, instance segmentation masks).
**5. Export**: Save as COCO JSON, KITTI format, or custom format for training.

---

## Replicator API Basics

### Core Concepts

**Replicator** is Isaac Sim's Python API for synthetic data generation. Key components:

1. **Randomizer**: Functions that vary scene parameters (`randomize_lighting()`, `randomize_poses()`).
2. **Writer**: Saves data to disk (RGB images, labels, metadata).
3. **Trigger**: Defines when to capture data (every N frames, on event).

### Simple Example: Generate 100 Images

```python
import omni.replicator.core as rep

# 1. Define camera
camera = rep.create.camera(position=(2, 0, 1.5), look_at=(0, 0, 0.5))

# 2. Define randomization
def randomize_scene():
    # Randomize lighting (rotate HDRI map)
    with rep.get.prims(path_pattern="/World/DomeLight"):
        rep.modify.attribute("xformOp:rotateY", rep.distribution.uniform(0, 360))

    # Randomize object positions
    with rep.get.prims(path_pattern="/World/Objects/*"):
        rep.modify.pose(
            position=rep.distribution.uniform((-0.5, -0.5, 0), (0.5, 0.5, 0.5))
        )

# 3. Register randomization trigger
rep.randomizer.register(randomize_scene)

# 4. Attach writer (save RGB + bounding boxes in COCO format)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="/data/synthetic", rgb=True, bounding_box_2d_tight=True)
writer.attach([camera])

# 5. Run for 100 frames (captures 100 randomized images)
rep.orchestrator.run_until_complete(num_frames=100)
```

**Output**:
- `/data/synthetic/rgb/0000.png` through `/data/synthetic/rgb/0099.png`
- `/data/synthetic/bounding_box_2d_tight/0000.json` (COCO format annotations)

---

## Annotation Types

### 1. 2D Bounding Boxes

**Use case**: Object detection (YOLO, Faster R-CNN).

**Data format** (COCO JSON):
```json
{
  "images": [{"id": 0, "file_name": "0000.png", "width": 640, "height": 480}],
  "annotations": [
    {
      "id": 0,
      "image_id": 0,
      "category_id": 1,  # "cup"
      "bbox": [120, 200, 80, 60],  # [x, y, width, height]
      "area": 4800
    }
  ],
  "categories": [{"id": 1, "name": "cup"}, {"id": 2, "name": "plate"}]
}
```

**Replicator API**:
```python
writer = rep.WriterRegistry.get("COCOWriter")
writer.initialize(output_dir="/data/coco", classes=["cup", "plate", "fork"])
```

### 2. Instance Segmentation Masks

**Use case**: Mask R-CNN, semantic segmentation.

**Data**: Per-pixel class labels (image where each pixel value = class ID).

**Replicator API**:
```python
# Render segmentation (each object instance has unique color)
writer.initialize(output_dir="/data/seg", instance_segmentation=True)
```

**Output**: `/data/seg/instance_segmentation/0000.png` (grayscale image: pixel value 1 = object 1, value 2 = object 2).

### 3. Depth Maps

**Use case**: Depth estimation networks (MiDaS, DPT).

**Data**: Distance from camera to surface (meters), stored as 32-bit float PNG.

**Replicator API**:
```python
writer.initialize(output_dir="/data/depth", depth=True)
```

**Output**: `/data/depth/depth/0000.npy` (NumPy array, shape [480, 640], values in meters).

### 4. 3D Bounding Boxes

**Use case**: 3D object detection (PointPillars, SECOND).

**Data**: 3D box center, size, rotation (in camera or world frame).

**Format** (KITTI):
```
# Object class, truncation, occlusion, alpha, 2D bbox, 3D dimensions, 3D location, rotation
Cup 0.0 0 0.0 120 200 200 260 0.1 0.1 0.15 0.5 0.3 0.8 1.57
```

**Replicator API**:
```python
writer = rep.WriterRegistry.get("KittiWriter")
writer.initialize(output_dir="/data/kitti")
```

---

## Domain Randomization Strategies

Goal: Generate **diverse data** to prevent overfitting to simulation artifacts.

### 1. Appearance Randomization

**Lighting**:
```python
def randomize_lighting():
    # Randomize HDRI rotation (time of day)
    dome_light = rep.get.prims(path_pattern="/World/DomeLight")
    rep.modify.attribute(dome_light, "xformOp:rotateY", rep.distribution.uniform(0, 360))

    # Randomize light intensity (cloudy vs. sunny)
    rep.modify.attribute(dome_light, "inputs:intensity", rep.distribution.uniform(800, 1200))
```

**Textures**:
```python
def randomize_textures():
    # Randomize floor material (wood, tile, concrete)
    floor = rep.get.prims(path_pattern="/World/Floor")
    materials = ["/Materials/Wood", "/Materials/Tile", "/Materials/Concrete"]
    rep.randomizer.material(floor, rep.distribution.choice(materials))
```

**Colors**:
```python
def randomize_object_colors():
    objects = rep.get.prims(semantics=[("class", "cup")])
    rep.modify.attribute(objects, "inputs:diffuse_tint",
                         rep.distribution.uniform((0.5, 0.5, 0.5), (1.0, 1.0, 1.0)))
```

### 2. Geometric Randomization

**Object poses**:
```python
def randomize_object_poses():
    objects = rep.get.prims(path_pattern="/World/Objects/*")
    rep.modify.pose(
        objects,
        position=rep.distribution.uniform((-0.5, -0.5, 0), (0.5, 0.5, 0.5)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))  # Yaw only
    )
```

**Occlusions** (place distractors):
```python
def add_distractors():
    # Spawn 5-10 random objects as clutter
    num_distractors = rep.distribution.uniform(5, 10)
    for i in range(int(num_distractors)):
        rep.create.from_usd("/Assets/clutter_object.usd",
                            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 1)))
```

### 3. Sensor Randomization

**Camera intrinsics**:
```python
def randomize_camera():
    camera = rep.get.prims(prim_type="Camera")
    # Randomize focal length (15-50mm equivalent)
    rep.modify.attribute(camera, "focalLength", rep.distribution.uniform(15, 50))
    # Randomize exposure (simulate auto-exposure)
    rep.modify.attribute(camera, "fStop", rep.distribution.uniform(4.0, 16.0))
```

**Lens distortion**:
```python
# Add barrel/pincushion distortion (post-processing in OpenCV after render)
import cv2
k1 = np.random.uniform(-0.3, 0.3)  # Distortion coefficient
image_distorted = cv2.undistort(image, camera_matrix, distCoeffs=[k1, 0, 0, 0])
```

---

## Scaling Data Generation

### Parallel Rendering

Generate 100,000 images in hours, not days:

**Strategy 1: Multi-GPU**
- Run 4 Isaac Sim instances on 4 GPUs (A100 ×4).
- Each generates 25,000 images → total 100,000 images in 6 hours.

**Strategy 2: Cloud Rendering**
- Use NVIDIA Omniverse Cloud (pay-per-use).
- Spin up 100 GPU instances → 100,000 images in 1 hour.

**Code structure**:
```python
# Divide work across GPUs
import os
gpu_id = int(os.environ["CUDA_VISIBLE_DEVICES"])
start_frame = gpu_id * 25000
end_frame = (gpu_id + 1) * 25000

rep.orchestrator.run(start_frame=start_frame, end_frame=end_frame)
```

### Data Efficiency

**How much data is enough?**
- **Object detection (10 classes)**: 10,000 images (1,000 per class).
- **Instance segmentation (20 classes)**: 50,000 images (2,500 per class).
- **Depth estimation**: 100,000 image-depth pairs.

**Diminishing returns**: After 100K images, accuracy gains plateau. Focus on **diversity** (lighting, poses, backgrounds) over raw quantity.

---

## Validation: Does Synthetic Data Work?

### Sim-to-Real Transfer Metrics

**Experiment**: Train YOLOv8 on synthetic data, test on real robot camera.

| Training Data | Validation mAP (Sim) | Test mAP (Real) | Reality Gap |
|---------------|----------------------|-----------------|-------------|
| Real only (10K) | 0.85 | 0.85 | 0% (baseline) |
| Synthetic (basic) | 0.90 | 0.60 | **30% gap** |
| Synthetic + randomization | 0.88 | 0.78 | 10% gap |
| Synthetic + fine-tune (1K real) | 0.87 | 0.85 | 2% gap ✓ |

**Conclusion**: Domain randomization reduces gap from 30% → 10%. Fine-tuning with small real dataset (1K images) closes gap to 2%.

### When Synthetic Data Fails

1. **Fine-grained textures**: Real wood grain, fabric weave are hard to simulate. → Use scanned PBR materials.
2. **Transparent objects**: Glass, water refraction is expensive to ray-trace. → Rasterize depth, use post-processing.
3. **Lighting extremes**: Lens flare, overexposure, HDR effects. → Capture real HDRIs from deployment environment.

---

## Summary

Synthetic data generation with Isaac Sim's Replicator API:

1. **Automates labeling**: Bounding boxes, segmentation, depth (vs. $0.10-$1/image manual labeling).
2. **Scales infinitely**: Generate 100K images overnight (vs. weeks of real-world collection).
3. **Domain randomization**: Vary lighting, textures, poses to reduce sim-to-real gap.

**Key steps**:
1. Set up scene with objects and camera.
2. Register randomizers (lighting, poses, textures).
3. Attach writer (COCO, KITTI, custom format).
4. Run orchestrator (100-100,000 frames).
5. Fine-tune on small real dataset (1K images) for best performance.

**Next chapter**: Deploy trained models on real robots using Isaac ROS (GPU-accelerated inference).

---

**Navigation**
← [Chapter 2: Isaac Sim Photorealism](/ai-robot-brain/isaac-sim-photorealism)
→ [Chapter 4: Isaac ROS Perception](/ai-robot-brain/isaac-ros-perception)
