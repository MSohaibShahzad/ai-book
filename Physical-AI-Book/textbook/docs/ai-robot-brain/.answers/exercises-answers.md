---
title: "Module 3: Instructor Solutions"
sidebar_label: "Instructor Solutions"
description: "Complete solutions, grading rubrics, and common mistakes for Module 3 exercises (GPU-Accelerated Perception & Isaac)"
---

# Module 3: Instructor Solutions (GPU-Accelerated Perception & Isaac)

**Note**: This file is intended for instructors only and should be excluded from student-facing materials.

---

## 📘 Recall Exercises

### Exercise 1: GPU vs. CPU Performance Analysis

**Difficulty**: Easy | **Time**: 25 min

#### Complete Solution

| **Aspect** | **Gazebo (16-core CPU)** | **Isaac Sim (RTX 4090 GPU)** |
|------------|--------------------------|------------------------------|
| **Physics rate (Hz)** | 500-1000 Hz (single robot) | 10,000+ Hz (with GPU acceleration) |
| **Rendering (1080p)** | 30-60 FPS (rasterization) | 60-120 FPS (ray tracing) |
| **LiDAR update (64×1024)** | 5-10 Hz (CPU raycasting) | 60+ Hz (GPU raycasting) |
| **Parallel robots** | 1-4 robots (multithreading limited) | 100-1000+ robots (massive GPU parallelism) |
| **Photorealism** | Limited (basic shading, no ray tracing) | High (RTX ray tracing, global illumination, reflections) |
| **Hardware cost** | $500-1500 (high-end CPU) | $1500-2000 (RTX 4090) |
| **Primary use case** | Simple physics testing, low-fidelity prototyping | RL training, synthetic data generation, photorealistic HRI |

**Justifications**:
- **Physics rate**: GPUs excel at parallel constraint solving (joint dynamics, collision detection) across multiple robots
- **Rendering**: RTX cores accelerate ray tracing for realistic lighting; Gazebo uses traditional rasterization
- **LiDAR**: GPU raycasting processes thousands of rays in parallel vs. CPU sequential processing
- **Parallel robots**: GPUs can simulate 100+ robots simultaneously for RL training; CPUs bottleneck at 4-8 robots
- **Photorealism**: Isaac Sim uses RTX for global illumination, reflections, refractions; Gazebo uses simplified shaders
- **Hardware cost**: RTX 4090 is more expensive but enables capabilities impossible with CPU alone
- **Use cases**: Gazebo for basic ROS testing; Isaac Sim for sim-to-real transfer, perception training, HRI studies

#### Grading Rubric (Total: 10 points)
- **Physics rate** (1 pt): Correct Hz values (±20% tolerance), mentions GPU parallelism
- **Rendering** (1 pt): FPS values reasonable, distinguishes ray tracing vs. rasterization
- **LiDAR update** (1 pt): GPU advantage for parallel raycasting
- **Parallel robots** (2 pts): Understands massive parallelism for RL (100+ robots)
- **Photorealism** (1 pt): Mentions ray tracing features (global illumination, reflections)
- **Hardware cost** (1 pt): Realistic prices (within $500 of actual costs)
- **Use cases** (3 pts): Clearly differentiates when to use each tool (Gazebo for simple testing, Isaac Sim for perception/RL)

#### Common Mistakes
- Claiming Gazebo is "always better" for robotics (ignores RL training, synthetic data needs)
- Underestimating GPU parallelism (thinking 10-20 robots vs. 100-1000+ possible)
- Confusing rendering FPS with physics rate (they're independent)
- Not justifying use cases (vague answers like "Isaac Sim is newer")

#### Extensions
- Benchmark both simulators on the same laptop (measure actual FPS, CPU/GPU usage)
- Research NVIDIA Omniverse Farm for cloud-scale simulation (10,000+ robots)
- Compare memory usage (VRAM for Isaac Sim vs. RAM for Gazebo)

---

### Exercise 2: Ray Tracing for Robotics

**Difficulty**: Easy | **Time**: 20 min

#### Complete Solution (250 words)

Ray tracing provides three critical rendering features absent in rasterization:

1. **Global illumination**: Simulates light bouncing between surfaces, creating realistic indirect lighting (e.g., red light reflecting from a table onto a white wall). Rasterization only computes direct lighting.
2. **Accurate reflections and refractions**: Models mirrors, glass, and metallic surfaces physically correctly. Rasterization uses approximations (cubemaps, screen-space reflections).
3. **Physically accurate shadows**: Handles soft shadows (penumbra from area lights) and complex occlusions. Rasterization uses shadow maps with artifacts at edges.

**Two robotics use cases where photorealism is critical**:

1. **Sim-to-real transfer for vision models**: Perception networks trained on rasterized images fail on real robots due to unrealistic lighting and textures. Ray tracing generates training data that matches real-world camera observations, reducing domain gap. For example, a grasping policy trained with ray-traced depth images generalizes better because shadows and reflections match real sensors.
2. **Human-robot interaction studies**: HRI researchers need realistic humanoid avatars for user studies (e.g., testing human comfort with robot expressions). Ray-traced faces with subsurface scattering (skin translucency) and realistic eye reflections are essential for valid psychological experiments.

**One legitimate drawback**: Ray tracing is computationally expensive. Even with RTX GPUs, rendering complex scenes at 60 FPS requires high-end hardware ($1500+ GPU), limiting accessibility for students and small labs.

**When Gazebo is sufficient**: For purely algorithmic research (e.g., testing a motion planning algorithm), visual fidelity doesn't matter. Gazebo's fast rasterization is ideal for rapid iteration on control logic without perception components.

#### Grading Rubric (Total: 10 points)
- **Ray tracing features** (3 pts): 3 technically accurate features (global illumination, reflections/refractions, shadows)
- **Use case 1** (2 pts): Explains why photorealism matters (e.g., reduces sim-to-real gap for vision models)
- **Use case 2** (2 pts): Specific scenario where realism is critical (HRI, training data quality)
- **Drawback** (1 pt): Recognizes computational cost or accessibility issues
- **Gazebo use case** (1 pt): Identifies scenario where simple rendering suffices (algorithmic testing, no perception)
- **Word count** (1 pt): Within 200-300 words (deduct if >350 or <200)

#### Common Mistakes
- Listing generic "better graphics" without technical details (global illumination, etc.)
- Vague use cases like "vision models work better" without explaining why
- Ignoring computational cost drawback (ray tracing isn't always better)
- Dismissing Gazebo entirely (it has legitimate use cases)

---

### Exercise 3: Domain Randomization Strategy

**Difficulty**: Moderate | **Time**: 30 min

#### Complete Solution

| **Randomization Parameter** | **Range/Options** | **Justification** |
|-----------------------------|-------------------|-------------------|
| **Lighting (dome light rotation)** | 0-360° (uniform), intensity 500-2000 lux | Prevents overfitting to specific shadows/highlights; real kitchens have varying natural/artificial light throughout the day |
| **Object texture (mug material)** | 10-20 materials (ceramic, plastic, metal, glass) with randomized colors (HSV: H=0-360°, S=0.3-1.0, V=0.4-1.0) | Real mugs have diverse appearances; randomizing prevents the model from memorizing specific textures instead of learning shape |
| **Object pose (X, Y, Z position)** | X: -0.3 to 0.3 m, Y: -0.2 to 0.2 m, Z: table height ± 0.05 m (uniform) | Covers typical placement variation on tables; prevents overfitting to centered objects |
| **Camera focal length** | 35-65 mm equivalent (realistic range for robot head cameras) | Accounts for calibration errors and different camera models; prevents reliance on specific perspective distortion |
| **Background clutter (# objects)** | 5-20 objects (randomized from kitchen props: bowls, utensils, bottles) | Real environments are cluttered; forces model to handle occlusions and distinguish target from distractors |
| **Surface reflectivity** | Table roughness: 0.1-0.8 (0=mirror, 1=matte); randomize specular highlights | Prevents overfitting to specific lighting reflections; real tables range from glossy wood to matte plastic |

#### Grading Rubric (Total: 10 points)
- **Lighting** (1.5 pts): Realistic range (0-360°), justifies shadow/highlight variation
- **Texture** (2 pts): Multiple materials + color randomization, explains overfitting prevention
- **Pose** (1.5 pts): Reasonable spatial bounds (within arm's reach), covers realistic placement
- **Camera** (1 pt): Focal length range is realistic (30-70 mm equivalent)
- **Clutter** (2 pts): Adequate object count (5-20), justifies occlusion handling
- **Reflectivity** (1 pt): Surface property randomization, mentions specular highlights
- **Justifications** (1 pt): Each row explains sim-to-real benefit (not vague)

#### Common Mistakes
- Extreme ranges (e.g., "Lighting 0-10,000 lux" includes unrealistic scenarios)
- Missing color randomization (only varying materials, not colors)
- Vague justifications like "makes it more realistic" (should explain specific failure modes prevented)
- Only randomizing appearance (ignoring geometry, sensor parameters)

#### Extensions
- Add motion blur randomization (robot movement during capture)
- Include depth sensor noise models (flying pixels at edges, depth-dependent error)
- Design an ablation study: train with/without each randomization parameter, measure mAP impact

---

### Exercise 4: TensorRT Optimization

**Difficulty**: Moderate | **Time**: 25 min

#### Complete Solution

**1. What TensorRT does** (3 optimizations):

- **Layer fusion**: Combines sequential operations (e.g., convolution + batch norm + ReLU) into a single GPU kernel, reducing memory bandwidth and kernel launch overhead.
- **Precision calibration (INT8 quantization)**: Converts FP32 weights to 8-bit integers using calibration data, reducing memory by 4× and leveraging Tensor Cores for faster arithmetic.
- **Kernel auto-tuning**: Profiles hundreds of CUDA kernel implementations for each layer and selects the fastest for the target GPU architecture (e.g., Ampere vs. Hopper).

**2. Performance impact**:

If YOLOv8 runs at 5 FPS (CPU) and 15 FPS (GPU PyTorch), TensorRT FP16 should achieve approximately **40-60 FPS** (2.5-4× speedup over PyTorch GPU). Justification: TensorRT eliminates Python overhead, fuses layers, and uses optimized kernels. INT8 quantization could push this to 80-100 FPS but may reduce mAP by 1-2%.

**3. Trade-off**:

TensorRT engines are **hardware-specific**: An engine built for RTX 4090 won't run on Jetson Orin (different GPU architecture). This reduces portability compared to ONNX/PyTorch models. Additionally, INT8 quantization can reduce detection accuracy (mAP) by 0.5-2% if calibration data doesn't cover the target domain.

**4. When to use**:

- **Essential**: Real-time robotics (e.g., humanoid obstacle avoidance at 30 FPS on Jetson Orin). Without TensorRT, PyTorch may only achieve 8-12 FPS, too slow for reactive control.
- **Unnecessary**: Offline batch processing (e.g., annotating a 10,000-image dataset overnight). PyTorch is sufficient; optimization effort isn't justified.

#### Grading Rubric (Total: 10 points)
- **Optimizations** (3 pts): 3 technically correct optimizations (layer fusion, quantization, kernel tuning)
- **FPS estimate** (2 pts): Reasonable speedup (2-5×), justifies with specific TensorRT features
- **Trade-off** (2 pts): Identifies hardware-specificity or accuracy loss, explains impact
- **Essential scenario** (1.5 pts): Real-time constraint (30+ FPS) where TensorRT is critical
- **Unnecessary scenario** (1.5 pts): Offline task where optimization doesn't matter

#### Common Mistakes
- Claiming 100× speedup (unrealistic; typical is 2-5× over PyTorch GPU)
- Only mentioning "makes it faster" without technical details (layer fusion, etc.)
- Ignoring hardware-specificity trade-off (TensorRT engines aren't portable)
- Using same scenario for both "essential" and "unnecessary" (should be opposites)

---

### Exercise 5: Visual SLAM Principles

**Difficulty**: Moderate | **Time**: 30 min

#### Complete Solution

**1. Why vSLAM?**

Humanoid robots can't rely on IMU alone because **IMU drift accumulates quadratically** over distance due to integration errors. For example, a 1% heading error becomes 10 cm lateral error after 10 m of walking. Unlike wheeled robots with odometry, bipedal locomotion has no direct position measurement. vSLAM provides absolute position estimates by matching visual features, correcting IMU drift continuously.

**2. Pipeline stages**:

1. **Feature extraction**: Detects and describes visual keypoints (e.g., ORB features) in each camera frame to create a sparse representation of the scene.
2. **Frame matching**: Matches features between consecutive frames to estimate relative camera motion (visual odometry) using techniques like RANSAC.
3. **Pose estimation**: Computes the robot's 6-DOF pose (position + orientation) by triangulating matched features and minimizing reprojection error.
4. **Map building**: Stores 3D feature positions in a global map, enabling re-localization and loop closure detection.
5. **Loop closure**: Detects when the robot revisits a previously seen location, then optimizes the entire trajectory to eliminate accumulated drift (global pose graph optimization).

**3. Loop closure**:

Loop closure is the process of recognizing a previously visited location and correcting drift by aligning the current pose with the map. It's critical for long-duration navigation (e.g., 30-minute patrol) because without it, accumulated drift would cause the estimated position to diverge from reality by meters, leading to navigation failures.

**4. Failure modes**:

1. **Low-texture environments** (e.g., white walls, empty hallways): Few visual features can be extracted, causing feature matching to fail. The vSLAM system loses tracking and defaults to IMU-only odometry (which drifts rapidly).
2. **Motion blur** (e.g., rapid head movements during running): Camera images are blurred, making feature extraction unreliable. The system either rejects blurred frames or produces inaccurate pose estimates with high uncertainty.
3. **Dynamic scenes** (e.g., crowded spaces with moving people): Features on moving objects are misclassified as static landmarks, corrupting the map. The SLAM filter accumulates incorrect constraints, leading to trajectory warping or divergence.

#### Grading Rubric (Total: 10 points)
- **IMU drift explanation** (2 pts): Mentions quadratic accumulation or quantitative error (1% → 10 cm example)
- **Pipeline stages** (3 pts): All 5 stages correct and in order (0.6 pt each)
- **Loop closure** (2 pts): Explains drift correction and long-duration importance
- **Failure mode 1** (1 pt): Low-texture + technical reason (few features)
- **Failure mode 2** (1 pt): Motion blur + technical reason (feature extraction fails)
- **Failure mode 3** (1 pt): Dynamic scenes + technical reason (moving objects corrupt map)

#### Common Mistakes
- Claiming "IMU is perfectly accurate" (ignores drift physics)
- Confusing pipeline order (e.g., map building before pose estimation)
- Vague loop closure explanation ("fixes errors" without explaining how)
- Unrealistic failure modes (e.g., "camera breaks"—physical failure, not algorithmic)

#### Extensions
- Implement a toy vSLAM pipeline in Python (OpenCV feature detection, simple triangulation)
- Research ORB-SLAM3 vs. RTAB-Map trade-offs (accuracy vs. speed)
- Analyze vSLAM failure in the DARPA Robotics Challenge (Atlas in low-light)

---

## 📗 Application Exercises

### Exercise 6: Isaac Sim Scene Setup

**Difficulty**: Moderate | **Time**: 1.5 hours

#### Complete Solution

**Deliverables**:

1. **Screenshot**: Shows humanoid robot in kitchen with ray-traced lighting (realistic shadows, reflections on table surface)
2. **USD file**: Includes:
   - Humanoid robot (e.g., `/Isaac/Robots/Humanoid/humanoid.usd` or custom URDF2USD conversion)
   - Ground plane (10m × 10m, material: wood or tile)
   - Furniture: 1 table (1.5m × 0.8m, height 0.75m), 4 chairs, 1 counter (2m × 0.6m)
   - Dome light with HDRI (e.g., `indoor_studio.hdr` from Poly Haven)
3. **Camera configuration** (Python script):

```python
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.sensor import Camera

# Create camera on robot's head
camera = Camera(
    prim_path="/World/Humanoid/head_link/camera",
    position=np.array([0.1, 0, 0]),  # 10 cm forward from head
    orientation=np.array([1, 0, 0, 0]),  # quaternion (identity = looking forward)
    frequency=30,  # Hz
    resolution=(640, 480),
    horizontal_aperture=20.955,  # mm (for 60° HFOV on 35mm sensor)
)
camera.initialize()

# Enable ray tracing in viewport settings (GUI)
# Alternatively via script:
import carb
carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
```

**Scene setup notes**:
- Use physics-enabled rigid bodies for furniture (collision detection active)
- Verify camera FOV: `HFOV = 2 * arctan(sensor_width / (2 * focal_length))` → 60° requires horizontal aperture ≈ 21 mm
- Enable RTX PathTracing (not just RayTracing) for full global illumination

#### Grading Rubric (Total: 15 points)
- **Scene completeness** (4 pts): All objects present (robot, floor, table, chairs, counter, light)
- **Realistic arrangement** (2 pts): Furniture positioned logically (not overlapping, realistic kitchen layout)
- **Camera mounting** (3 pts): Correctly attached to `head_link`, position/orientation reasonable
- **Camera configuration** (3 pts): 640×480, 60° HFOV, 30 Hz (verify in script or GUI settings)
- **Ray tracing enabled** (2 pts): Screenshot shows RTX features (soft shadows, reflections)
- **Reproducibility** (1 pt): Instructions or USD file allows recreation

#### Common Mistakes
- Camera not attached to robot (floating in space)
- Incorrect FOV calculation (confusing horizontal/vertical, degrees/radians)
- Ray tracing not enabled (screenshot shows hard shadows, no reflections)
- Missing objects (e.g., only table, no chairs)

#### Verification Checklist
- [ ] Run `camera.get_resolution()` → returns (640, 480)
- [ ] Run `camera.get_frequency()` → returns 30.0
- [ ] Check viewport settings → "PathTracing" or "RayTracing" enabled
- [ ] Screenshot includes realistic shadows and at least one visible reflection

---

### Exercise 7: Replicator Script for Synthetic Data

**Difficulty**: Moderate | **Time**: 2 hours

#### Complete Solution Outline

**Script structure** (`generate_data.py`):

```python
import omni.replicator.core as rep
import numpy as np

# 1. Scene setup
with rep.new_layer():
    # Table
    table = rep.create.cube(
        position=(0, 0, 0.375), scale=(0.8, 0.6, 0.75), semantics=[("class", "table")]
    )

    # 5-10 objects from prop library
    props = [
        rep.create.from_usd(f"/Isaac/Props/YCB/Axis_Aligned/{obj}.usd")
        for obj in ["002_master_chef_can", "003_cracker_box", "004_sugar_box",
                    "005_tomato_soup_can", "006_mustard_bottle", "025_mug",
                    "036_wood_block", "037_scissors"]
    ]

    # Camera
    camera = rep.create.camera(position=(1.5, 0, 1.2), look_at=(0, 0, 0.4))

    # Dome light
    dome_light = rep.create.light(light_type="dome")

# 2. Randomizers
with rep.trigger.on_frame(num_frames=1000):
    # Object positions (on table surface)
    with props:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.4, -0.3, 0.75), (0.4, 0.3, 0.85)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
        )

    # Dome light rotation
    with dome_light:
        rep.modify.pose(rotation=rep.distribution.uniform((0, 0, 0), (360, 0, 0)))

    # Camera position (hemisphere above table)
    with camera:
        radius = rep.distribution.uniform(1.0, 2.0)
        theta = rep.distribution.uniform(0, 360)  # azimuth
        phi = rep.distribution.uniform(30, 80)    # elevation (30-80° above horizon)
        x = radius * np.sin(np.radians(phi)) * np.cos(np.radians(theta))
        y = radius * np.sin(np.radians(phi)) * np.sin(np.radians(theta))
        z = radius * np.cos(np.radians(phi))
        rep.modify.pose(position=(x, y, z), look_at=(0, 0, 0.4))

# 3. Writer (COCO format)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="_out_replicator",
    rgb=True,
    bounding_box_2d_tight=True,
)
writer.attach([camera])

# 4. Orchestrator
rep.orchestrator.run()
```

**Sample COCO JSON** (partial):

```json
{
  "images": [
    {"id": 0, "file_name": "rgb_0000.png", "width": 640, "height": 480}
  ],
  "annotations": [
    {"id": 0, "image_id": 0, "category_id": 1, "bbox": [120, 200, 80, 100], "area": 8000}
  ],
  "categories": [
    {"id": 1, "name": "mug"}
  ]
}
```

**README.md**:
```markdown
## Setup
1. Install Isaac Sim 2023.1+ with Replicator extension
2. Run: `python generate_data.py`
3. Output: `_out_replicator/` contains 1,000 RGB images + `coco_annotations.json`

## Verification
- Check image count: `ls _out_replicator/rgb*.png | wc -l` → 1000
- Validate COCO JSON: Use `pycocotools` to load annotations
```

#### Grading Rubric (Total: 20 points)
- **Replicator API usage** (5 pts): Correct randomizers, writer, orchestrator setup
- **Randomization ranges** (4 pts): Objects within table bounds, camera in hemisphere (no invalid poses)
- **COCO format** (4 pts): Valid JSON schema (images, annotations, categories arrays)
- **Completeness** (3 pts): Generates all 1,000 images without errors
- **Documentation** (2 pts): README with clear setup/run instructions
- **Sample outputs** (2 pts): 3 images + JSON snippet demonstrate correctness

#### Common Mistakes
- Objects spawn inside table or floating (wrong Z coordinate)
- Camera spawns inside table (phi=90°, radius=0.5m → z=0)
- COCO JSON missing required fields (image_id, category_id)
- Script crashes after 100 frames (memory leak, not clearing physics state)

#### Verification Checklist
- [ ] Run script → completes without errors
- [ ] Check output folder → contains 1,000 RGB images
- [ ] Validate COCO JSON: `python -c "from pycocotools.coco import COCO; COCO('_out_replicator/coco.json')"`
- [ ] Manually inspect 10 random images → objects on table, lighting varies, camera angle changes

---

### Exercise 8: Isaac ROS Visual SLAM Launch

**Difficulty**: Moderate | **Time**: 1.5 hours

#### Complete Solution Outline

**Launch file** (`vslam_demo.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to RViz config
    rviz_config = os.path.join(os.getcwd(), "vslam_demo.rviz")

    return LaunchDescription([
        # Isaac Sim bridge (assumes Isaac Sim already running)
        # Alternatively, launch Isaac Sim via separate process if needed

        # Isaac ROS Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'use_sim_time': True,
                'enable_imu_fusion': True,
                'enable_loop_closure': True,
                'rectified_images': True,
                'num_cameras': 1,
            }],
            remappings=[
                ('stereo_camera/left/image', '/camera/rgb/image_raw'),
                ('stereo_camera/left/camera_info', '/camera/rgb/camera_info'),
                ('stereo_camera/right/image', '/camera/depth/image_raw'),  # Depth as "right stereo"
                ('stereo_camera/right/camera_info', '/camera/depth/camera_info'),
                ('imu', '/imu'),
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

**RViz config** (`vslam_demo.rviz`):
- Add `Image` display: Topic `/camera/rgb/image_raw`
- Add `PoseStamped` display: Topic `/visual_slam/tracking/vo_pose`
- Add `PointCloud2` display: Topic `/visual_slam/tracking/slam_map` (if available)
- Set Fixed Frame to `odom` or `map`

**README.md**:
```markdown
## Setup
1. Install Isaac ROS: `sudo apt install ros-humble-isaac-ros-visual-slam`
2. Launch Isaac Sim with humanoid robot + RGB-D camera
3. Run: `ros2 launch vslam_demo.launch.py`
4. Teleop robot: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Verification
- RViz shows camera image + SLAM trajectory
- Check SLAM status: `ros2 topic echo /visual_slam/tracking/vo_pose`
```

#### Grading Rubric (Total: 20 points)
- **Launch file structure** (5 pts): Correct nodes, parameters, remappings
- **Topic remappings** (4 pts): Match Isaac Sim output (`/camera/rgb/image_raw`) to Isaac ROS input
- **use_sim_time** (2 pts): Set to `true` for all nodes
- **RViz displays** (4 pts): Image, PoseStamped trajectory, PointCloud (if applicable)
- **SLAM functionality** (3 pts): Trajectory tracks robot motion (screenshot/video shows movement)
- **Documentation** (2 pts): README with setup and run steps

#### Common Mistakes
- Topic names don't match (e.g., Isaac Sim publishes `/camera/color/image_raw`, launch expects `/camera/rgb/image_raw`)
- Forgetting `use_sim_time:=true` (causes timestamp mismatch, SLAM fails)
- RViz Fixed Frame incorrect (e.g., `base_link` instead of `odom`, trajectory doesn't display)
- No verification (just launch files, no proof SLAM works)

#### Verification Checklist
- [ ] Launch succeeds without errors
- [ ] RViz displays camera image (live feed)
- [ ] SLAM publishes odometry: `ros2 topic hz /visual_slam/tracking/vo_pose` → ~30 Hz
- [ ] Move robot → RViz trajectory updates in real-time

---

### Exercise 9: Convert PyTorch Model to TensorRT

**Difficulty**: Challenging | **Time**: 2 hours

#### Complete Solution

**1. Export ONNX** (`export_onnx.py`):

```python
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.export(format='onnx', opset=11, simplify=True)
print("Exported to yolov8n.onnx")
```

**2. Convert to TensorRT** (`convert_trt.sh`):

```bash
#!/bin/bash
trtexec \
  --onnx=yolov8n.onnx \
  --saveEngine=yolov8n_fp16.trt \
  --fp16 \
  --workspace=4096 \
  --verbose
```

**3. Benchmark script** (`benchmark_inference.py`):

```python
import torch
import tensorrt as trt
import numpy as np
import time
from ultralytics import YOLO

# Dummy input
dummy_input = np.random.rand(1, 3, 640, 640).astype(np.float32)

# PyTorch CPU
model_cpu = YOLO('yolov8n.pt')
model_cpu.to('cpu')
start = time.time()
for _ in range(100):
    model_cpu.predict(dummy_input, verbose=False)
cpu_time = (time.time() - start) / 100 * 1000  # ms

# PyTorch GPU
model_gpu = YOLO('yolov8n.pt')
model_gpu.to('cuda')
dummy_input_gpu = torch.from_numpy(dummy_input).to('cuda')
torch.cuda.synchronize()
start = time.time()
for _ in range(100):
    model_gpu.predict(dummy_input_gpu, verbose=False)
torch.cuda.synchronize()
gpu_time = (time.time() - start) / 100 * 1000  # ms

# TensorRT (using trtexec or Python API)
# For simplicity, run trtexec separately and paste results
# trtexec --loadEngine=yolov8n_fp16.trt --iterations=100
# Example output: mean = 8.5 ms

trt_time = 8.5  # Placeholder (actual value from trtexec)

# Results
print(f"| PyTorch CPU | {cpu_time:.1f} ms | {1000/cpu_time:.1f} FPS | 1.0× |")
print(f"| PyTorch GPU | {gpu_time:.1f} ms | {1000/gpu_time:.1f} FPS | {cpu_time/gpu_time:.1f}× |")
print(f"| TensorRT FP16 | {trt_time:.1f} ms | {1000/trt_time:.1f} FPS | {cpu_time/trt_time:.1f}× |")
```

**Sample results table**:

| **Model Type** | **Latency (ms)** | **FPS** | **Speedup vs CPU** |
|----------------|------------------|---------|--------------------|
| PyTorch CPU | 150.2 ms | 6.7 FPS | 1.0× |
| PyTorch GPU | 35.8 ms | 27.9 FPS | 4.2× |
| TensorRT FP16 | 8.5 ms | 117.6 FPS | 17.7× |

**Analysis** (1 paragraph):
TensorRT achieves a 17.7× speedup over CPU by leveraging GPU parallelism (4.2×) and further optimizations like layer fusion, kernel auto-tuning, and FP16 precision (additional 4.2×). FP16 halves memory bandwidth requirements and utilizes Tensor Cores on RTX GPUs, which are designed for mixed-precision arithmetic. The combined effect enables real-time object detection at 117 FPS on 640×640 images, suitable for robotic applications requiring sub-10ms latency.

#### Grading Rubric (Total: 15 points)
- **Conversion correctness** (3 pts): ONNX export succeeds, TensorRT engine builds without errors
- **Benchmark methodology** (4 pts): Averaged over 100 runs, excludes warmup, uses `torch.cuda.synchronize()`
- **Results table** (3 pts): Realistic progression (CPU < GPU < TensorRT), actual values (not placeholders)
- **Speedup analysis** (3 pts): Explains GPU parallelism, TensorRT optimizations, FP16 precision
- **Reproducibility** (2 pts): Scripts run without modification, commands documented

#### Common Mistakes
- Forgetting warmup (first inference includes model loading time)
- Not synchronizing CUDA (measures kernel launch time, not execution time)
- Unrealistic speedup claims (e.g., 100× from TensorRT alone)
- Missing FP16 flag in `trtexec` (builds FP32 engine, slower)

#### Verification Checklist
- [ ] ONNX file exists and is valid (load with `onnx.checker`)
- [ ] TensorRT engine builds: `ls yolov8n_fp16.trt` (file size ~10-20 MB)
- [ ] Benchmark shows clear speedup: TensorRT > PyTorch GPU > PyTorch CPU
- [ ] Analysis mentions specific optimizations (not just "it's faster")

---

### Exercise 10: Isaac ROS DNN Inference Pipeline

**Difficulty**: Challenging | **Time**: 2.5 hours

#### Complete Solution Outline

**Launch file** (`dnn_inference.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    rviz_config = os.path.join(os.getcwd(), "dnn_inference.rviz")

    return LaunchDescription([
        # Isaac ROS DNN Inference node
        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            name='dnn_inference',
            parameters=[{
                'model_file_path': '/path/to/yolov8n_fp16.trt',
                'engine_file_path': '/path/to/yolov8n_fp16.trt',
                'input_tensor_names': ['images'],
                'input_binding_names': ['images'],
                'output_tensor_names': ['output0'],
                'output_binding_names': ['output0'],
                'use_sim_time': True,
            }],
            remappings=[
                ('tensor_pub', '/detections'),
                ('image', '/camera/rgb/image_raw'),
            ]
        ),

        # Latency monitor node
        Node(
            package='custom_pkg',  # Assuming custom package
            executable='latency_monitor.py',
            name='latency_monitor',
            parameters=[{'use_sim_time': True}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

**Latency monitor node** (`latency_monitor.py`):

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.image_timestamps = {}
        self.latencies = []

    def image_callback(self, msg):
        self.image_timestamps[msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9] = msg.header.stamp

    def detection_callback(self, msg):
        detection_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Find closest image timestamp
        if self.image_timestamps:
            image_time = min(self.image_timestamps.keys(), key=lambda t: abs(t - detection_time))
            latency = (detection_time - image_time) * 1000  # ms
            self.latencies.append(latency)
            if len(self.latencies) == 100:
                self.get_logger().info(f"Latency stats (100 frames): mean={np.mean(self.latencies):.2f} ms, "
                                       f"std={np.std(self.latencies):.2f} ms, "
                                       f"min={np.min(self.latencies):.2f} ms, "
                                       f"max={np.max(self.latencies):.2f} ms")
                self.latencies = []

def main():
    rclpy.init()
    node = LatencyMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Sample latency statistics**:
```
mean=12.5 ms, std=2.3 ms, min=8.2 ms, max=18.7 ms
```

**README.md**:
```markdown
## Setup
1. Build custom package: `colcon build --packages-select custom_pkg`
2. Source workspace: `source install/setup.bash`
3. Launch: `ros2 launch dnn_inference.launch.py`

## Verification
- RViz shows detections overlaid on camera image
- Terminal prints latency stats every 100 frames
```

#### Grading Rubric (Total: 20 points)
- **Isaac ROS node setup** (5 pts): Correct parameters, model path, topic remappings
- **Detections accuracy** (4 pts): Correct classes, reasonable bounding boxes (verified in RViz)
- **Latency measurement** (4 pts): Timestamp differencing (image → detection), computed over 100 frames
- **Real-time performance** (3 pts): Mean latency < 50 ms for 640×480 (on RTX/Jetson)
- **Visualization** (2 pts): RViz displays image + bounding boxes
- **Documentation** (2 pts): README with setup and execution steps

#### Common Mistakes
- Model path incorrect (node fails to load TensorRT engine)
- Latency measurement uses `time.time()` instead of message timestamps (includes ROS overhead)
- No warmup (first 10-20 frames have higher latency due to initialization)
- Bounding boxes not displayed (missing RViz plugin configuration)

#### Verification Checklist
- [ ] Launch succeeds, DNN inference node publishes detections
- [ ] RViz shows live camera feed + bounding boxes
- [ ] Latency monitor prints stats every 100 frames
- [ ] Latency < 50 ms (real-time for 30 Hz camera)

---

## 📕 Synthesis Exercises

### Exercise 11: Design a Sim-to-Real Perception Pipeline

**Difficulty**: Moderate | **Time**: 1.5 hours

#### Complete Solution (700-900 words)

**1. Training data generation (Isaac Sim)**

**Scene setup**:
- Environment: Kitchen (10m × 8m) with counters, table, chairs, appliances (fridge, microwave, sink)
- Objects: 15 object categories (mugs, bowls, bottles, utensils, food items) from YCB dataset, spawned in randomized poses
- Camera: RGB-D camera (640×480, 60° HFOV) mounted on humanoid head (`/head_link`), height 1.5-1.8m (standing/crouching randomized)

**Domain randomization (5 parameters)**:
1. **Lighting**: Dome light rotation (0-360°), intensity (500-2000 lux), randomize per scene
2. **Object textures**: Randomize materials (10 variants per object: ceramic, plastic, metal) and colors (HSV: H=0-360°)
3. **Object poses**: X, Y uniform within counters/table bounds, Z = surface height, yaw rotation 0-360°
4. **Background clutter**: Spawn 5-20 additional distractor objects (not in target classes)
5. **Camera noise**: Add Gaussian noise to RGB (σ=5/255), depth sensor noise model (σ = 1cm + 0.05% of distance)

**Dataset size**: 50,000 images (3,333 images per object category). Justification: Modern YOLO models require 1,000-5,000 instances per class for robust performance; 3,333 ensures sufficient variation with domain randomization.

**2. Model training**

**Architecture**: YOLOv8m (medium variant). Why: Balances accuracy (mAP ~45% on COCO) and speed (25-30 FPS on RTX 4090 with TensorRT). Larger models (YOLOv8x) are too slow for real-time; smaller (YOLOv8n) sacrifice accuracy.

**Framework**: PyTorch (via Ultralytics library for ease of use, built-in augmentation, export to ONNX).

**Hyperparameters**:
- Learning rate: 0.001 (with cosine annealing)
- Batch size: 32 (fits in 24 GB VRAM for 640×640 images)
- Epochs: 100 (with early stopping if validation mAP plateaus for 20 epochs)
- Augmentation: Mosaic (4-image mix), random scale (0.5-1.5×), HSV jittering (already covered by Isaac Sim randomization, minimal additional needed)

**3. Deployment (Isaac ROS)**

**TensorRT conversion**:
1. Export PyTorch to ONNX: `yolo export model=yolov8m.pt format=onnx`
2. Convert ONNX to TensorRT FP16: `trtexec --onnx=yolov8m.onnx --saveEngine=yolov8m_fp16.trt --fp16`
3. Verify engine: Run inference on 100 test images, ensure mAP drop < 1% vs. PyTorch

**Isaac ROS GEMs**:
- `isaac_ros_dnn_inference`: Runs TensorRT engine on input images
- `isaac_ros_image_proc`: Preprocessing (resize to 640×640, normalization)
- Optional: `isaac_ros_apriltag` for re-localization (if detection fails, use AprilTags as backup)

**Hardware**: NVIDIA Jetson Orin NX (16 GB VRAM, $800). Rationale: Embedded deployment on humanoid robot; RTX 4090 is too large/power-hungry for mobile robots. Orin NX achieves 20-25 FPS with TensorRT FP16, sufficient for manipulation tasks.

**4. Validation**

**Sim-to-real gap measurement**:
- Collect 500 real-world kitchen images (manual capture with humanoid robot camera)
- Annotate ground truth bounding boxes (COCO format)
- Compute mAP@0.5 on:
  - Synthetic test set (5,000 images, separate from training): Expected mAP ~75%
  - Real-world test set (500 images): Expected mAP ~60-65% (domain gap)
- Gap metric: `(mAP_sim - mAP_real) / mAP_sim` → Target < 20%

**Three real-world tests**:
1. **Office kitchen test**: 10 object instances (varying lighting: morning, noon, evening). Success: mAP > 60% across all lighting conditions.
2. **Occlusion robustness**: Place objects partially behind others (50% visibility). Success: Detect ≥80% of objects with >50% visibility.
3. **Novel object generalization**: Test on 5 unseen object instances (same categories, different textures/shapes). Success: mAP > 55% (slight drop acceptable for novel instances).

**5. Mitigation strategies (if mAP drops from 85% to 60%)**

1. **Fine-tune on real data**: Collect 1,000 real-world images (diverse lighting, backgrounds), annotate, fine-tune YOLOv8 for 20 epochs with low learning rate (0.0001). Expected improvement: +10-15% mAP.
2. **Enhance synthetic realism**: Add motion blur (camera shake during robot movement), chromatic aberration (lens distortion), lens flare. Re-train on augmented synthetic data. Expected improvement: +5-8% mAP.
3. **Domain adaptation**: Use CycleGAN to translate synthetic images to "real-looking" style before training. Train YOLOv8 on style-transferred images. Expected improvement: +8-12% mAP (reduces texture mismatch).

#### Grading Rubric (Total: 20 points)
- **Training data plan** (4 pts): Diverse scene, 5 randomization parameters with ranges, justified dataset size
- **Model choice** (3 pts): YOLOv8 (or equivalent) with justification (real-time constraints)
- **Training setup** (3 pts): Hyperparameters are realistic (batch size, epochs, learning rate)
- **Deployment plan** (4 pts): TensorRT conversion steps, Isaac ROS GEMs, appropriate hardware
- **Validation metrics** (3 pts): Quantitative sim-to-real gap measurement (mAP on both datasets)
- **Mitigation strategies** (3 pts): 3 specific, actionable strategies with expected impact

#### Common Mistakes
- Vague randomization ("randomize lighting"—no range specified)
- Unrealistic dataset size (1,000 images total for 15 classes = 67 per class, insufficient)
- No justification for model choice (claiming YOLOv8 without explaining trade-offs)
- Generic mitigation ("collect more data"—doesn't specify how much or what kind)

---

### Exercise 12: Analyze Isaac Sim vs. Real-World Depth Camera Fidelity

**Difficulty**: Challenging | **Time**: 2 hours

#### Complete Solution (800-1000 words)

**1. Five causes of depth sensor sim-to-real gap**

| **Cause** | **Gap description** | **Impact on grasping** |
|-----------|---------------------|------------------------|
| **Flying pixels** | Simulated depth has clean edges; real RealSense D435 produces "flying pixels" (spurious depth values at object boundaries due to mixed-pixel effect). | Corrupts grasp pose estimation—grasping planner sees incorrect object edges, leading to off-center grasps that slip. |
| **Depth noise** | Simulated depth is noise-free; real sensors have distance-dependent noise (σ = 1cm + 0.05% × distance²). | Noisy depth causes jittery grasp poses; robot's gripper may miss the object by 1-2 cm, critical for small objects. |
| **Infrared interference** | Simulated cameras don't model IR pattern degradation in bright sunlight or reflective surfaces; real sensors fail in these conditions. | Depth map becomes invalid (all zeros or NaN) near windows or metallic objects, causing grasping to abort. |
| **Motion blur** | Simulated cameras capture sharp images even during fast robot motion; real cameras suffer from rolling shutter and motion blur. | Blurred depth maps produce incorrect surface normals, causing grasping planner to approach objects at wrong angles. |
| **Temporal lag** | Simulated depth is synchronous with RGB; real sensors have 10-30ms RGB-depth misalignment. | Grasp pose computed from misaligned RGB-D frames leads to spatial errors when object is moving (e.g., on a moving tray). |

**2. Five mitigation strategies**

| **Cause** | **Mitigation strategy** | **Difficulty** | **Impact** |
|-----------|-------------------------|----------------|-----------|
| **Flying pixels** | Post-process simulated depth: detect edges (Canny), set depth values within 2-pixel border to NaN or average of neighbors. | Easy (OpenCV) | High—eliminates edge artifacts that corrupt grasp planning |
| **Depth noise** | Add Gaussian noise: `depth_noisy = depth + N(0, σ(d))` where `σ(d) = 0.01 + 0.0005 × d²` (meters). | Easy (NumPy) | Medium—prevents overfitting to perfect depth, but doesn't fix other gaps |
| **IR interference** | Randomly invalidate 10-30% of depth pixels in bright regions (simulate sunlight) and near high-reflectivity materials (metallic objects). | Moderate (requires material property detection) | High—forces grasping policy to handle missing depth data (fallback to RGB-only or tactile) |
| **Motion blur** | Apply directional blur to depth images based on robot velocity: `blur_kernel_size = k × |v_robot|` where k=0.5 px/(m/s). | Moderate (requires robot odometry in sim) | Medium—teaches policy to avoid fast motions during grasping |
| **Temporal lag** | Offset RGB and depth timestamps by 10-30ms in training data (shift depth frame indices by 1-3 frames). | Easy (data pipeline modification) | Low—helps only if object is moving; static grasping unaffected |

**3. Experiment design (validating depth noise mitigation)**

**Test setup**:
- **Sim scene**: Table with 10 objects (mugs, bowls, boxes) at distances 0.5-1.5m from camera. Lighting: indoor HDRI, no direct sunlight.
- **Real scene**: Same objects arranged identically (use AprilTags for registration), RealSense D435 mounted at same position/orientation as simulated camera.

**Data collection**:
- Capture 100 depth images per scene (sim and real).
- Metrics:
  1. **Mean Absolute Error (MAE)**: Compare depth values at corresponding pixels (after alignment). Compute `MAE = mean(|d_sim - d_real|)` for object regions (mask background).
  2. **Edge artifact density**: Count pixels with depth discontinuities > 10cm within 5-pixel radius of object edges. Metric: `artifacts_per_100px_edge_length`.
  3. **Grasp success rate**: Run grasping policy on both datasets (50 grasps per dataset). Measure `success_rate = # successful grasps / 50`.

**Success criteria**:
- **Before mitigation** (no noise added to sim):
  - MAE > 3 cm (sim is too perfect)
  - Edge artifact density: 0 (sim) vs. 15 (real)
  - Grasp success: 90% (sim) → 65% (real)
- **After mitigation** (noise + edge artifacts added to sim):
  - MAE reduces to 1-2 cm (closer to real sensor)
  - Edge artifact density: 12-18 (sim matches real)
  - Grasp success: 75% (sim) → 80% (real)—better generalization

If real-world success rate increases from 65% to 80% (≥15 percentage points), the mitigation is validated.

#### Grading Rubric (Total: 20 points)
- **Causes** (5 pts): 5 technically accurate causes (flying pixels, noise, IR interference, motion blur, temporal lag)
- **Impact explanations** (3 pts): Explains how each gap affects grasping (not generic)
- **Mitigation strategies** (5 pts): Specific parameters (e.g., σ(d) formula), difficulty/impact estimates justified
- **Experiment setup** (4 pts): Scene description, data collection protocol, quantitative metrics (MAE, artifact density, grasp success)
- **Success criteria** (3 pts): Defines before/after metrics, improvement threshold (≥15 pp increase)

#### Common Mistakes
- Vague causes ("depth is wrong"—doesn't specify flying pixels, noise, etc.)
- Generic mitigations ("add noise"—doesn't specify distribution or parameters)
- No quantitative success criteria (e.g., "success rate improves"—by how much?)
- Unrealistic improvement claims (90% → 100% after one mitigation)

---

### Exercise 13: Design a Footstep Planner with Isaac ROS and Nav2

**Difficulty**: Challenging | **Time**: 2.5 hours

#### Complete Solution (1000-1200 words)

**Architecture diagram**: [Hand-drawn or digital]
```
Sensors → Isaac ROS vSLAM → Odometry → Nav2 Costmap → Footstep Planner → Locomotion Controller → Robot
         ↑                                            ↑
         IMU (fusion)                                Custom Plugin
```

**1. Perception pipeline**

**Isaac ROS GEMs**:
- `isaac_ros_visual_slam`: Primary localization using RGB-D camera (640×480, 30 Hz). Outputs `/visual_slam/tracking/odometry` (6-DOF pose).
- `isaac_ros_apriltag`: Backup localization for loop closure in feature-poor areas (e.g., white walls). Detects fiducial markers at doorways, updates global pose.

**Sensors**:
- **RGB-D camera** (head-mounted): 30 Hz, 60° HFOV, range 0.5-5m. Primary input for vSLAM feature extraction.
- **IMU** (torso-mounted): 100 Hz, 6-axis (accel + gyro). Fused with vSLAM for high-frequency pose updates (vSLAM runs at 30 Hz, IMU at 100 Hz fills gaps).
- **Optional: 2D LiDAR** (chest-mounted): 10 Hz, 270° FOV, range 0.1-10m. Provides obstacle detection for costmap (more reliable than depth camera for navigation).

**Loop closure**:
- Isaac ROS vSLAM runs loop closure automatically (bag-of-words matching + pose graph optimization).
- For long-duration navigation (>30 min), add AprilTag relocalization every 5 minutes: robot scans for tags, corrects drift if detected.

**2. Costmap configuration**

**Layers**:
1. **Static obstacles**: Pre-built map from SLAM (walls, furniture). Update frequency: 0.1 Hz (only when map changes).
2. **Dynamic obstacles**: Real-time sensor data (LiDAR, depth camera). Update frequency: 5 Hz. Use temporal filtering (3-frame average) to reduce noise.
3. **Inflation layer**: Expand obstacles by footstep safety margin (see below).

**Bipedal footprint**:
- Represent as **two rectangular foot polygons** (each 25cm × 12cm, separated by step width ~20cm).
- During planning, check both feet for collisions (not a single circular footprint as in wheeled robots).

**Inflation radius**:
- Base inflation: 15 cm (half of step length ~30 cm, ensures foot doesn't collide mid-step).
- Additional safety margin: 5 cm (accounts for pose uncertainty from vSLAM).
- Total: 20 cm around obstacles.
- Justification: If step length is 30 cm, foot must not approach obstacles within 15 cm (else foot collides mid-swing). Adding 5 cm for uncertainty prevents edge cases.

**3. Footstep planner design**

**Algorithm** (A* search over discrete foot placements):
1. **State space**: Each state = (left_foot_pose, right_foot_pose, support_foot). Pose = (x, y, yaw).
2. **Action space**: From current state, generate next foot placements:
   - Forward step: ±30 cm ahead, yaw ±15°
   - Lateral step: ±10 cm sideways, yaw 0°
   - Turn in place: yaw ±30°, no translation
3. **Constraints**:
   - Step length: 20-40 cm (kinematic limits)
   - Step height: Max 15 cm (stair climbing capability)
   - Balance: Center of Mass (CoM) must remain within support polygon (convex hull of support foot)
   - No penetration: Check collision for both feet against costmap
4. **Cost function**: `cost = distance_to_goal + 0.5 × num_steps + 2.0 × heading_change`
   - Encourages direct paths, minimizes steps, penalizes excessive turning
5. **Heuristic**: Euclidean distance to goal (admissible for A*)

**Nav2 integration**:
- Implement as **custom Nav2 planner plugin** (inherit from `nav2_core::GlobalPlanner`).
- Plugin reads costmap from Nav2, computes footstep plan, publishes as `nav_msgs/Path` (visualization) and custom `FootstepArray` message (for locomotion controller).

**4. Locomotion controller**

**Execution**:
- Once footsteps are planned, locomotion controller (from Module 2) executes them sequentially:
  1. Shift weight to support foot (ZMP-based balance).
  2. Swing non-support foot to target pose (trajectory planning via cubic splines).
  3. Land foot, stabilize (PD control on joint torques).
  4. Repeat for next footstep.

**Dynamic replanning**:
- Subscribe to costmap updates at 5 Hz.
- If new obstacle detected in planned footstep path (costmap value > threshold):
  1. Pause current step.
  2. Recompute footstep plan from current pose to goal.
  3. Resume execution with new plan.
- Replanning frequency: Max 1 Hz (avoid thrashing; footstep planning takes ~0.5-1 sec for 10-step plans).

**5. Failure modes and recovery**

| **Failure scenario** | **Recovery behavior** |
|----------------------|----------------------|
| **vSLAM loses tracking** (low-light corridor) | Switch to IMU-only odometry for 5 seconds (accept drift), reduce walking speed to 0.3 m/s (safer), trigger AprilTag scan for relocalization. If tracking doesn't recover in 10 sec, stop and alert operator. |
| **Footstep planner finds no valid path** (blocked by dynamic obstacle) | Enter "sidestep mode": attempt lateral steps (±10 cm) for 3 iterations. If still blocked, request operator guidance (teleoperation) or switch to "wait mode" (obstacle may move). |
| **Locomotion controller falls** (slippery floor, unexpected push) | Detect fall via IMU (accel > 2g, pitch > 45°). Trigger safe fall protocol (extend arms to brace, go limp to minimize damage). After fall, enter recovery mode: attempt to stand up (if possible) or alert operator for manual intervention. |

#### Grading Rubric (Total: 25 points)
- **Perception pipeline** (5 pts): Isaac ROS GEMs correct (vSLAM, AprilTag), sensors specified (RGB-D, IMU, LiDAR), loop closure handled
- **Costmap configuration** (4 pts): Layers realistic (static, dynamic, inflation), bipedal footprint (two feet), justified inflation radius
- **Footstep planner** (7 pts): Algorithm described (A* or equivalent), constraints listed (step length, height, balance, collision), Nav2 integration method
- **Locomotion controller** (4 pts): Execution steps (ZMP, swing trajectory), dynamic replanning strategy
- **Failure modes** (5 pts): 3 realistic scenarios with specific recovery behaviors (not vague "retry")

#### Common Mistakes
- Using circular footprint (ignores bipedal geometry)
- No loop closure handling (vSLAM drift unbounded over 30 min)
- Vague planner algorithm ("uses AI"—not a concrete algorithm)
- Generic recovery ("try again"—doesn't specify how or when to give up)

---

### Exercise 14: Evaluate Synthetic Data Quality for VLA Training

**Difficulty**: Challenging | **Time**: 2 hours

#### Complete Solution (900-1100 words)

**1. Diversity metrics (5 quantitative metrics)**

| **Metric** | **Computation method** | **Target value** |
|------------|------------------------|------------------|
| **Object category distribution** | Count images per object class: `count[class] = # images containing class`. | ≥500 images per class (for 15 classes, 50k total) |
| **Lighting histogram** | Compute mean pixel intensity histogram (256 bins) across dataset. Measure entropy: `H = -Σ p(i) log p(i)`. | Entropy H > 6.5 bits (uniform distribution has H=8 bits; H>6.5 ensures diverse lighting) |
| **Pose variety (6-DOF)** | For each object class, compute variance of 6-DOF poses: `Var(X, Y, Z, roll, pitch, yaw)`. | Variance > 0.1 m² (position), > 30° (orientation) per axis |
| **Background complexity** | Measure texture entropy: For each image, compute GLCM (Gray-Level Co-occurrence Matrix), extract entropy. | Mean entropy > 7.0 bits (cluttered backgrounds have high entropy) |
| **Camera viewpoint coverage** | Convert camera positions to spherical coordinates (θ, φ), plot 2D histogram (10° bins). Count occupied bins. | ≥80% of feasible bins occupied (assuming hemisphere above scene, ~200 bins → ≥160 occupied) |

**2. Realism assessment (3 tests)**

**Test 1: Real vs. Synthetic classifier**
- **Dataset**: 5,000 real images (collected from humanoid robot) + 5,000 synthetic images (from Isaac Sim). Label real=1, synthetic=0.
- **Protocol**: Train a ResNet-18 binary classifier (80/20 train/test split). Measure test accuracy.
- **Success threshold**: Accuracy < 60%. If classifier achieves >70%, synthetic images have obvious artifacts (too easy to distinguish).

**Test 2: Feature distribution matching**
- **Dataset**: Extract CLIP embeddings (512-D vectors) for 1,000 real + 1,000 synthetic images.
- **Protocol**: Compute Fréchet Inception Distance (FID): `FID = ||μ_real - μ_syn||² + Tr(Σ_real + Σ_syn - 2√(Σ_real Σ_syn))` where μ, Σ are mean/covariance of embeddings.
- **Success threshold**: FID < 50. Lower FID = more similar distributions. FID > 100 indicates significant domain gap.

**Test 3: Human perceptual study**
- **Dataset**: 100 image pairs (real vs. synthetic, matched by scene content).
- **Protocol**: Show pairs to 20 human raters (3 sec per pair), ask "Which is real?" Measure accuracy.
- **Success threshold**: Accuracy < 65%. If humans can't reliably distinguish (random guessing = 50%, 65% allows some perceptual cues), data is photorealistic.

**3. Domain gap mitigation (3 synthetic artifacts)**

| **Artifact** | **Problem** | **Fix** |
|-------------|-------------|---------|
| **Perfect edges (no anti-aliasing)** | Real cameras have lens blur and sensor anti-aliasing; synthetic edges are pixel-perfect. VLA policy may fail on slightly blurry real images. | Apply Gaussian blur (σ=0.5 px) to synthetic images post-render, or enable Isaac Sim's anti-aliasing filter (FXAA or TAA). |
| **No motion blur** | Synthetic images are sharp even during robot motion; real images blur during arm movement. Policy trained on sharp images may misinterpret blurred edges as object boundaries. | Add motion blur in Isaac Sim: enable camera motion blur with shutter speed = 1/60s (realistic for 30 FPS camera). Requires simulating camera motion during frame capture. |
| **Perfect depth maps (no flying pixels)** | Synthetic depth is noise-free; real RealSense has flying pixels at edges. VLA policy relying on depth for grasping may fail due to edge artifacts. | Post-process synthetic depth: detect edges (Sobel), invalidate depth within 2-pixel border (set to NaN). Add Gaussian noise: `depth += N(0, 0.01 + 0.0005 × depth²)`. |

**4. VLA-specific considerations (language annotations)**

**Annotation generation**:
- **Automatic (from scene graph)**: Isaac Sim provides object poses and class labels. Generate annotations programmatically:
  - Template 1: "Pick up the [color] [object]" → "Pick up the red mug"
  - Template 2: "Grasp the [object] on the [surface]" → "Grasp the bowl on the counter"
  - Template 3: "Open the [ordinal] [object]" → "Open the top drawer"
- Use scene graph to fill placeholders: object class, color (from texture name), spatial relation (compute from poses: "on", "near", "left of").

**Annotation format**:
- Structured JSON:
```json
{
  "image_id": 12345,
  "instruction": "Pick up the red mug",
  "target_object": {"class": "mug", "bbox": [120, 200, 80, 100], "color": "red"},
  "action": "grasp"
}
```

**Language variations**:
- Generate 5-10 variations per scene by:
  1. Synonym substitution: "Pick up" ↔ "Grab" ↔ "Retrieve", "mug" ↔ "cup"
  2. Structural variation: "Pick up the red mug" ↔ "Grasp the mug that is red"
  3. Adding spatial context: "Pick up the red mug near the sink"
- Use GPT-4 API to paraphrase templates (ensure diversity without changing semantics).

**Scalability**: Automatic generation scales to 50,000 images (10,000 GPU-hours for Isaac Sim rendering + 1 hour for GPT-4 paraphrasing). Manual labeling would require 5,000 person-hours (10 min per image × 50k) = infeasible.

#### Grading Rubric (Total: 20 points)
- **Diversity metrics** (5 pts): 5 quantitative metrics with formulas and target values
- **Realism assessment** (5 pts): 3 objective tests (not subjective), clear protocols and thresholds
- **Domain gap mitigation** (5 pts): 3 specific artifacts with technical problems and fixes (Gaussian blur parameters, etc.)
- **VLA annotation strategy** (5 pts): Automatic generation method (scene graph templates), format (JSON), variation strategy (5-10 per scene)

#### Common Mistakes
- Subjective diversity metrics ("images look diverse"—not quantitative)
- Vague realism tests ("humans look at images"—no protocol or threshold)
- Generic fixes ("add noise"—doesn't specify distribution, parameters)
- Manual annotation (doesn't scale to 50k images)

---

### Exercise 15: Design a Multi-Robot Isaac Sim Training Environment

**Difficulty**: Challenging | **Time**: 3 hours

#### Complete Solution (1200-1500 words)

**1. Hardware requirements**

**GPU selection**: 8× NVIDIA A100 80GB (data center GPUs).

**VRAM per robot**:
- Scene geometry: 50 MB (warehouse: floor, walls, shelves, pallets—instanced to save memory)
- Robot URDF + physics state: 10 MB (humanoid mesh, joint states, collision shapes)
- Textures: 20 MB (PBR materials for robot + environment, shared across instances)
- Rendering buffers: 5 MB (camera RGB + depth, 640×480 resolution)
- **Total per robot**: ~85 MB

**Total VRAM**:
- 512 robots × 85 MB = **43.5 GB**
- Additional overhead (physics solver, Isaac Sim core): 10 GB
- **Total**: ~55 GB (fits in single A100 80GB with headroom)

**CPU & RAM**:
- CPU cores: 64 cores (AMD EPYC or Intel Xeon). Usage: scene graph updates, ROS messaging, dataset I/O.
- RAM: 256 GB (safety factor 2× for OS, logging, data buffering).

**Cost estimate**:
- 8× A100 80GB: $80,000 (cloud rental: ~$25/hr/GPU → $200/hr total)
- Workstation: $15,000 (CPU, RAM, storage, PSU, cooling)
- **Total**: ~$95,000 (or $200/hr cloud)

**2. Scene design**

**Warehouse environment**:
- Dimensions: 20m × 20m × 5m (height) per robot instance.
- Obstacles: Shelving units (2m × 0.5m × 2m, arranged in aisles), pallets (1.2m × 0.8m × 0.15m, randomly placed), dynamic human agents (3-5 per environment, walking at 1 m/s).
- Goal positions: Randomized within warehouse bounds, marked with AprilTag (for visual guidance).

**512 independent environments**:
- Use **USD instancing**: Define warehouse as a single USD prototype, instantiate 512 copies with different transforms.
- Each instance has isolated physics context (no inter-environment collisions).
- Memory saved: Instead of 512 × 50 MB = 25 GB for geometry, instancing uses 50 MB + 512 × 1 MB (instance metadata) = ~600 MB.

**Visual fidelity**:
- **Fast rasterization** (not ray tracing). Justification:
  - Goal: Maximize throughput (512 robots × 50 Hz = 25,600 frames/sec). Ray tracing would reduce to ~5,000 frames/sec.
  - RL doesn't require photorealism—only needs consistent visual features (edges, colors). Rasterization provides 5× speedup.
  - Use basic Phong shading, no global illumination (saves GPU cycles).

**3. Domain randomization (8 parameters)**

| **Parameter** | **Range** | **Update frequency** | **Justification** |
|---------------|-----------|----------------------|-------------------|
| **Warehouse layout** | 3 layout templates (grid, random, clustered shelves) | Per episode (every 1000 steps) | Prevents overfitting to specific aisle configurations |
| **Lighting intensity** | 200-2000 lux (dim warehouse to bright daylight) | Every 10 episodes | Simulates different times of day, warehouse conditions |
| **Lighting direction** | Dome light rotation 0-360° | Every 10 episodes | Prevents shadow memorization |
| **Obstacle positions** | Pallets: uniform within warehouse bounds | Per episode | Forces reactive obstacle avoidance (not memorized paths) |
| **Goal positions** | Uniform within 5-15m radius from start | Per episode | Covers short/medium/long navigation tasks |
| **Human agent behavior** | 3-5 agents, speed 0.5-1.5 m/s, random waypoints | Every 50 steps | Simulates dynamic obstacles (delivery workers) |
| **Camera noise** | Gaussian RGB noise σ=5/255, depth noise σ=1-3% | Per step | Matches real camera sensor noise |
| **Robot initial pose** | Position: uniform ±2m from center, yaw: 0-360° | Per episode | Prevents overfitting to specific starting configuration |

**Sim-to-real transfer benefit**: Randomization covers appearance (lighting, textures), geometry (layouts, obstacle positions), dynamics (human agents), and sensor (camera noise). This prevents overfitting to any single domain property, improving zero-shot real-world performance.

**4. Training pipeline**

**RL algorithm**: **PPO (Proximal Policy Optimization)**. Why:
- Sample-efficient (compared to DQN).
- Stable (clipped objective prevents policy collapse).
- Scales well to parallel environments (512 robots = 512× data collection rate).
- Widely used in robotics (proven on humanoid locomotion in prior work: Cassie, Atlas).

**Observation collection**:
- Each robot publishes observations to central trainer via Isaac Sim Python API (no ROS overhead during training):
  - RGB-D camera: 640×480 image (encoded to latent vector via pretrained CNN)
  - LiDAR: 2D laser scan (1024 points, downsampled to 64 for efficiency)
  - IMU: Linear velocity, angular velocity (6D vector)
  - Goal: Relative position to goal in robot frame (2D vector: distance, heading)
- Total observation: 512-D latent (camera) + 64 (LiDAR) + 6 (IMU) + 2 (goal) = **584-D vector**
- Batch size: 512 robots × 128 steps = **65,536 samples per PPO update**

**Synchronization**:
- **Synchronous**: All 512 robots step together (physics simulation advances by 20 ms for all environments simultaneously).
- Advantage: Ensures consistent batch sizes, simplifies PPO updates.
- Isaac Sim GPU parallelism handles synchronous stepping at 50 Hz (20 ms per step).

**Training time estimate**:
- Total timesteps: 10⁷
- Robots: 512
- Physics rate: 50 Hz
- Time per step: 1/50 = 0.02 sec
- Total steps per robot: 10⁷ / 512 = 19,531 steps
- Wall-clock time: 19,531 steps / 50 Hz = **390 seconds ≈ 6.5 minutes**
- (Accounting for PPO updates, rendering overhead: **~20-30 minutes total**)

**5. Sim-to-real deployment**

**Policy deployment**:
- Export trained policy (PyTorch → ONNX → TensorRT FP16).
- Deploy on real humanoid (Jetson Orin NX for onboard compute).
- Policy input: RGB-D camera + LiDAR + IMU (same sensors as simulation).
- Policy output: Target joint velocities (sent to low-level locomotion controller).

**Isaac ROS GEMs**:
- `isaac_ros_visual_slam`: Real-time localization (backup for goal-relative navigation).
- `isaac_ros_dnn_inference`: Run policy inference (TensorRT engine).
- `isaac_ros_image_proc`: Preprocess camera images (resize, normalize).

**Domain gap challenges (3 with mitigations)**:

1. **Challenge**: **Ground friction mismatch**. Simulated warehouse floor (μ=0.8) vs. real polished concrete (μ=0.6, slippery).
   - **Mitigation**: Randomize friction coefficient in sim (0.5-1.0) during training. Test policy on multiple real floor types (concrete, carpet, tile). Fine-tune policy with 100 real-world episodes if needed.

2. **Challenge**: **Actuator dynamics**. Simulated motors have perfect torque control; real motors have delays (5-10 ms) and backlash.
   - **Mitigation**: Add actuator delay model in sim: `torque_applied[t] = torque_commanded[t-1]` (one-step delay). Add Gaussian torque noise ±5%. Fine-tune PD gains on real robot (may need to reduce proportional gain by 20-30%).

3. **Challenge**: **Dynamic obstacles (humans)**. Simulated humans follow scripted waypoints; real humans are unpredictable (sudden stops, direction changes).
   - **Mitigation**: Train policy with **adversarial agents** in sim (randomly change direction every 2-5 sec, vary speed). Use Isaac Sim's behavior trees for realistic human motion. Deploy with safety margin: increase obstacle inflation radius by 10 cm in real-world costmap.

#### Grading Rubric (Total: 30 points)
- **Hardware requirements** (6 pts): GPU choice justified (A100 80GB), VRAM per robot calculated (scene + robot + textures), total cost estimated
- **Scene design** (5 pts): Warehouse description (layout, obstacles), USD instancing for memory efficiency, fidelity trade-off justified (rasterization vs. ray tracing)
- **Domain randomization** (6 pts): 8 parameters with ranges, update frequencies, justified benefits
- **Training pipeline** (7 pts): PPO algorithm justified, observation collection detailed (512-D vector), synchronous stepping, time estimate (6-30 min)
- **Sim-to-real deployment** (6 pts): Policy export (TensorRT), Isaac ROS GEMs listed, 3 domain gaps with specific mitigations

#### Common Mistakes
- Unrealistic VRAM estimates (claiming 1 GB per robot—too high, or 1 MB—too low)
- No justification for rasterization vs. ray tracing (saying "ray tracing is always better")
- Vague randomization (e.g., "randomize environment"—doesn't specify parameters)
- Time estimate ignores parallelism (calculates 10⁷ steps / 50 Hz = 200,000 sec, forgetting 512 robots)
- Generic domain gap mitigations ("add noise"—doesn't specify friction randomization, actuator delay, etc.)

---

## Summary

These instructor solutions provide:
- **Complete answers** with technical depth (equations, code snippets, quantitative values)
- **Grading rubrics** with point allocation and assessment criteria
- **Common mistakes** to help instructors identify student misconceptions
- **Extensions** for advanced students seeking deeper challenges

All solutions emphasize:
- **Quantitative reasoning**: Specific numbers (FPS, latency, dataset sizes) instead of vague claims
- **Technical accuracy**: Correct terminology (TensorRT optimizations, vSLAM pipeline stages)
- **Practical relevance**: Real-world constraints (hardware costs, real-time requirements, sim-to-real gaps)
- **System-level thinking**: Integration of multiple components (perception + planning + control)

Instructors should adjust grading based on course level (undergraduate vs. graduate) and student backgrounds (CS vs. Robotics vs. ML).
