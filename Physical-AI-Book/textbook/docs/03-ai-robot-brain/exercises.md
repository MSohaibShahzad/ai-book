---
title: "Module 3: Exercises"
slug: /ai-robot-brain/exercises
sidebar_label: "Exercises"
sidebar_position: 8
toc: true
description: "15 hands-on exercises for Module 3: Recall, Application, and Synthesis problems covering GPU acceleration, Isaac Sim synthetic data, Isaac ROS deployment, and perception pipelines."
---

# Module 3: GPU-Accelerated Perception & Isaac - Exercises

This page provides **15 progressive exercises** to reinforce the concepts from Module 3. Exercises are categorized by difficulty and learning objective:

- **📘 Recall (5 exercises)**: Test conceptual understanding (GPU vs CPU, ray tracing, TensorRT, vSLAM principles).
- **📗 Application (5 exercises)**: Hands-on implementation (Isaac Sim setup, Replicator scripts, Isaac ROS deployment, model conversion).
- **📕 Synthesis (5 exercises)**: System design and critical thinking (perception pipelines, sim-to-real strategies, footstep planner integration).

Each exercise includes:
- **Difficulty**: Easy / Moderate / Challenging
- **Time estimate**: 15 min to 3 hours
- **Deliverable**: Written answer, code/config file, or design document
- **Related chapter(s)**

---

## 📘 Recall Exercises (Conceptual Understanding)

### Exercise 1: GPU vs. CPU Performance Analysis (Chapter 1)

**Difficulty**: Easy
**Time estimate**: 25 minutes

**Prompt**:
A robotics team is deciding between Gazebo (CPU) and Isaac Sim (GPU) for training a locomotion policy. Create a comparison table addressing the following aspects:

| **Aspect** | **Gazebo (16-core CPU)** | **Isaac Sim (RTX 4090 GPU)** |
|------------|--------------------------|------------------------------|
| Physics rate (Hz) | | |
| Rendering (1080p) | | |
| LiDAR update (64×1024) | | |
| Parallel robots | | |
| Photorealism | | |
| Hardware cost | | |
| Primary use case | | |

For each cell, provide concise quantitative answers (e.g., "500 Hz", "20 FPS") and brief explanations where needed.

**Deliverable**: Completed comparison table with 7 rows × 3 columns (21 cells total).

**Assessment criteria**:
- Performance metrics match Chapter 1 specifications (±10% tolerance)
- Hardware costs are realistic ($0-$5000 range)
- Use cases clearly differentiate when to choose each tool
- Answers demonstrate understanding of parallelism and ray tracing benefits

---

### Exercise 2: Ray Tracing for Robotics (Chapter 2)

**Difficulty**: Easy
**Time estimate**: 20 minutes

**Prompt**:
A colleague claims: "Ray tracing is just for video games. Robotics doesn't need photorealistic rendering—Gazebo's rasterization is good enough."

Write a 250-word response addressing:

1. **What ray tracing provides** that rasterization does not (list 3 specific rendering features).
2. **Two robotics use cases** where photorealism is critical (be specific—don't say "vision models" without explaining why).
3. **One legitimate drawback** of ray tracing for robotics (e.g., computational cost, setup complexity).
4. **When Gazebo is actually sufficient** (give one concrete scenario).

**Deliverable**: Written response (250 words).

**Assessment criteria**:
- Ray tracing features are technically accurate (global illumination, reflections, refractions, shadows)
- Use cases justify why photorealism matters (e.g., sim-to-real transfer, HRI perception studies)
- Drawback and Gazebo use case show balanced understanding (not blind advocacy for GPU)

---

### Exercise 3: Domain Randomization Strategy (Chapter 3)

**Difficulty**: Moderate
**Time estimate**: 30 minutes

**Prompt**:
You are generating synthetic data for training a humanoid robot to detect and grasp coffee mugs in home kitchens. Design a domain randomization strategy by filling out this table:

| **Randomization Parameter** | **Range/Options** | **Justification (Why this helps sim-to-real transfer)** |
|-----------------------------|-------------------|----------------------------------------------------------|
| Lighting (dome light rotation) | | |
| Object texture (mug material) | | |
| Object pose (X, Y, Z position) | | |
| Camera focal length | | |
| Background clutter (# objects) | | |
| Surface reflectivity | | |

For each parameter:
- Specify realistic ranges (e.g., "0-360°", "5-20 objects")
- Explain how this randomization improves real-world generalization (1 sentence per justification)

**Deliverable**: Completed table with 6 rows × 3 columns.

**Assessment criteria**:
- Ranges are realistic and cover expected real-world variation
- Justifications correctly link randomization to overcoming specific sim-to-real gaps (e.g., "Lighting variation prevents overfitting to specific shadows")
- Covers appearance, geometry, and sensor parameters (not just one category)

---

### Exercise 4: TensorRT Optimization (Chapter 4)

**Difficulty**: Moderate
**Time estimate**: 25 minutes

**Prompt**:
Explain TensorRT's role in Isaac ROS by answering these questions:

1. **What TensorRT does**: Describe 3 specific optimizations TensorRT applies to neural networks (e.g., layer fusion, quantization). (1 sentence each)
2. **Performance impact**: If a YOLOv8 model runs at 5 FPS on CPU and 15 FPS on GPU (PyTorch), estimate the FPS with TensorRT optimization. Justify your estimate. (2-3 sentences)
3. **Trade-off**: What is one potential drawback of TensorRT optimization? (e.g., model accuracy, portability, debugging). (2 sentences)
4. **When to use**: Give one scenario where TensorRT is essential and one where it's unnecessary. (1 sentence each)

**Deliverable**: Written answers (approximately 300 words total).

**Assessment criteria**:
- Optimizations are technically accurate (layer fusion, INT8 quantization, kernel auto-tuning, etc.)
- FPS estimate is reasonable (2-5× speedup from TensorRT on top of GPU baseline)
- Trade-off demonstrates awareness of compression/accuracy relationship
- Scenarios show understanding of real-time constraints

---

### Exercise 5: Visual SLAM Principles (Chapter 5)

**Difficulty**: Moderate
**Time estimate**: 30 minutes

**Prompt**:
Humanoid robots use visual SLAM (vSLAM) instead of wheel odometry. Explain the vSLAM pipeline by answering:

1. **Why vSLAM?** Why can't humanoid robots rely on IMU alone for localization? What causes IMU drift? (2-3 sentences)
2. **Pipeline stages**: Describe the 5 stages of vSLAM (feature extraction → frame matching → pose estimation → map building → loop closure). For each stage, write 1 sentence explaining its function.
3. **Loop closure**: What is loop closure, and why is it critical for long-duration navigation? (2 sentences)
4. **Failure modes**: List 3 scenarios where vSLAM fails or degrades (e.g., low-texture environments, motion blur). For each, explain why (1 sentence per scenario).

**Deliverable**: Written answers (approximately 400 words).

**Assessment criteria**:
- IMU drift explanation is quantitatively grounded (e.g., "1% error accumulates over distance")
- Pipeline stages are correctly described in sequence
- Loop closure explanation demonstrates understanding of drift correction
- Failure modes are realistic and technically justified

---

## 📗 Application Exercises (Hands-On Implementation)

### Exercise 6: Isaac Sim Scene Setup (Chapter 2)

**Difficulty**: Moderate
**Time estimate**: 1.5 hours

**Prompt**:
Create a basic Isaac Sim scene for a humanoid robot in a kitchen environment.

**Requirements**:
1. Import a humanoid robot USD model (use Isaac Sim's example assets or a custom URDF converted to USD).
2. Add a kitchen environment with:
   - Floor (10m × 10m)
   - 1 table, 4 chairs, 1 counter
   - Dome light with HDR environment map (any realistic indoor HDRI)
3. Mount an RGB-D camera on the robot's head (`/base_link` → `head_link` → camera at X=0.1m, Y=0, Z=0).
4. Configure the camera: 640×480 resolution, 60° horizontal FOV, 30 Hz.
5. Enable ray tracing (RTX mode) and verify photorealistic rendering.

**Deliverable**:
1. Screenshot of the Isaac Sim scene (showing robot + kitchen from camera view).
2. USD file (or instructions to reproduce the scene).
3. Camera configuration snippet (Python script or Isaac Sim GUI settings).

**Assessment criteria**:
- Scene includes all required objects and is realistically arranged
- Camera is correctly mounted and configured (verify FOV, resolution, framerate)
- Ray tracing is enabled (screenshot shows realistic shadows, reflections)
- Instructions are clear and reproducible

---

### Exercise 7: Replicator Script for Synthetic Data (Chapter 3)

**Difficulty**: Moderate
**Time estimate**: 2 hours

**Prompt**:
Write a Python script using Isaac Sim's Replicator API to generate 1,000 labeled images of a table with randomized objects.

**Requirements**:
1. Scene: Table with 5-10 objects (mugs, bowls, tools—use Isaac Sim's prop library).
2. Randomization (per frame):
   - Object positions: uniform within table bounds (X: -0.4 to 0.4 m, Y: -0.3 to 0.3 m, Z: table height)
   - Object rotations: uniform yaw (0-360°)
   - Lighting: rotate dome light (0-360°)
   - Camera pose: random positions within hemisphere above table (radius 1-2 m, looking at table center)
3. Annotations: 2D bounding boxes (COCO format)
4. Output: 1,000 RGB images + COCO JSON annotations

**Deliverable**:
1. Python script (`generate_data.py`, 100-150 lines).
2. Sample output: 3 images + corresponding COCO JSON snippet (for verification).
3. README: Instructions to run the script in Isaac Sim.

**Assessment criteria**:
- Script uses Replicator API correctly (randomizers, writer, orchestrator)
- Randomization ranges cover realistic variation (no objects floating, no camera inside table)
- COCO JSON format is valid (matches COCO schema)
- Script generates all 1,000 images without errors
- README provides clear execution steps

**Hint**: Refer to Chapter 3's Replicator example for structure.

---

### Exercise 8: Isaac ROS Visual SLAM Launch (Chapter 4)

**Difficulty**: Moderate
**Time estimate**: 1.5 hours

**Prompt**:
Create a ROS 2 launch file to run Isaac ROS visual SLAM with a simulated humanoid robot in Isaac Sim.

**Requirements**:
1. Launch Isaac Sim with a robot (use any humanoid model with an RGB-D camera).
2. Launch `isaac_ros_visual_slam` node with:
   - Input topics: `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/imu`
   - Output topics: `/visual_slam/tracking/odometry`, `/visual_slam/tracking/vo_pose`
   - Enable IMU fusion and loop closure
3. Launch RViz with displays for:
   - Camera image (`/camera/rgb/image_raw`)
   - SLAM trajectory (`/visual_slam/tracking/vo_pose` as PoseStamped)
   - Point cloud map (`/visual_slam/tracking/slam_map` if available)
4. Use `use_sim_time:=true` for all nodes.
5. Test: Move the robot (teleop or scripted motion) and verify SLAM tracks the pose.

**Deliverable**:
1. Launch file (`vslam_demo.launch.py`, 80-120 lines).
2. RViz config file (`vslam_demo.rviz`).
3. Screenshot or short video showing: robot moving + SLAM trajectory in RViz.
4. README: Setup instructions and how to run.

**Assessment criteria**:
- Launch file correctly starts Isaac Sim bridge, Isaac ROS vSLAM, and RViz
- Topic remappings match Isaac Sim's output and Isaac ROS vSLAM's input
- RViz displays show SLAM data (trajectory, map points)
- SLAM successfully tracks robot motion (no divergence, reasonable accuracy)
- Documentation is clear and tested

---

### Exercise 9: Convert PyTorch Model to TensorRT (Chapter 4)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
Convert a YOLOv8 model (trained on COCO) to TensorRT and benchmark the speedup.

**Steps**:
1. Export YOLOv8 to ONNX format:
   ```bash
   yolo export model=yolov8n.pt format=onnx
   ```
2. Convert ONNX to TensorRT engine using `trtexec`:
   ```bash
   trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n.trt --fp16
   ```
3. Benchmark inference latency:
   - Run 100 inferences on 640×640 images (use dummy data or COCO images)
   - Measure average latency (ms) for:
     - PyTorch (CPU)
     - PyTorch (GPU)
     - TensorRT (GPU, FP16)
4. Create a comparison table of latency and FPS.

**Deliverable**:
1. Conversion scripts (`export_onnx.py`, `convert_trt.sh`).
2. Benchmark script (`benchmark_inference.py`) that times each model type.
3. Results table:
   | **Model Type** | **Latency (ms)** | **FPS** | **Speedup vs CPU** |
   |----------------|------------------|---------|--------------------|
   | PyTorch CPU | | | 1.0× |
   | PyTorch GPU | | | |
   | TensorRT FP16 | | | |
4. One-paragraph analysis (e.g., "TensorRT achieves 12× speedup over CPU by...").

**Assessment criteria**:
- Conversion commands are correct and produce valid TensorRT engine
- Benchmark script measures latency accurately (averaged over 100 runs, excludes warmup)
- Results table shows clear performance progression (CPU < GPU < TensorRT)
- Analysis explains sources of speedup (GPU parallelism, TensorRT optimizations, FP16 precision)

---

### Exercise 10: Isaac ROS DNN Inference Pipeline (Chapter 4)

**Difficulty**: Challenging
**Time estimate**: 2.5 hours

**Prompt**:
Create a ROS 2 pipeline that uses Isaac ROS DNN Inference to run object detection on a live camera feed.

**Requirements**:
1. Use a TensorRT-optimized YOLOv8 model (from Exercise 9, or use a pre-trained one).
2. Set up Isaac ROS DNN Inference node:
   - Input: `/camera/rgb/image_raw` (from Isaac Sim or real camera)
   - Output: `/detections` (bounding boxes in `vision_msgs/Detection2DArray`)
3. Visualize detections in RViz (overlay bounding boxes on camera image).
4. Measure and log inference latency (time from image received to detections published).
5. Test on a scene with 3-5 objects (from COCO classes: person, chair, cup, etc.).

**Deliverable**:
1. Launch file (`dnn_inference.launch.py`, 100-150 lines).
2. RViz config file with image + bounding box display.
3. Latency measurement node (`latency_monitor.py`): subscribes to camera and detections, computes latency.
4. Test results:
   - Screenshot showing detections overlaid on camera image
   - Latency statistics (mean, std, min, max over 100 frames)
5. README with setup and execution instructions.

**Assessment criteria**:
- Isaac ROS DNN Inference node runs without errors and publishes detections
- Detections are accurate (correct classes, reasonable bounding boxes)
- Latency is measured correctly (image timestamp → detection timestamp)
- Latency is real-time (< 50 ms for 640×480 images on RTX or Jetson)
- Visualization in RViz is functional and clear

---

## 📕 Synthesis Exercises (System Design & Critical Thinking)

### Exercise 11: Design a Sim-to-Real Perception Pipeline (Chapters 2, 3, 4)

**Difficulty**: Moderate
**Time estimate**: 1.5 hours

**Prompt**:
Design an end-to-end pipeline for training and deploying a humanoid robot's object detection system using Isaac Sim and Isaac ROS.

**Answer the following**:

1. **Training data generation** (Isaac Sim):
   - Describe the scene setup (environment, objects, camera placement).
   - Specify 5 domain randomization parameters with ranges (lighting, textures, poses, etc.).
   - How many images will you generate? Justify the dataset size.

2. **Model training**:
   - What model architecture (YOLO, Faster R-CNN, etc.)? Why?
   - What training framework (PyTorch, TensorFlow)?
   - What hyperparameters (learning rate, batch size, epochs)?

3. **Deployment** (Isaac ROS):
   - How will you convert the model to TensorRT?
   - What Isaac ROS GEMs will you use (DNN Inference, image preprocessing)?
   - What hardware (Jetson Orin, RTX GPU)?

4. **Validation**:
   - How will you measure sim-to-real gap? (e.g., mAP on synthetic vs. real data)
   - What 3 real-world tests will validate the system?

5. **Mitigation strategies**:
   - If real-world performance is poor (e.g., mAP drops from 85% to 60%), what 3 steps will you take?

**Deliverable**: Design document (700-900 words) organized by the 5 sections above.

**Assessment criteria**:
- Training data plan covers diverse scenes and adequate randomization
- Model choice is justified by task requirements (real-time constraints, accuracy needs)
- Deployment plan includes all necessary steps (conversion, ROS integration, hardware)
- Validation metrics are quantitative and realistic
- Mitigation strategies are specific and actionable (not vague like "collect more data")

---

### Exercise 12: Analyze Isaac Sim vs. Real-World Depth Camera Fidelity (Chapters 2, 3)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
A team trained a grasping policy using depth images from Isaac Sim (simulated RealSense D435). In simulation, the policy succeeds 90% of the time. On the real robot, success drops to 65%.

**Tasks**:

1. **Identify 5 potential causes** of the depth sensor sim-to-real gap. For each cause:
   - Describe the gap (e.g., "Simulated depth has no flying pixels at edges").
   - Explain why this matters for grasping (e.g., "Flying pixels corrupt grasp pose estimation").

2. **Propose 5 mitigation strategies**, one per cause:
   - Be specific (e.g., "Add Gaussian noise with σ = 1.5 cm depth-dependent: σ(d) = 0.01 + 0.0005×d²").
   - Estimate implementation difficulty (Easy / Moderate / Hard) and impact (Low / Medium / High).

3. **Design an experiment** to validate the most promising mitigation:
   - Test setup: What scene (objects, distances, lighting) will you use in sim and real?
   - Data collection: How many depth images? What metrics (MAE, edge artifact density, etc.)?
   - Success criteria: What improvement indicates the mitigation worked? (e.g., "Success rate increases from 65% to 80%")

**Deliverable**: Analysis report (800-1000 words) with:
- Table of 5 causes, mitigation strategies, and impact estimates (with justifications)
- Detailed experiment design (1 paragraph each: setup, data collection, success criteria)

**Assessment criteria**:
- Causes are technically accurate and specific to depth cameras (not generic vision issues)
- Mitigation strategies are actionable (specific Isaac Sim parameters, post-processing steps)
- Impact estimates are justified with reasoning (e.g., "High impact because grasping relies on edge detection")
- Experiment design is reproducible and tests a concrete hypothesis

---

### Exercise 13: Design a Footstep Planner with Isaac ROS and Nav2 (Chapter 5)

**Difficulty**: Challenging
**Time estimate**: 2.5 hours

**Prompt**:
Humanoid robots cannot use Nav2's default differential-drive controller—they need a **footstep planner** that generates discrete foot placements. Design a perception-navigation pipeline integrating Isaac ROS vSLAM and Nav2 with a custom footstep planner.

**Answer the following**:

1. **Perception pipeline**:
   - What Isaac ROS GEMs will you use for localization? (vSLAM, AprilTag, etc.)
   - What sensors (RGB-D, LiDAR, IMU) and update rates (Hz)?
   - How will you handle loop closure for long-duration navigation?

2. **Costmap configuration**:
   - What layers will your costmap include? (static obstacles, dynamic obstacles, inflation layer, etc.)
   - How will you represent bipedal footprint? (standard radius, two foot rectangles, etc.)
   - What inflation radius around obstacles? (justify based on step length, robot width)

3. **Footstep planner design**:
   - Describe the planning algorithm at a high level (e.g., A* search over discrete foot placements).
   - What are the constraints? (step length, step height, balance limits, no-penetration with obstacles)
   - How will the planner interface with Nav2? (custom Nav2 plugin, external node, etc.)

4. **Locomotion controller**:
   - Once footsteps are planned, how does the robot execute them? (reference to Module 2 locomotion controller, or describe ZMP-based walking)
   - How will you handle dynamic replanning if an obstacle moves?

5. **Failure modes and recovery**:
   - List 3 failure scenarios (e.g., "vSLAM loses tracking in low-light", "Footstep planner finds no valid path").
   - For each, propose a recovery behavior (e.g., "Switch to IMU-only odometry for 5 seconds, reduce walking speed").

**Deliverable**: Design document (1000-1200 words) with:
- Architecture diagram (hand-drawn or digital: ROS nodes, topics, data flow)
- Answers to all 5 questions (organized as sections)

**Assessment criteria**:
- Perception pipeline is realistic (sensor choices, update rates match Isaac ROS capabilities)
- Costmap configuration accounts for bipedal geometry (not just treating humanoid as wheeled robot)
- Footstep planner design is feasible (algorithm, constraints, integration method)
- Locomotion controller integration is explained (not just "it executes the plan")
- Failure mode analysis is thorough with specific recovery behaviors

---

### Exercise 14: Evaluate Synthetic Data Quality for VLA Training (Chapter 3)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
You are generating 50,000 synthetic images in Isaac Sim to train a vision-language-action (VLA) policy for humanoid manipulation (tasks like "Pick up the red mug", "Open the top drawer"). Design a quality evaluation protocol to ensure the synthetic data will generalize to real-world deployment.

**Tasks**:

1. **Diversity metrics**: Define 5 quantitative metrics to measure dataset diversity. Examples:
   - Object category distribution (# images per class)
   - Lighting histogram (distribution of pixel intensities across dataset)
   - Pose variety (variance in object 6-DOF poses)
   - Background complexity (# unique textures, clutter levels)
   - Camera viewpoint coverage (spherical distribution of camera positions)

   For each metric, specify:
   - How to compute it (equation or algorithm)
   - Target value (e.g., "At least 500 images per object class")

2. **Realism assessment**: How will you validate that synthetic images are photorealistic enough? Propose 3 tests:
   - Example: "Train a binary classifier (real vs. synthetic images). If accuracy < 60%, data is realistic."
   - For each test, describe: dataset, evaluation protocol, success threshold.

3. **Domain gap mitigation**: List 3 synthetic artifacts that might not occur in real images (e.g., "Perfect edges, no motion blur"). For each:
   - Explain why it's a problem (how it could cause real-world failure)
   - Propose a fix (Replicator randomization, post-processing, etc.)

4. **VLA-specific considerations**: VLA policies require language annotations. How will you generate them?
   - Automatic (from scene graph: "red mug at position X") or manual?
   - What annotation format (natural language, structured commands)?
   - How many language variations per scene (e.g., "Pick the red cup" vs. "Grab the crimson mug")?

**Deliverable**: Quality evaluation protocol (900-1100 words) with:
- Diversity metrics table (5 metrics, computation methods, target values)
- Realism assessment plan (3 tests with protocols and thresholds)
- Domain gap mitigation table (3 artifacts, problems, fixes)
- VLA annotation strategy (automation method, format, variation)

**Assessment criteria**:
- Diversity metrics are quantitative and comprehensive (cover appearance, geometry, viewpoints)
- Realism tests are objective (not subjective human inspection)
- Domain gap mitigation addresses specific artifacts with actionable fixes
- VLA annotation strategy scales to 50,000 images (not manual labeling)

---

### Exercise 15: Design a Multi-Robot Isaac Sim Training Environment (Chapters 1, 2, 3)

**Difficulty**: Challenging
**Time estimate**: 3 hours

**Prompt**:
You are training a reinforcement learning policy for humanoid warehouse navigation (avoid dynamic obstacles, reach goal). Design a **massively parallel Isaac Sim training environment** with 512 robots training simultaneously.

**Answer the following**:

1. **Hardware requirements**:
   - What GPU(s) will you use? (Consider VRAM for 512 robots)
   - Estimate VRAM per robot (scene geometry, physics state, textures)
   - Total VRAM needed? (512 × per-robot cost)
   - What about CPU cores, RAM?

2. **Scene design**:
   - Describe the warehouse environment (dimensions, obstacles, randomization)
   - How will you ensure 512 independent environments don't interfere? (separate USD stages, instancing, etc.)
   - What level of visual fidelity? (photorealistic ray tracing vs. fast rasterization—justify trade-off)

3. **Domain randomization**:
   - List 8 randomization parameters (layout, lighting, obstacles, goal positions, human agents, etc.)
   - For each, specify ranges and update frequency (per episode, per 10 episodes, etc.)
   - How will randomization improve real-world transfer?

4. **Training pipeline**:
   - What RL algorithm (PPO, SAC, etc.)? Why?
   - How will observations be collected from 512 robots? (ROS topics, Isaac Sim APIs?)
   - How often will you synchronize robots (all step together, or async)?
   - Estimate training time: 10⁷ timesteps, 512 robots, 50 Hz → ? hours

5. **Sim-to-real deployment**:
   - After training in Isaac Sim, how will you deploy the policy on a real humanoid?
   - What Isaac ROS GEMs will you use for perception?
   - What domain gap challenges do you anticipate? (List 3 with mitigation plans)

**Deliverable**: System design document (1200-1500 words) with:
- Hardware spec sheet (GPU, CPU, RAM, estimated costs)
- Scene design description (with layout diagram or reference images)
- Domain randomization table (8 parameters, ranges, frequencies, justifications)
- Training pipeline description (algorithm, data flow, timeline estimate)
- Deployment plan (Isaac ROS integration, domain gap mitigations)

**Assessment criteria**:
- Hardware requirements are realistic (VRAM estimates account for scene complexity, not arbitrary)
- Scene design balances fidelity and speed (photorealism is expensive; justify trade-off)
- Domain randomization covers diverse aspects (not just lighting—also layout, dynamics, sensors)
- Training pipeline is efficient (leverages GPU parallelism, reasonable time estimate)
- Sim-to-real plan addresses concrete gaps (e.g., "Add motion blur to sim images", "Fine-tune on 1,000 real images")

---

## Summary

These 15 exercises progress from conceptual understanding (Recall) to hands-on implementation (Application) to system design (Synthesis). Completing all exercises will ensure mastery of:

- **GPU acceleration concepts**: Parallelism, ray tracing, physics on GPU, performance trade-offs
- **Isaac Sim skills**: Scene creation, Replicator API, synthetic data generation, domain randomization
- **Isaac ROS deployment**: Visual SLAM, DNN inference, TensorRT optimization, real-time perception
- **Perception pipelines**: Sensor fusion, costmap integration, footstep planning, sim-to-real transfer
- **Critical thinking**: Domain gap analysis, system design for RL, multi-robot training, quality evaluation

Recommended progression:
1. **Week 1**: Recall exercises (1-5) to solidify GPU and Isaac concepts.
2. **Week 2**: Application exercises (6-10) to build hands-on Isaac Sim and Isaac ROS skills.
3. **Week 3**: Synthesis exercises (11-15) to develop end-to-end perception system design expertise.

---

**Navigation**
← [Examples](/ai-robot-brain/examples)
→ [Module 4: Vision-Language-Action Models](/vla) (coming soon)
