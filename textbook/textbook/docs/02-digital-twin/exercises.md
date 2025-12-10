---
title: "Module 2: Exercises"
slug: /digital-twin/exercises
sidebar_label: "Exercises"
sidebar_position: 8
toc: true
description: "15 hands-on exercises for Module 2: Recall, Application, and Synthesis problems covering simulation concepts, Gazebo configuration, Unity rendering, and sensor simulation."
---

# Module 2: Digital Twin & Simulation - Exercises

This page provides **15 progressive exercises** to reinforce the concepts from Module 2. Exercises are categorized by difficulty and learning objective:

- **📘 Recall (5 exercises)**: Test conceptual understanding (definitions, comparisons, explanations).
- **📗 Application (5 exercises)**: Hands-on implementation (configure sensors, run simulations, visualize data).
- **📕 Synthesis (5 exercises)**: System design and critical thinking (design simulations, analyze trade-offs, propose solutions).

Each exercise includes:
- **Difficulty**: Easy / Moderate / Challenging
- **Time estimate**: 15 min to 3 hours
- **Deliverable**: Written answer, code/config file, or design document
- **Related chapter(s)**

---

## 📘 Recall Exercises (Conceptual Understanding)

### Exercise 1: Simulation Value Proposition (Chapter 1)

**Difficulty**: Easy
**Time estimate**: 20 minutes

**Prompt**:
You are presenting the case for simulation to a hardware-focused robotics team that prefers physical testing. Write a 300-word memo addressing:

1. The four primary benefits of simulation for humanoid robotics (safety, cost, iteration speed, scalability).
2. One concrete example for each benefit (with quantitative estimates where possible, e.g., "Physical testing costs $X per iteration vs. simulation costs $Y").
3. Acknowledge the reality gap and propose one mitigation strategy.

**Deliverable**: Written memo (300 words).

**Assessment criteria**:
- All four benefits clearly explained
- Examples are specific to humanoid robotics (not generic)
- Reality gap is acknowledged with at least one mitigation strategy

---

### Exercise 2: Physics-Based vs. Kinematic Simulation (Chapter 2)

**Difficulty**: Easy
**Time estimate**: 15 minutes

**Prompt**:
Create a comparison table with the following columns:

| **Aspect** | **Kinematic Simulation** | **Physics-Based Simulation** |
|------------|--------------------------|------------------------------|
| Speed (Hz) | | |
| Models gravity? | | |
| Models contact forces? | | |
| Use case 1 | | |
| Use case 2 | | |
| Example tool | | |

Fill in each cell with concise answers (1-2 sentences max per cell).

**Deliverable**: Completed comparison table.

**Assessment criteria**:
- Accurate technical details (speed, capabilities)
- Use cases are distinct and appropriate
- Example tools are correct (e.g., RViz for kinematic, Gazebo for physics-based)

---

### Exercise 3: Gazebo Physics Engines (Chapter 3)

**Difficulty**: Easy
**Time estimate**: 20 minutes

**Prompt**:
Gazebo supports four physics engines: ODE, Bullet, Simbody, and DART. For each engine, answer:

1. **Primary strength**: What is it best at? (1 sentence)
2. **Typical use case**: When would you choose this engine? (1 sentence)
3. **Limitation**: What is one drawback compared to other engines? (1 sentence)

**Deliverable**: Written answers (4 engines × 3 questions = 12 sentences).

**Assessment criteria**:
- Strengths, use cases, and limitations are technically accurate
- Answers reflect understanding of physics simulation trade-offs (accuracy vs. speed)

---

### Exercise 4: Unity for HRI (Chapter 4)

**Difficulty**: Easy
**Time estimate**: 25 minutes

**Prompt**:
A research lab wants to study how humans react to a humanoid robot's approach behavior (speed, distance, gaze). The lab is debating whether to use Gazebo or Unity for the study. Write a 200-word recommendation addressing:

1. Why Unity is better suited for this HRI study (list 3 specific features).
2. What Gazebo cannot provide that Unity can (list 2 limitations of Gazebo).
3. One potential drawback of using Unity (e.g., setup complexity, physics fidelity).

**Deliverable**: Written recommendation (200 words).

**Assessment criteria**:
- Unity advantages are specific to HRI (not generic graphics praise)
- Gazebo limitations are accurate
- Acknowledges at least one Unity trade-off

---

### Exercise 5: Sensor Noise Models (Chapter 5)

**Difficulty**: Moderate
**Time estimate**: 30 minutes

**Prompt**:
You are configuring sensor noise models for a humanoid robot simulation. For each sensor below, specify:

1. **Noise type** (Gaussian, Poisson, outliers, etc.)
2. **Typical parameter values** (mean, standard deviation, or rate)
3. **One real-world effect this noise simulates** (e.g., "thermal drift", "multi-path reflections")

Sensors:
- 2D LiDAR (distance measurement)
- Depth camera (RGB-D sensor)
- IMU gyroscope (angular velocity)
- RGB camera (pixel intensity)

**Deliverable**: Table with 4 rows (one per sensor) and 3 columns (noise type, parameters, real-world effect).

**Assessment criteria**:
- Noise types and parameters are technically accurate (match Chapter 5 specifications)
- Real-world effects are correctly attributed to the sensor type

---

## 📗 Application Exercises (Hands-On Implementation)

### Exercise 6: Configure a 3D LiDAR Sensor in Gazebo (Chapter 5)

**Difficulty**: Moderate
**Time estimate**: 1 hour

**Prompt**:
Create a Gazebo SDF sensor plugin for a 3D LiDAR with the following specifications:

- **Model**: Ouster OS1-64 (64 vertical beams)
- **Horizontal resolution**: 1024 samples (360° / 1024 = 0.35°)
- **Vertical range**: -22.5° to +22.5° (45° total FOV)
- **Update rate**: 10 Hz
- **Range**: 0.5 m to 120 m
- **Noise**: Gaussian, σ = 2 cm

**Deliverable**:
1. Complete SDF sensor XML snippet (200-300 lines).
2. ROS 2 topic name where point cloud will be published.
3. RViz command to visualize the point cloud.

**Assessment criteria**:
- XML syntax is valid (can be parsed by Gazebo)
- Sensor parameters match specifications exactly
- ROS 2 topic name follows standard conventions (e.g., `/robot/lidar/points`)
- RViz visualization command is correct

**Hint**: Refer to Example 2 in the Examples page for structure.

---

### Exercise 7: Add a Depth Camera to a URDF Robot (Chapter 3)

**Difficulty**: Moderate
**Time estimate**: 1.5 hours

**Prompt**:
You have a humanoid robot URDF file with a `head_link`. Add an Intel RealSense D435-like depth camera sensor using Gazebo plugins. The sensor should:

- Be mounted 10 cm in front of the `head_link` (X = 0.1 m, Y = 0, Z = 0)
- Publish RGB images (640×480, 30 Hz) to `/robot/camera/rgb/image_raw`
- Publish depth maps (640×480, 30 Hz) to `/robot/camera/depth/image_raw`
- Publish point clouds to `/robot/camera/depth/points`
- Have a horizontal FOV of 69.4° (RealSense D435 spec)
- Have depth range: 0.3 m to 10 m
- Include Gaussian noise: σ = 1.5 cm

**Deliverable**:
1. URDF snippet with `<gazebo>` tags (100-150 lines).
2. Screenshot or description of how to verify the camera works (e.g., RViz setup).

**Assessment criteria**:
- URDF syntax is valid (can be parsed by `robot_state_publisher`)
- Sensor parameters match specifications
- All three topics are correctly remapped
- Verification method is clear and executable

---

### Exercise 8: Launch Gazebo + RViz with Sensor Visualization (Chapters 3, 5)

**Difficulty**: Moderate
**Time estimate**: 1 hour

**Prompt**:
Create a ROS 2 Python launch file (`sensor_demo.launch.py`) that:

1. Starts Gazebo with a world file (`sensor_world.world` - you can create a minimal one or use Example 1).
2. Spawns a robot with at least:
   - 2D LiDAR (publishes to `/robot/scan`)
   - RGB camera (publishes to `/robot/camera/image_raw`)
   - IMU (publishes to `/robot/imu`)
3. Launches RViz with pre-configured displays for all three sensors.
4. Uses `use_sim_time:=true` for all nodes.

**Deliverable**:
1. Complete launch file (`sensor_demo.launch.py`).
2. Minimal world file (`sensor_world.world`) if not using Example 1.
3. RViz config file (`sensor_demo.rviz`) with LaserScan, Image, and Imu displays.
4. Instructions to run: `ros2 launch <package> sensor_demo.launch.py`.

**Assessment criteria**:
- Launch file runs without errors
- All sensors publish data (verified with `ros2 topic list` and `ros2 topic echo`)
- RViz displays show sensor data correctly
- Documentation is clear and reproducible

---

### Exercise 9: Measure LiDAR Accuracy in Simulation (Chapter 5)

**Difficulty**: Moderate
**Time estimate**: 1.5 hours

**Prompt**:
Test the accuracy of a simulated 2D LiDAR by placing a flat wall at known distances and measuring the LiDAR readings.

**Steps**:
1. Create a Gazebo world with a robot (with 2D LiDAR) and a large vertical wall.
2. Position the wall at 3 distances: 2 m, 5 m, and 10 m from the robot.
3. For each distance:
   - Record 100 LiDAR scans (subscribe to `/robot/scan`).
   - Extract the subset of rays that hit the wall (filter by angle).
   - Compute mean and standard deviation of distances.
4. Compare measured distances to ground-truth (2 m, 5 m, 10 m).
5. Analyze: Does noise increase with distance? By how much?

**Deliverable**:
1. Python script to collect and analyze data (`lidar_accuracy_test.py`).
2. Table of results:
   | **True Distance (m)** | **Measured Mean (m)** | **Measured Std (m)** | **Error (m)** |
   |-----------------------|-----------------------|----------------------|---------------|
   | 2.0 | | | |
   | 5.0 | | | |
   | 10.0 | | | |
3. One-paragraph analysis of the results.

**Assessment criteria**:
- Script correctly subscribes to LaserScan topic and filters relevant rays
- Measurements are averaged over 100 scans (not just 1)
- Results table shows mean, std, and error
- Analysis identifies noise trends (e.g., "Noise increases with distance, consistent with Gaussian noise model")

---

### Exercise 10: Unity Scene with ROS Integration (Chapter 4)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
Create a Unity scene that subscribes to ROS 2 joint states and visualizes a humanoid robot in real-time.

**Requirements**:
1. Import a humanoid robot URDF into Unity (use the URDF Importer package).
2. Create a C# script (`JointStateSubscriber.cs`) that:
   - Connects to ROS via ROS-TCP Connector.
   - Subscribes to `/robot/joint_states`.
   - Updates the robot's ArticulationBody joint targets based on incoming data.
3. Add a simple environment (floor, one light source).
4. Test by publishing joint states from ROS 2:

   ```bash
   ros2 topic pub /robot/joint_states sensor_msgs/msg/JointState \
     "{header: {frame_id: 'base_link'}, \
       name: ['left_knee', 'right_knee'], \
       position: [0.5, 0.5]}"
   ```

5. Verify that the robot's knees bend to 0.5 radians (~28.6°) in Unity.

**Deliverable**:
1. Unity project folder (or ZIP file).
2. C# script (`JointStateSubscriber.cs`).
3. Short video (or screenshots) showing the robot moving in Unity when ROS publishes joint states.
4. README with setup instructions.

**Assessment criteria**:
- Unity scene runs without errors
- ROS-TCP Connector successfully connects to ROS 2
- Robot joints move in response to joint state messages
- Video/screenshots demonstrate real-time synchronization

---

## 📕 Synthesis Exercises (System Design & Critical Thinking)

### Exercise 11: Design a Simulation Strategy for Stair Climbing (Chapters 1, 2, 3)

**Difficulty**: Moderate
**Time estimate**: 1 hour

**Prompt**:
You are developing a locomotion controller for a humanoid robot to climb stairs. Design a simulation-based testing strategy that balances speed, realism, and safety.

**Answer the following**:

1. **Simulation environment**: Describe the Gazebo world (stair geometry, surface properties, lighting).
2. **Physics parameters**: What timestep, physics engine, and contact parameters would you use? Justify your choices.
3. **Test scenarios**: List 5 test scenarios with increasing difficulty (e.g., "Flat stairs with friction μ = 1.0", "Wet stairs with μ = 0.3", "Irregular step heights ±2 cm").
4. **Metrics**: What quantitative metrics would you measure to evaluate the controller? (e.g., success rate, energy consumption, time to climb).
5. **Sim-to-real validation**: After simulation testing, what 3 tests would you run on the physical robot to validate the controller?

**Deliverable**: Design document (500-700 words, organized by the 5 sections above).

**Assessment criteria**:
- Simulation environment is realistic and appropriate for stair climbing
- Physics parameters are justified (not arbitrary)
- Test scenarios cover a range of difficulties and failure modes
- Metrics are quantitative and relevant to locomotion performance
- Sim-to-real validation plan is practical and tests critical assumptions

---

### Exercise 12: Analyze Sim-to-Real Gap for Vision Models (Chapters 4, 5)

**Difficulty**: Challenging
**Time estimate**: 1.5 hours

**Prompt**:
A team trained a YOLOv8 object detector on 50,000 synthetic images (Unity) to detect humans, chairs, and tables in home environments. In simulation, the detector achieves 85% mAP (mean Average Precision). When deployed on a real humanoid robot, mAP drops to 60%.

**Tasks**:

1. **Identify 5 possible causes** of the sim-to-real gap (related to rendering, sensor simulation, or dataset diversity).
2. **For each cause**, propose a specific mitigation strategy.
3. **Estimate the impact** of each mitigation (qualitative: "High impact", "Moderate impact", "Low impact") and justify.
4. **Design an experiment** to test the most promising mitigation strategy (describe dataset, training procedure, and evaluation protocol).

**Deliverable**: Analysis report (600-800 words) with:
- Table: 5 causes, 5 mitigations, 5 impact estimates (with justifications)
- Detailed experiment design (1 paragraph per: dataset, training, evaluation)

**Assessment criteria**:
- Causes are technically accurate and specific to vision/simulation
- Mitigation strategies are actionable (not vague, e.g., "improve rendering" is too vague; "add lens distortion and motion blur" is specific)
- Impact estimates are justified with reasoning (not arbitrary)
- Experiment design is detailed enough to be reproducible

---

### Exercise 13: Design a Hybrid Gazebo-Unity Simulation (Chapters 3, 4)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
You are building a humanoid robot for warehouse telepresence. Operators in a control room need to:
- See a photorealistic view of the robot's environment (for situational awareness).
- Control the robot in real-time (low latency, accurate physics).

Design a **hybrid simulation architecture** that uses both Gazebo (for physics) and Unity (for visualization).

**Answer the following**:

1. **Architecture diagram**: Draw or describe the data flow between ROS 2, Gazebo, Unity, and the operator interface. Which components run on which machines?
2. **Data exchange**: What ROS 2 topics are published/subscribed by Gazebo, Unity, and the operator interface? (List at least 6 topics.)
3. **Latency considerations**: What is the acceptable latency for each data path (e.g., operator command → robot action, robot camera → Unity rendering)? How would you minimize latency?
4. **Scalability**: If you need to simulate 10 robots simultaneously, how would you scale this architecture? (Consider CPU/GPU requirements, network bandwidth.)
5. **Failure modes**: What happens if Unity crashes? What happens if the ROS-TCP connection drops? Propose fallback strategies.

**Deliverable**: Design document (800-1000 words) with:
- Architecture diagram (hand-drawn scan or digital diagram)
- Answers to all 5 questions (organized as sections)

**Assessment criteria**:
- Architecture diagram is clear and shows all components + data flow
- Data exchange topics are correctly identified (match ROS 2 conventions)
- Latency analysis is quantitative (e.g., "Camera rendering must have &lt;100 ms latency for VR usability")
- Scalability plan addresses resource constraints (CPU, GPU, network)
- Failure mode analysis is thorough and includes fallback strategies

---

### Exercise 14: Evaluate Sensor Simulation Fidelity (Chapter 5)

**Difficulty**: Challenging
**Time estimate**: 2 hours

**Prompt**:
You have access to a real humanoid robot with an Intel RealSense D435 depth camera and a simulated version in Gazebo. Design an experimental protocol to quantify the **fidelity** of the depth camera simulation (how closely it matches real sensor behavior).

**Tasks**:

1. **Experimental setup**: Describe the physical scene (objects, distances, lighting) you would use for both real and simulated tests. The scene must be reproducible (measurable).
2. **Data collection**: What data would you collect from both real and simulated sensors? (Specify data types, number of samples, sampling rate.)
3. **Metrics**: Define 5 quantitative metrics to compare real vs. simulated data. Examples:
   - Mean absolute error (MAE) in depth measurements (m)
   - Pixel-wise standard deviation (temporal noise, m)
   - Invalid pixel rate (% of pixels with no depth)
   - Edge artifact density (flying pixels per meter of edge)
   - Depth-dependent noise coefficient (fit σ(d) = σ₀ + k × d²)
4. **Analysis**: For each metric, define "acceptable fidelity" thresholds (e.g., "MAE < 3 cm is acceptable for manipulation tasks").
5. **Improvement recommendations**: If simulation fidelity is insufficient, what 3 parameters would you tune in Gazebo to improve it?

**Deliverable**: Experimental protocol document (700-900 words) with:
- Experimental setup description (with diagram or photo reference)
- Data collection procedure (step-by-step)
- Metrics definitions (with equations where applicable)
- Fidelity thresholds (justified by use case)
- Improvement recommendations (specific Gazebo parameters)

**Assessment criteria**:
- Experimental setup is reproducible and realistic
- Data collection procedure is detailed and systematic
- Metrics are quantitative and relevant to depth camera performance
- Fidelity thresholds are justified (not arbitrary)
- Improvement recommendations are actionable (specific Gazebo parameters, not vague)

---

### Exercise 15: Design a Synthetic Data Pipeline for Manipulation (Chapters 4, 5)

**Difficulty**: Challenging
**Time estimate**: 3 hours

**Prompt**:
You are training a vision-language-action (VLA) policy for humanoid manipulation (tasks like "Pick up the red cup", "Open the drawer"). The policy requires 100,000 labeled demonstrations. Design a **synthetic data generation pipeline** using Unity.

**Answer the following**:

1. **Scene design**: Describe 3-5 environment templates (kitchen, office, workshop) and 10-20 object categories (cups, tools, furniture). How would you model them in Unity?
2. **Domain randomization**: List 10 randomization parameters (e.g., object position, lighting, textures) and their ranges (e.g., "Object X: uniform(-0.5, 0.5)"). Justify why each is important for generalization.
3. **Labeling strategy**: What labels do you need for each frame? (e.g., object bounding boxes, segmentation masks, grasp poses, language annotations). How would you generate them automatically in Unity?
4. **Data volume and sampling**: How many frames per environment? How many variations per object? What sampling strategy ensures diversity (avoid over-representing easy scenarios)?
5. **Validation**: How would you validate that the synthetic dataset improves real-world policy performance? Propose an evaluation protocol (sim + real).

**Deliverable**: Pipeline design document (1000-1200 words) with:
- Scene design section (with example object list)
- Domain randomization table (10 parameters, ranges, justifications)
- Labeling strategy section (data formats, automation tools)
- Data volume and sampling strategy (with calculations: "100,000 frames = X environments × Y variations × Z frames per variation")
- Validation protocol (sim evaluation + real-world test plan)

**Assessment criteria**:
- Scene design is diverse and realistic (covers multiple environments and object types)
- Domain randomization parameters are justified (not arbitrary) and cover appearance, geometry, and physics
- Labeling strategy is automated (not manual annotation) and uses Unity tools (Perception package)
- Data volume calculations are mathematically sound and ensure coverage
- Validation protocol includes both sim and real-world tests, with clear metrics

---

## Summary

These 15 exercises progress from conceptual understanding (Recall) to hands-on implementation (Application) to system design (Synthesis). Completing all exercises will ensure mastery of:

- **Simulation concepts**: Value proposition, physics vs. kinematics, digital twins
- **Gazebo skills**: World design, sensor configuration, launch files, RViz visualization
- **Unity skills**: Scene setup, ROS integration, synthetic data generation
- **Sensor simulation**: LiDAR, depth cameras, IMUs, RGB cameras (noise models, validation)
- **Critical thinking**: Sim-to-real analysis, architecture design, fidelity evaluation

Recommended progression:
1. **Week 1**: Recall exercises (1-5) to solidify concepts.
2. **Week 2**: Application exercises (6-10) to build hands-on skills.
3. **Week 3**: Synthesis exercises (11-15) to develop system design expertise.

---

**Navigation**
← [Examples](/digital-twin/examples)
→ [Module 3: Vision-Language-Action Models](/vla) (coming soon)
