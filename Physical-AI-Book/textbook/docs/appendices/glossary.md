---
title: "Glossary"
slug: glossary
sidebar_label: "Glossary"
toc: true
description: "Definitions of key terms and concepts used throughout the Physical-AI textbook"
---

# Glossary

This glossary provides definitions for key terms used throughout the textbook. Terms are organized alphabetically with cross-references where applicable.

---

## A

**Action (ROS 2)**
A communication pattern for long-running tasks that provides feedback during execution and allows cancellation. Consists of a goal, feedback, and result. See also: *Topic*, *Service*.

**Agent**
In AI contexts, an autonomous entity that perceives its environment through sensors and acts upon it through actuators. In this textbook, typically refers to LLM-based cognitive planners.

**AMCL (Adaptive Monte Carlo Localization)**
A probabilistic localization algorithm that uses particle filters to track a robot's pose within a known map.

**Articulated Robot**
A robot with rotational joints connecting rigid links, allowing multi-degree-of-freedom movement. Humanoids are articulated robots.

**Autonomy**
The capability of a system to perform tasks without continuous human intervention. Levels range from teleoperation (no autonomy) to full autonomy.

---

## B

**Baseline**
A reference implementation or benchmark used for comparison when evaluating improvements.

**Bloom's Taxonomy**
An educational framework classifying learning objectives by cognitive complexity: Remember, Understand, Apply, Analyze, Evaluate, Create. This textbook uses Recall, Application, and Synthesis tiers.

**Bounding Box**
A rectangular region enclosing an object in an image, typically used in object detection tasks.

---

## C

**Cartesian Space**
The 3D coordinate system (x, y, z) representing positions in physical space, as opposed to joint space.

**Cloud Robotics**
Offloading computation, storage, or learning to cloud servers rather than performing all processing onboard the robot.

**Collision Geometry**
Simplified shapes (boxes, cylinders, meshes) used for physics-based collision detection, distinct from visual geometry.

**Configuration Space (C-Space)**
The space of all possible joint angles for a robot. Path planning often operates in C-space.

**CoT (Chain-of-Thought)**
A prompting technique where LLMs are asked to show intermediate reasoning steps, improving complex task performance.

**CUDA**
NVIDIA's parallel computing platform enabling GPU-accelerated computation, critical for simulation and AI inference.

---

## D

**Degrees of Freedom (DoF)**
The number of independent parameters needed to define a robot's configuration. A humanoid arm typically has 7 DoF.

**Digital Twin**
A high-fidelity virtual replica of a physical system, used for simulation, testing, and prediction.

**DDS (Data Distribution Service)**
A middleware standard for real-time publish-subscribe communication. ROS 2 uses DDS as its underlying transport layer.

---

## E

**Edge Computing**
Performing computation on or near the robot (the "edge") rather than in the cloud, reducing latency.

**Embodied AI**
AI systems that interact with the physical world through sensors and actuators, as opposed to purely digital AI.

**End Effector**
The device at the end of a robotic arm (gripper, tool, hand) that interacts with the environment.

**E-Stop (Emergency Stop)**
A safety mechanism that immediately halts all robot motion, typically a physical button.

**Extrinsic Calibration**
Determining the relative pose between multiple sensors (e.g., camera-to-LiDAR transform).

---

## F

**Forward Kinematics**
Computing end-effector pose from joint angles. See also: *Inverse Kinematics*.

**Frontier Exploration**
A navigation strategy where the robot moves toward the boundary between known and unknown map regions.

**Fusion (Sensor Fusion)**
Combining data from multiple sensors (camera, LiDAR, IMU) to produce more accurate estimates than any single sensor.

---

## G

**Gazebo**
An open-source 3D robotics simulator supporting physics, sensors, and ROS integration. Used in Module 2.

**Grasp Planning**
Determining how to position a gripper to securely hold an object, often using force-closure analysis.

**Ground Truth**
The actual, verified correct values for a dataset, used to evaluate model performance.

---

## H

**Hallucination (LLM)**
When a language model generates plausible-sounding but factually incorrect or nonsensical outputs.

**Homogeneous Transformation**
A 4x4 matrix representing both rotation and translation, used to transform coordinates between frames.

**Humanoid Robot**
A robot with human-like morphology, typically including a torso, head, two arms, and two legs.

---

## I

**IMU (Inertial Measurement Unit)**
A sensor combining accelerometers and gyroscopes to measure linear acceleration and angular velocity.

**Inference**
Running a trained AI model to make predictions on new data (as opposed to training).

**Inverse Kinematics (IK)**
Computing joint angles required to reach a desired end-effector pose. See also: *Forward Kinematics*.

**Isaac Sim**
NVIDIA's GPU-accelerated robotics simulator with photorealistic rendering and AI-ready features. Used in Module 3.

---

## J

**Joint Space**
The coordinate system defined by robot joint angles (θ₁, θ₂, ..., θₙ), as opposed to Cartesian space.

**JPEG**
A lossy image compression format. Common for transmitting camera data in ROS topics.

---

## K

**Kalman Filter**
A recursive algorithm for estimating the state of a dynamic system from noisy measurements.

**Kinematic Chain**
A series of rigid links connected by joints, forming the structure of an articulated robot.

---

## L

**Latency**
The time delay between an event (sensor reading) and the system's response (action).

**LiDAR (Light Detection and Ranging)**
A sensor that measures distances using laser pulses, producing point clouds for mapping and obstacle detection.

**LLM (Large Language Model)**
Neural networks trained on vast text corpora (e.g., GPT, PaLM) capable of natural language understanding and generation.

**Localization**
Determining a robot's position and orientation within a map or environment.

---

## M

**Manipulation**
The task of grasping, moving, and placing objects using robotic arms/hands.

**Map (SLAM)**
A representation of the environment, typically as an occupancy grid (2D) or point cloud (3D).

**Message (ROS 2)**
A data structure passed between nodes via topics, services, or actions. Defined in `.msg` files.

**Motion Planning**
Computing a collision-free path from a start to a goal configuration, considering kinematic/dynamic constraints.

---

## N

**Nav2 (Navigation 2)**
The ROS 2 navigation stack, providing path planning, obstacle avoidance, and goal-reaching behaviors.

**Node (ROS 2)**
An independent process that performs computation, communicates via topics/services/actions.

---

## O

**Occupancy Grid**
A 2D map representation where each cell is labeled as free, occupied, or unknown.

**Odometry**
Estimating a robot's position by integrating wheel encoder or IMU data over time (subject to drift).

**Omniverse**
NVIDIA's platform for 3D simulation and collaboration, which Isaac Sim is built on.

---

## P

**Parameter Server (ROS 2)**
A distributed key-value store for runtime configuration (e.g., PID gains, topic names).

**Particle Filter**
A probabilistic algorithm representing beliefs as a set of weighted samples, used in AMCL localization.

**Perception**
The process of extracting meaningful information from sensor data (object detection, segmentation, etc.).

**Physical-AI**
The integration of AI (perception, planning, learning) with physical embodiment (robots), the focus of this textbook.

**Point Cloud**
A set of 3D points (x, y, z) representing surfaces, typically from LiDAR or depth cameras.

**Pose**
Position (x, y, z) and orientation (roll, pitch, yaw or quaternion) of an object or robot in space.

---

## Q

**Quaternion**
A 4-element representation of rotation (w, x, y, z) that avoids gimbal lock, preferred over Euler angles in robotics.

**QoS (Quality of Service)**
ROS 2 settings controlling message delivery reliability, history depth, and durability.

---

## R

**ROS 2 (Robot Operating System 2)**
An open-source middleware framework for robot software development, featuring nodes, topics, services, and actions.

**RTAB-Map (Real-Time Appearance-Based Mapping)**
A SLAM algorithm using visual features for loop closure detection and 3D mapping.

---

## S

**SLAM (Simultaneous Localization and Mapping)**
The problem of building a map while simultaneously tracking the robot's location within it.

**Service (ROS 2)**
A synchronous request-response communication pattern. See also: *Topic*, *Action*.

**Sim-to-Real**
Transferring policies or models trained in simulation to real-world robots, addressing the reality gap.

**Synthetic Data**
Artificially generated training data (images, point clouds) from simulation, avoiding privacy concerns and enabling scalability.

---

## T

**tf2 (Transform Library)**
ROS 2's system for managing coordinate frame transformations over time, critical for sensor fusion.

**Topic (ROS 2)**
An asynchronous publish-subscribe communication channel for streaming data (e.g., sensor readings).

**Trajectory**
A time-parameterized path specifying position, velocity, and acceleration at each timestep.

---

## U

**Unity**
A game engine used for high-fidelity robot simulation and rendering in Module 2.

**URDF (Unified Robot Description Format)**
An XML format describing a robot's kinematic structure, visual appearance, and collision geometry.

---

## V

**VLA (Vision-Language-Action)**
Models that map visual observations and language instructions directly to robot actions, covered in Module 4.

**Visual Odometry**
Estimating motion by tracking visual features across camera frames.

**VRAM (Video RAM)**
Memory on the GPU, critical for rendering and AI inference. Isaac Sim requires 12+ GB.

**VSLAM (Visual SLAM)**
SLAM using camera-based features rather than LiDAR.

---

## W

**Whisper**
OpenAI's automatic speech recognition model, used in Module 4 for voice-to-text conversion.

**World Frame**
A fixed global coordinate system relative to which all other frames are defined.

---

## X

**XACRO (XML Macros)**
A macro language for generating URDF files programmatically, reducing repetition.

---

## Y

**YAML (YAML Ain't Markup Language)**
A human-readable data serialization format used for ROS configuration files and frontmatter in this textbook.

**Yaw, Pitch, Roll**
Euler angles representing rotation around the z, y, and x axes respectively.

---

## Z

**Zero-Shot Learning**
A model's ability to perform tasks it wasn't explicitly trained on, leveraging pre-trained knowledge (common in LLMs).

---

## Cross-References

For in-depth explanations, refer to:
- **ROS 2 concepts**: Module 1
- **Simulation tools**: Module 2
- **Isaac-specific terms**: Module 3
- **AI/ML terms**: Module 4
- **Further reading**: [Appendix - Further Reading](./further-reading.md)

---

**Didn't find a term?** Search the textbook or suggest additions via the GitHub issues page.
