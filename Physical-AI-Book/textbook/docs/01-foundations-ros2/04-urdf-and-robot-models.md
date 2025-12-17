---
title: "Chapter 4: URDF and Robot Models"
slug: urdf-and-robot-models
sidebar_label: "4. URDF & Robot Models"
toc: true
description: "Modeling humanoid robots with URDF: links, joints, transforms, and visualization in RViz for physical AI systems"
---

# Chapter 4: URDF and Robot Models

## Introduction

You've learned how to communicate between nodes (topics, services, actions). But how does ROS 2 know what your robot *looks like*? Where are its cameras? How do its arms move? What's the relationship between the base and the gripper?

**The answer**: **URDF (Unified Robot Description Format)**—an XML-based language for describing robot geometry, kinematics, and dynamics.

**The driving question**: How do you model a humanoid robot with 40+ joints, multiple sensors, and complex kinematics in a way that ROS 2 can use for motion planning, collision detection, and visualization?

**Learning Objectives**:

By the end of this chapter, you will:

1. **Define** URDF structure: links, joints, collision/visual geometry
2. **Write** a URDF model for a simplified humanoid arm
3. **Use** XACRO to reduce repetition and improve maintainability
4. **Visualize** robot models in RViz with real-time joint updates
5. **Explain** coordinate frames and tf2 transforms
6. **Configure** inertial properties for physics simulation

**Time**: 4 hours (reading + hands-on)

---

## 4.1 URDF: The Robot Description Language

### 4.1.1 What is URDF?

**URDF** (Unified Robot Description Format) is an XML specification for robot models. It defines:

- **Links**: Rigid bodies (e.g., arm segment, camera mount)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Visual geometry**: What the robot looks like (meshes, primitives)
- **Collision geometry**: Simplified shapes for collision detection
- **Inertial properties**: Mass, center of mass, inertia tensor (for simulation)

**Analogy**: URDF is like a CAD file for robotics—it describes the mechanical structure, but also includes kinematic constraints (joint limits, axes) and physical properties (mass, friction).

### 4.1.2 Why URDF?

**Without URDF**:
- Hard-code joint positions in your code
- No visualization (how do you debug a 40-joint humanoid?)
- No collision checking (crash into walls/self)
- Can't reuse models across projects

**With URDF**:
- Single source of truth for robot geometry
- RViz visualization (see robot state in real-time)
- MoveIt 2 uses URDF for motion planning
- Gazebo uses URDF for physics simulation
- tf2 uses URDF to build transform tree

---

## 4.2 URDF Structure: Links and Joints

### 4.2.1 Links: Rigid Bodies

A **link** is a rigid body. For a humanoid:
- `base_link`: Torso
- `left_shoulder_link`: Left shoulder segment
- `left_elbow_link`: Left forearm
- `left_wrist_link`: Left hand

**Basic link structure**:

```xml
<link name="left_shoulder_link">
  <!-- Visual: what you see in RViz -->
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision: simplified geometry for collision detection -->
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>

  <!-- Inertial: mass properties (for simulation) -->
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

**Key sections**:

1. **Visual**: Displayed in RViz/Gazebo (can be detailed mesh)
2. **Collision**: Used for collision checking (usually simpler than visual)
3. **Inertial**: Required for physics simulation (Gazebo, Isaac Sim)

**Why separate collision and visual?**
- Visual: High-poly mesh for aesthetics (10k triangles)
- Collision: Low-poly primitive for fast collision checks (10 triangles)

### 4.2.2 Joints: Connections Between Links

A **joint** defines how two links move relative to each other.

**Joint types**:

| **Type** | **Description** | **Example** |
|----------|-----------------|-------------|
| **Revolute** | Rotation around axis (with limits) | Elbow (0° to 150°) |
| **Continuous** | Rotation around axis (no limits) | Wheel |
| **Prismatic** | Linear motion along axis | Gripper fingers |
| **Fixed** | No motion (rigid attachment) | Camera mount |
| **Planar** | Motion in a plane | Uncommon (use floating instead) |
| **Floating** | 6-DOF free motion | Underwater robot |

**Example: Revolute joint** (elbow):

```xml
<joint name="left_elbow_joint" type="revolute">
  <!-- Parent link: upper arm -->
  <parent link="left_shoulder_link"/>

  <!-- Child link: forearm -->
  <child link="left_elbow_link"/>

  <!-- Joint origin: where child is attached to parent -->
  <origin xyz="0 0 0.3" rpy="0 0 0"/>

  <!-- Axis of rotation (local to joint frame) -->
  <axis xyz="0 1 0"/>

  <!-- Joint limits (radians) -->
  <limit lower="0" upper="2.618" effort="100" velocity="2.0"/>
</joint>
```

**Key parameters**:

1. **`parent` / `child`**: Which links are connected
2. **`origin`**: Position and orientation of child relative to parent
   - `xyz`: Translation (meters)
   - `rpy`: Rotation (roll, pitch, yaw in radians)
3. **`axis`**: Direction of motion (for revolute/prismatic)
4. **`limit`**: Joint constraints
   - `lower` / `upper`: Range of motion (rad or m)
   - `effort`: Max torque/force (N⋅m or N)
   - `velocity`: Max speed (rad/s or m/s)

---

## 4.3 Building a Humanoid Arm Model

### 4.3.1 Simplified Arm Structure

Let's model a 3-DOF humanoid arm:

```
base_link (torso)
  └─ left_shoulder_joint (revolute)
      └─ left_shoulder_link (upper arm)
          └─ left_elbow_joint (revolute)
              └─ left_elbow_link (forearm)
                  └─ left_wrist_joint (revolute)
                      └─ left_wrist_link (hand)
```

### 4.3.2 Complete URDF Example

**File**: `humanoid_arm.urdf`

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_link"/>
    <origin xyz="0 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Shoulder link (upper arm) -->
  <link name="left_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder_link"/>
    <child link="left_elbow_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.618" effort="30" velocity="2.0"/>
  </joint>

  <!-- Elbow link (forearm) -->
  <link name="left_elbow_link">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Wrist joint -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_elbow_link"/>
    <child link="left_wrist_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>

  <!-- Wrist link (hand) -->
  <link name="left_wrist_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

---

## 4.4 XACRO: Don't Repeat Yourself

### 4.4.1 The Problem with Raw URDF

**Issue**: Humanoids have 2 arms (mirrored geometry). Copy-pasting URDF is:
- Error-prone (forget to update one side)
- Unmaintainable (change shoulder length → update 2 places)

### 4.4.2 XACRO: Macros for URDF

**XACRO** (XML Macros) extends URDF with:
- **Variables**: `${arm_length}`
- **Macros**: Reusable link/joint templates
- **Math**: `${arm_length * 0.5}`
- **Conditionals**: `<xacro:if value="${use_camera}">`

**Example: Macro for humanoid arm**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Parameters -->
  <xacro:property name="shoulder_length" value="0.3"/>
  <xacro:property name="elbow_length" value="0.25"/>
  <xacro:property name="shoulder_radius" value="0.04"/>

  <!-- Arm macro -->
  <xacro:macro name="humanoid_arm" params="side reflect">

    <!-- Shoulder joint -->
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_shoulder_link"/>
      <origin xyz="0 ${reflect * 0.15} 0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
    </joint>

    <!-- Shoulder link -->
    <link name="${side}_shoulder_link">
      <visual>
        <origin xyz="0 0 ${-shoulder_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${shoulder_radius}" length="${shoulder_length}"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-shoulder_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${shoulder_radius}" length="${shoulder_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <origin xyz="0 0 ${-shoulder_length/2}" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Elbow joint -->
    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_shoulder_link"/>
      <child link="${side}_elbow_link"/>
      <origin xyz="0 0 ${-shoulder_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.618" effort="30" velocity="2.0"/>
    </joint>

    <!-- Elbow link -->
    <link name="${side}_elbow_link">
      <visual>
        <origin xyz="0 0 ${-elbow_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="${elbow_length}"/>
        </geometry>
        <material name="green">
          <color rgba="0 1 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-elbow_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="${elbow_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 ${-elbow_length/2}" rpy="0 0 0"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
      </inertial>
    </link>

  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Instantiate left and right arms -->
  <xacro:humanoid_arm side="left" reflect="1"/>
  <xacro:humanoid_arm side="right" reflect="-1"/>

</robot>
```

**Compile XACRO to URDF**:

```bash
ros2 run xacro xacro humanoid.urdf.xacro > humanoid.urdf
```

**Benefits**:
- Change `shoulder_length` in one place → updates both arms
- `reflect` parameter mirrors left/right geometry
- Less code, fewer errors

---

## 4.5 Coordinate Frames and tf2

### 4.5.1 The Transform Tree

Every link in URDF has a **coordinate frame**. ROS 2 uses **tf2** (transform library) to:
- Compute transforms between frames (e.g., "Where is the wrist relative to the base?")
- Publish dynamic transforms (e.g., joint angles change → wrist position changes)

**Example transform tree**:

```
base_link
  └─ left_shoulder_link (via left_shoulder_joint)
      └─ left_elbow_link (via left_elbow_joint)
          └─ left_wrist_link (via left_wrist_joint)
```

**Query transform** (Python):

```python
from tf2_ros import Buffer, TransformListener

buffer = Buffer()
listener = TransformListener(buffer, node)

# Get transform from base_link to left_wrist_link
transform = buffer.lookup_transform('base_link', 'left_wrist_link', rclpy.time.Time())
print(f"Wrist position: {transform.transform.translation}")
```

### 4.5.2 Publishing Robot State

The **`robot_state_publisher`** node:
- Reads URDF from `/robot_description` topic
- Subscribes to `/joint_states` (current joint angles)
- Publishes tf transforms for all links

**Launch robot_state_publisher**:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid_arm.urdf)"
```

**Publish fake joint states** (for testing):

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

## 4.6 Visualizing URDF in RViz

### 4.6.1 RViz: The Robot Visualization Tool

**RViz** is ROS 2's 3D visualization tool. It can display:
- Robot models (URDF)
- Sensor data (camera images, point clouds, laser scans)
- Coordinate frames (tf tree)
- Trajectories, markers, paths

### 4.6.2 Launching RViz with URDF

**Step 1: Start robot_state_publisher**

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro humanoid_arm.urdf.xacro)"
```

**Step 2: Start joint_state_publisher_gui** (to move joints)

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Step 3: Launch RViz**

```bash
rviz2
```

**Step 4: Configure RViz**

1. Set **Fixed Frame** to `base_link`
2. Click **Add** → **RobotModel**
3. In RobotModel settings, set **Description Topic** to `/robot_description`
4. You should see your humanoid arm!
5. Click **Add** → **TF** to see coordinate frames

**Step 5: Move joints**

Use the sliders in `joint_state_publisher_gui` to see the arm move in RViz.

---

## 4.7 Collision vs. Visual Geometry

### 4.7.1 Why Separate Geometry?

**Visual geometry**: High-detail mesh for aesthetics
- Example: 10,000-triangle mesh of a hand

**Collision geometry**: Low-detail primitive for fast collision checks
- Example: Box approximation of hand (8 triangles)

**Trade-off**: Collision checking is O(n²) in triangle count. Using primitives → 100× speedup.

### 4.7.2 Best Practices

| **Use Case** | **Visual** | **Collision** |
|--------------|------------|---------------|
| **Arm segments** | Cylinder mesh | Cylinder primitive |
| **Gripper** | Detailed STL | Box primitive |
| **Camera mount** | Mesh | None (fixed joint) |
| **Wheels** | Mesh | Cylinder |

**Example** (detailed gripper):

```xml
<link name="gripper">
  <!-- Visual: detailed mesh -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/gripper.stl"/>
    </geometry>
  </visual>

  <!-- Collision: simplified box -->
  <collision>
    <geometry>
      <box size="0.1 0.05 0.08"/>
    </geometry>
  </collision>
</link>
```

---

## 4.8 Inertial Properties for Simulation

### 4.8.1 Why Inertia Matters

**Physics simulators** (Gazebo, Isaac Sim) need:
- **Mass**: How heavy is the link?
- **Center of mass**: Where is the weight distributed?
- **Inertia tensor**: How does the link resist rotation?

**Without correct inertia**: Robot behaves unrealistically (e.g., falls through floor, spins uncontrollably).

### 4.8.2 Computing Inertia

**For simple shapes**, use analytical formulas:

**Cylinder** (radius `r`, length `l`, mass `m`):

```
Ixx = Iyy = (1/12) * m * (3*r² + l²)
Izz = (1/2) * m * r²
```

**Box** (width `w`, depth `d`, height `h`, mass `m`):

```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

**For complex meshes**, use CAD software (SolidWorks, Fusion 360) or `meshlab`.

### 4.8.3 Example Inertial Block

```xml
<inertial>
  <mass value="1.5"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>  <!-- Center of mass offset -->
  <inertia ixx="0.01" ixy="0" ixz="0"
           iyy="0.01" iyz="0"
           izz="0.001"/>
</inertial>
```

**Sanity check**: For a 1.5 kg cylinder (radius 0.04 m, length 0.3 m):
- `Izz ≈ 0.5 * 1.5 * 0.04² = 0.0012` ✅ (matches `0.001`)

---

## 4.9 Command-Line Tools for URDF

### 4.9.1 Validating URDF

**Check URDF syntax**:

```bash
check_urdf humanoid_arm.urdf
```

**Output**:
```
robot name is: humanoid_arm
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  left_shoulder_link
        child(1):  left_elbow_link
            child(1):  left_wrist_link
```

### 4.9.2 Visualizing URDF Structure

**Generate graph of link tree**:

```bash
urdf_to_graphiz humanoid_arm.urdf
# Output: humanoid_arm.gv, humanoid_arm.pdf
```

Open `humanoid_arm.pdf` to see the link tree diagram.

---

## 4.10 Real-World Example: Tesla Optimus

**Tesla Optimus** (humanoid robot) has:
- **28 DOF** (degrees of freedom): arms, legs, neck, fingers
- **URDF-like description** (internal format, not public)
- **Inertial properties**: Critical for bipedal walking (center of mass, ZMP)

**Simplified URDF structure**:

```
base_link (pelvis)
  ├─ torso_link
  │   ├─ left_arm (7 DOF)
  │   │   └─ left_hand (6 DOF: 6 fingers)
  │   ├─ right_arm (7 DOF)
  │   └─ head (2 DOF: pan, tilt)
  ├─ left_leg (6 DOF)
  └─ right_leg (6 DOF)
```

**Why URDF matters for Optimus**:
- **Motion planning**: MoveIt 2 uses URDF for IK/FK
- **Collision avoidance**: Prevent self-collision (elbow hitting torso)
- **Simulation**: Test walking gaits in Gazebo before deploying to hardware

---

## 4.11 Common Pitfalls

**Problem 1**: "Failed to parse robot description" in RViz

**Solution**: Validate URDF with `check_urdf`. Common errors:
- Missing parent/child links
- Invalid XML syntax
- Joint axis not normalized (must be unit vector)

**Problem 2**: Robot is invisible in RViz

**Solution**: Check:
- Is `robot_state_publisher` running? (`ros2 node list`)
- Is `/robot_description` published? (`ros2 topic echo /robot_description`)
- Are joint states published? (`ros2 topic echo /joint_states`)
- Is **Fixed Frame** in RViz set to `base_link`?

**Problem 3**: Joint moves in wrong direction

**Solution**: Check `<axis xyz="..."/>` in joint definition. For elbow bending:
- `xyz="0 1 0"` (y-axis) ✅
- `xyz="1 0 0"` (x-axis) ❌

---

## 4.12 Summary

**What you learned**:

✅ **URDF** describes robot structure: links (rigid bodies) and joints (connections)
✅ **XACRO** reduces repetition with macros, variables, and math
✅ **tf2** computes transforms between coordinate frames
✅ **robot_state_publisher** converts joint states + URDF → tf transforms
✅ **RViz** visualizes robot models in 3D
✅ **Collision vs. visual geometry**: Simplified shapes for fast collision detection
✅ **Inertial properties**: Required for physics simulation

**What's next**:

In **Chapter 5**, we'll bridge AI agents (LLMs, planners) to ROS 2 for voice-controlled humanoid robots.

---

## 4.13 Self-Check Questions

Before proceeding, ensure you can answer:

1. **What are the three geometry types in URDF?** (Answer: Visual, Collision, Inertial)
2. **What is the difference between revolute and continuous joints?** (Answer: Revolute has limits, continuous does not)
3. **What does robot_state_publisher do?** (Answer: Converts URDF + joint states → tf transforms)
4. **Why use XACRO instead of raw URDF?** (Answer: Reduce repetition, use variables/macros)
5. **What is tf2 used for?** (Answer: Transform library for coordinate frame conversions)

If you're unsure on any, re-read the relevant section.

---

## 4.14 Exercises

See **[Exercises](./exercises.md)** for hands-on challenges:
- **Recall**: Define URDF, explain joint types
- **Application**: Model a 2-DOF gripper with parallel jaw prismatic joints
- **Synthesis**: Design a full humanoid upper body (2 arms, head, torso) with XACRO macros

---

**Next chapter**: [Chapter 5: Bridging AI Agents to ROS 2 →](./05-bridging-ai-agents-to-ros.md)
