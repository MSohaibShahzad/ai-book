---
title: "Chapter 3: Gazebo Physics Simulation"
slug: /digital-twin/gazebo-physics-simulation
sidebar_label: "3. Gazebo Physics Simulation"
sidebar_position: 4
toc: true
description: "Master Gazebo physics simulation: architecture, SDF world files, robot model integration, sensor plugins, ROS 2 integration, and contact dynamics."
---

# Chapter 3: Gazebo Physics Simulation

## Introduction

**Gazebo** is the industry-standard physics simulator for robotics research and development. Originally developed by the USC Robotics Research Lab in 2002 and later maintained by Open Robotics (now Open Source Robotics Foundation), Gazebo has been used to simulate robots for the DARPA Robotics Challenge, NASA's Mars rovers, and countless academic research projects. For humanoid robotics, Gazebo provides:

- **Accurate physics**: Rigid-body dynamics, contact/collision detection, joint constraints
- **Sensor simulation**: Cameras, LiDAR, depth sensors, IMUs, force-torque sensors
- **ROS integration**: Native support for ROS 1 and ROS 2 (topics, services, parameters)
- **Extensibility**: Plugin system for custom sensors, actuators, and world dynamics
- **Visualization**: Real-time 3D rendering with adjustable quality

This chapter provides a comprehensive guide to Gazebo, covering its **architecture**, **world design** (SDF format), **robot model integration**, **sensor plugins**, **ROS 2 launch files**, and **contact dynamics**. By the end, you will be able to create custom simulation environments for humanoid robots and launch them from ROS 2.

## Gazebo Architecture

### Core Components

Gazebo consists of several interacting processes:

1. **gzserver** (physics engine):
   - Runs the physics simulation (dynamics, collisions, sensors)
   - Headless (no GUI)—can run on remote servers or in Docker
   - Publishes simulation state to Gazebo topics (not ROS topics)

2. **gzclient** (GUI):
   - Provides the 3D visualization window
   - Allows manual object manipulation, camera control
   - Optional (you can run gzserver alone for faster simulations)

3. **Plugins** (shared libraries):
   - **World plugins**: Modify global simulation behavior (e.g., wind, custom forces)
   - **Model plugins**: Add behavior to specific robots or objects (e.g., controller, sensor)
   - **Sensor plugins**: Generate sensor data (e.g., camera images, LiDAR scans)
   - **System plugins**: Extend Gazebo's core functionality

4. **ROS-Gazebo bridge** (`ros_gz_bridge`):
   - Translates Gazebo topics ↔ ROS 2 topics
   - Example: Gazebo `/camera/image_raw` → ROS `/robot/camera/image_raw`

### Versions: Classic vs. Ignition/Fortress

As of 2023, there are two main Gazebo lineages:

| **Aspect** | **Gazebo Classic (11.x)** | **Gazebo Fortress (Ignition)** |
|------------|---------------------------|--------------------------------|
| **Release** | Final version: 11.14 (2021) | Active development (2021+) |
| **ROS 1** | ✅ Native support | ⚠️ Limited (via ros_gz_bridge) |
| **ROS 2** | ⚠️ Via gazebo_ros_pkgs | ✅ Native support |
| **Physics engines** | ODE, Bullet, Simbody, DART | Same + improved performance |
| **Graphics** | OGRE 1.x | OGRE 2.x (better rendering) |
| **Performance** | Good | Excellent (multithreaded) |
| **Status** | Maintenance mode | Active development |

**Recommendation for humanoid robotics**:
- Use **Gazebo Classic 11.x** if your ecosystem relies on ROS 1 or legacy packages.
- Use **Gazebo Fortress (or later)** for new ROS 2 projects, especially if you need high-fidelity rendering or large-scale simulations (100+ objects).

This chapter focuses on **Gazebo Classic 11.x** (most widely deployed), but concepts transfer directly to Fortress.

### Physics Engines

Gazebo supports multiple physics backends:

1. **ODE (Open Dynamics Engine)**:
   - Default in Gazebo Classic
   - Fast, stable for most robotics tasks
   - Limited accuracy for complex contacts (stacking, soft bodies)

2. **Bullet**:
   - Faster than ODE for many-body simulations (100+ objects)
   - Better handling of convex shapes
   - Used in gaming (Blender, Unreal Engine)

3. **Simbody**:
   - Highest accuracy for biomechanics and precise kinematics
   - Slower than ODE/Bullet
   - Best for humanoid research requiring accurate joint dynamics

4. **DART (Dynamic Animation and Robotics Toolkit)**:
   - Excellent for contact-rich manipulation
   - Supports soft bodies, cable simulation
   - Used in MuJoCo-like tasks

**How to choose**:
- **Default (ODE)**: General-purpose, good balance of speed and accuracy.
- **Bullet**: If you need >50 simulated objects or fast collision detection.
- **Simbody**: If joint accuracy is critical (e.g., biomechanical studies).
- **DART**: If you're simulating complex manipulation (cables, soft objects).

You can switch physics engines by modifying the world SDF file (see below).

## World Files: SDF Format

A **world file** (`.world`) defines the simulation environment: terrain, objects, lighting, gravity, and global physics parameters. Gazebo uses **SDF (Simulation Description Format)**, an XML-based format.

### Basic World Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include a robot model -->
    <include>
      <uri>model://my_humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Key World Parameters

#### 1. Physics Configuration

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>        <!-- 1 ms timestep -->
  <real_time_factor>1.0</real_time_factor>   <!-- 1.0 = real-time, 2.0 = 2× speed -->
  <real_time_update_rate>1000</real_time_update_rate> <!-- 1000 Hz update rate -->
  <gravity>0 0 -9.81</gravity>               <!-- Earth gravity -->
</physics>
```

**Timestep selection**:
- **Smaller timestep** (0.0001–0.001 s): More accurate, but slower simulation.
- **Larger timestep** (0.01 s): Faster, but may cause instability (robot jitters, falls through floor).
- **Rule of thumb**: Use 0.001 s (1 ms) for humanoid locomotion; decrease to 0.0005 s if you observe instability.

**Real-time factor**:
- 1.0 = simulation runs at real-time speed (1 second of simulation = 1 second of wall-clock time)
- &lt;1.0 = slower than real-time (complex scenes)
- \>1.0 = faster than real-time (if your CPU is fast enough)

#### 2. Gravity and Atmospheric Drag

```xml
<gravity>0 0 -9.81</gravity>  <!-- Standard Earth gravity -->
<!-- Or simulate Moon gravity: -->
<gravity>0 0 -1.62</gravity>

<!-- Atmospheric drag (optional, for high-speed motion) -->
<atmosphere type="adiabatic">
  <temperature>293.15</temperature>  <!-- 20°C -->
  <pressure>101325</pressure>        <!-- Sea level -->
</atmosphere>
```

#### 3. Contact Surface Properties

```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>      <!-- Coefficient of friction (0 = ice, 1 = rubber) -->
      <mu2>0.8</mu2>    <!-- Secondary friction direction -->
      <slip1>0.0</slip1>
      <slip2>0.0</slip2>
    </ode>
  </friction>
  <bounce>
    <restitution_coefficient>0.0</restitution_coefficient> <!-- 0 = no bounce, 1 = perfect bounce -->
    <threshold>1e6</threshold>
  </bounce>
  <contact>
    <ode>
      <kp>1e7</kp>     <!-- Contact stiffness (higher = harder surface) -->
      <kd>1.0</kd>     <!-- Contact damping -->
    </ode>
  </contact>
</surface>
```

**Example use cases**:
- **Slippery floor**: Set `mu = 0.1` (robot slips during walking)
- **Soft ground**: Lower `kp` to `1e5` (feet "sink" slightly)
- **Bouncy ball**: Set `restitution_coefficient = 0.9`

### Adding Objects to the World

You can include predefined models or define custom objects inline:

#### Include a Predefined Model

```xml
<include>
  <uri>model://cafe_table</uri>  <!-- From Gazebo model database -->
  <pose>2 0 0 0 0 0</pose>       <!-- x y z roll pitch yaw -->
</include>
```

Gazebo searches for models in:
1. `GAZEBO_MODEL_PATH` environment variable
2. `~/.gazebo/models/`
3. Online model database (auto-downloads)

#### Define a Custom Object Inline

```xml
<model name="box_obstacle">
  <static>false</static>  <!-- false = can be pushed, true = immovable -->
  <pose>1 0 0.5 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.083</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.083</iyy>
        <iyz>0.0</iyz>
        <izz>0.083</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>  <!-- Red -->
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Robot Model Integration: URDF and SDF

Gazebo can load robot models in two formats:

1. **URDF (Unified Robot Description Format)**: ROS-native format (XML), supports xacro macros.
2. **SDF (Simulation Description Format)**: Gazebo-native format, more expressive (supports model composition, sensors).

### URDF → SDF Conversion

Gazebo internally converts URDF to SDF. Most ROS users write URDF (with xacro), and Gazebo handles the conversion automatically.

**Limitations of URDF**:
- Cannot define multiple robots in one file
- No native sensor plugins (must use Gazebo extensions)
- Limited to tree-structured kinematic chains (no closed loops)

**Advantages of SDF**:
- Can define entire worlds (multiple robots, objects, lighting)
- Native sensor/actuator plugins
- Supports parallel mechanisms (closed-loop kinematics)

**Best practice**: Write your robot in **URDF (xacro)** for ROS compatibility, then add Gazebo-specific plugins via `<gazebo>` tags.

### Gazebo Extensions in URDF

To make a URDF robot work in Gazebo, add `<gazebo>` tags:

#### Example: Friction on a Link

```xml
<gazebo reference="left_foot">
  <mu1>1.0</mu1>  <!-- Friction coefficient direction 1 -->
  <mu2>1.0</mu2>  <!-- Friction coefficient direction 2 -->
  <kp>1e7</kp>    <!-- Contact stiffness -->
  <kd>1.0</kd>    <!-- Contact damping -->
  <material>Gazebo/Grey</material>
</gazebo>
```

#### Example: Joint Damping and Limits

```xml
<gazebo reference="left_knee">
  <implicitSpringDamper>true</implicitSpringDamper>
  <springStiffness>0.0</springStiffness>
  <springReference>0.0</springReference>
  <dampingFactor>0.1</dampingFactor>
</gazebo>
```

#### Example: Camera Sensor Plugin

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**What this does**:
- Creates a simulated camera on `camera_link`
- Publishes images to ROS 2 topic `/robot/camera/image_raw` at 30 Hz
- Adds Gaussian noise (σ = 0.007, about 2 intensity units on a 0–255 scale)

## Sensor Plugins for Humanoid Robots

Gazebo provides sensor plugins for common robotics sensors. Here are key plugins for humanoid robots:

### 1. RGB Camera (`libgazebo_ros_camera.so`)

Publishes RGB images and camera info (intrinsics, distortion).

**ROS 2 topics published**:
- `image_raw` (sensor_msgs/Image): RGB image
- `camera_info` (sensor_msgs/CameraInfo): Intrinsics, distortion

### 2. Depth Camera (`libgazebo_ros_camera.so` with depth)

Publishes RGB + depth map (RGB-D sensor like Intel RealSense).

**ROS 2 topics published**:
- `image_raw`: RGB image
- `depth/image_raw`: Depth map (32-bit float, in meters)
- `points`: Point cloud (sensor_msgs/PointCloud2)

**Configuration example**:

```xml
<sensor type="depth" name="depth_camera">
  <update_rate>10.0</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.5</near>   <!-- Min range: 0.5 m -->
      <far>10.0</far>    <!-- Max range: 10 m -->
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>depth/image_raw:=depth_camera/depth/image_raw</remapping>
      <remapping>depth/points:=depth_camera/points</remapping>
    </ros>
  </plugin>
</sensor>
```

### 3. LiDAR / Laser Scanner (`libgazebo_ros_ray_sensor.so`)

Publishes laser scans (2D) or 3D point clouds.

**ROS 2 topics published**:
- `scan` (sensor_msgs/LaserScan): 2D laser scan
- OR `points` (sensor_msgs/PointCloud2): 3D point cloud

**Configuration example (2D LiDAR)**:

```xml
<sensor type="ray" name="laser">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>laser_link</frame_name>
  </plugin>
</sensor>
```

### 4. IMU (`libgazebo_ros_imu_sensor.so`)

Publishes accelerometer, gyroscope, and (optionally) magnetometer data.

**ROS 2 topics published**:
- `imu` (sensor_msgs/Imu): Orientation, angular velocity, linear acceleration

**Configuration example**:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

**Noise values** (typical for consumer-grade IMUs):
- Gyroscope: σ = 0.0002 rad/s
- Accelerometer: σ = 0.017 m/s²

### 5. Force-Torque Sensor (`libgazebo_ros_ft_sensor.so`)

Measures forces and torques at a joint (e.g., foot-ground contact).

**ROS 2 topics published**:
- `wrench` (geometry_msgs/WrenchStamped): Force (Fx, Fy, Fz) and torque (Tx, Ty, Tz)

**Configuration example**:

```xml
<gazebo reference="left_ankle_joint">
  <sensor name="left_foot_force_torque" type="force_torque">
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>  <!-- Measure forces in child link frame -->
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=left_foot/wrench</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Launching Gazebo from ROS 2

To integrate Gazebo with ROS 2, use the `ros_gz` (or `gazebo_ros`) packages.

### Installation (ROS 2 Humble + Gazebo 11)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Basic Launch File

Create a ROS 2 launch file (`launch/gazebo_humanoid.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your world file
    world_file = os.path.join(
        get_package_share_directory('my_humanoid_description'),
        'worlds',
        'humanoid_world.world'
    )

    # Path to robot URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Start Gazebo server + client
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    # Publish robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
```

**Run the launch file**:

```bash
ros2 launch my_humanoid_description gazebo_humanoid.launch.py
```

### Gazebo ROS Control Integration

To control the robot's joints, use `ros2_control` with Gazebo:

1. Add `ros2_control` hardware interface to your URDF:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_knee">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Repeat for all actuated joints -->
</ros2_control>
```

2. Add Gazebo plugin to load ros2_control:

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_humanoid_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

3. Define controllers in `config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

position_controller:
  ros__parameters:
    joints:
      - left_hip_yaw
      - left_hip_roll
      - left_hip_pitch
      - left_knee
      - left_ankle_pitch
      - left_ankle_roll
      # ... (all actuated joints)
```

4. Launch controllers:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active position_controller
```

## Contact Forces and Collision Detection

Accurate contact simulation is critical for humanoid locomotion (foot-ground contact) and manipulation (grasping).

### Contact Models in Gazebo

Gazebo supports two contact models:

1. **Penalty-based** (soft contacts):
   - Objects can slightly interpenetrate.
   - Contact force F = kp × penetration_depth - kd × penetration_velocity
   - Controlled by `<kp>` (stiffness) and `<kd>` (damping).
   - **Pros**: Stable, fast.
   - **Cons**: Objects "sink" slightly into each other.

2. **Constraint-based** (hard contacts):
   - No interpenetration allowed (solved via constraints).
   - More accurate but can be unstable if timestep is too large.
   - Rarely used in Gazebo (ODE default is penalty-based).

### Tuning Contact Parameters

For humanoid foot-ground contact, typical values:

```xml
<surface>
  <contact>
    <ode>
      <kp>1e7</kp>      <!-- Stiffness: 10^7 N/m (concrete-like) -->
      <kd>1.0</kd>      <!-- Damping: 1 Ns/m -->
      <max_vel>0.01</max_vel>
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
  <friction>
    <ode>
      <mu>1.0</mu>     <!-- Rubber-like friction -->
      <mu2>1.0</mu2>
    </ode>
  </friction>
</surface>
```

**If the robot sinks into the floor**: Increase `kp` (e.g., `1e8`).
**If the robot bounces excessively**: Increase `kd` (e.g., `10.0`).
**If the robot slips during walking**: Increase `mu` (e.g., `1.5`).

### Visualizing Contact Forces

Enable contact visualization in Gazebo:

1. Open Gazebo GUI.
2. Go to **View → Contacts**.
3. Contacts are shown as pink/purple force vectors.

Or, subscribe to the `/gazebo/contact_states` topic (if enabled in world file):

```xml
<plugin name="gazebo_ros_bumper_controller" filename="libgazebo_ros_bumper.so">
  <bumperTopicName>contact_states</bumperTopicName>
</plugin>
```

## Summary

Gazebo is a powerful, industry-standard physics simulator for humanoid robotics. Key takeaways:

1. **Architecture**: `gzserver` (physics) + `gzclient` (GUI) + plugins (sensors, controllers).
2. **Physics engines**: ODE (default, balanced), Bullet (fast), Simbody (accurate), DART (manipulation).
3. **World files (SDF)**: Define environments, objects, lighting, gravity, and physics parameters.
4. **Robot integration**: Use URDF (xacro) with `<gazebo>` tags for sensors and material properties.
5. **Sensor plugins**: Cameras, depth sensors, LiDAR, IMU, force-torque sensors—configured via XML.
6. **ROS 2 integration**: Launch Gazebo from ROS 2, spawn robots, publish sensor data to topics.
7. **Contact dynamics**: Tune `kp`, `kd`, `mu` for realistic foot-ground and grasp contacts.

In the next chapter, you will explore **Unity** as a complementary tool for high-fidelity rendering and Human-Robot Interaction visualization.

## Self-Check Questions

1. **Explain the difference between `gzserver` and `gzclient`.** Why might you run `gzserver` without `gzclient`?

2. **Choose a physics engine**: You're simulating a humanoid robot stacking 50 boxes. Which physics engine (ODE, Bullet, Simbody, DART) would you choose, and why?

3. **Design a world file**: Write the SDF snippet for a 10 m × 10 m flat ground plane with friction coefficient 0.6 and a 1 m × 1 m × 1 m box placed at (2, 3, 0.5).

4. **Configure a depth camera**: You want a depth camera with 90° field of view, 320×240 resolution, 0.3–5 m range, and 20 Hz update rate. Write the Gazebo sensor plugin XML.

5. **Debug contact issues**: A simulated humanoid robot sinks 2 cm into the floor during standing. What parameter would you adjust, and to what value?

6. **Compare URDF and SDF**: What are two advantages of SDF over URDF? When would you prefer to write an SDF file instead of URDF?

## Next Steps

Proceed to **Chapter 4: Unity High-Fidelity Rendering** to learn how Unity complements Gazebo for photorealistic visualization and Human-Robot Interaction research.

---

**Chapter Navigation**
← [Chapter 2: Digital Twins Concept](/digital-twin/digital-twins-concept)
→ [Chapter 4: Unity High-Fidelity Rendering](/digital-twin/unity-high-fidelity-rendering)
