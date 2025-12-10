---
title: "Module 2: Examples"
slug: /digital-twin/examples
sidebar_label: "Examples"
sidebar_position: 7
toc: true
description: "Practical examples for Module 2: Gazebo world files, SDF robot models, sensor configurations, RViz visualization, and Unity scene setup concepts."
---

# Module 2: Digital Twin & Simulation - Examples

This page provides **practical, annotated examples** to complement the conceptual material in Module 2. Each example includes code snippets, configuration files, and step-by-step instructions for hands-on implementation.

---

## Example 1: Complete Gazebo World File for Humanoid Environment

This example demonstrates a complete world file (`.world`) for simulating a humanoid robot in an indoor environment with obstacles, varied terrain, and realistic physics.

### Scenario

Create a 20 m × 20 m indoor environment with:
- Flat ground plane
- Two staircase models (ascending/descending)
- Three box obstacles (representing furniture)
- Realistic lighting (directional sun + ambient)
- Physics timestep optimized for humanoid locomotion

### World File: `humanoid_indoor.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_indoor">

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>       <!-- 1 ms timestep for stability -->
      <real_time_factor>1.0</real_time_factor>   <!-- Target real-time -->
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Lighting: Simulates indoor environment with windows -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <light name="ambient" type="ambient">
      <diffuse>0.4 0.4 0.4 1</diffuse>
    </light>

    <!-- Ground plane: 20m x 20m with realistic friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>      <!-- Rubber-like friction -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1e7</kp>      <!-- Contact stiffness (concrete) -->
                <kd>1.0</kd>      <!-- Contact damping -->
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
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

    <!-- Obstacle 1: Large box (simulates furniture) -->
    <model name="box_obstacle_1">
      <pose>3 2 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <inertial>
          <mass>20.0</mass>
          <inertia>
            <ixx>0.67</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.67</iyy><iyz>0</iyz>
            <izz>0.67</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>1.0 1.0 1.0</size></box>
          </geometry>
          <surface>
            <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.0 1.0 1.0</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 2: Tall cylinder (simulates pillar) -->
    <model name="cylinder_obstacle">
      <pose>-2 -3 1.0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.3</radius><length>2.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.3</radius><length>2.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Stairs: Ascending (5 steps, 0.15 m height each) -->
    <model name="stairs_ascending">
      <pose>5 0 0 0 0 0</pose>
      <static>true</static>
      <link name="step1">
        <pose>0 0 0.075 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
      <link name="step2">
        <pose>0 0.3 0.225 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
      <link name="step3">
        <pose>0 0.6 0.375 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.0 0.3 0.15</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
      <!-- Add step4 and step5 similarly -->
    </model>

    <!-- Humanoid robot spawn location (included from separate model) -->
    <include>
      <uri>model://my_humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Key Annotations

1. **Physics timestep (0.001 s)**: Small enough to prevent foot-ground penetration during locomotion.
2. **Ground friction (μ = 1.0)**: Rubber-like; prevents slipping during walking.
3. **Contact stiffness (kp = 1e7)**: Simulates concrete—robot doesn't sink into floor.
4. **Lighting**: Directional light (sun through window) + ambient (soft fill light).
5. **Obstacles**: Static (pillar) and dynamic (pushable box).
6. **Stairs**: Each step is a separate link (static); robot can climb using foot placement algorithms.

### Usage

1. Save as `humanoid_indoor.world` in your package's `worlds/` directory.
2. Launch Gazebo:

   ```bash
   gazebo --verbose humanoid_indoor.world
   ```

3. Or integrate with ROS 2 launch file (see Example 5 below).

---

## Example 2: SDF Robot Model with Sensor Plugins

This example shows a minimal humanoid robot model (SDF format) with a camera, LiDAR, and IMU.

### Scenario

Create a simplified bipedal robot (2 legs, torso, head) with:
- RGB camera (head-mounted)
- 2D LiDAR (torso-mounted)
- IMU (torso-mounted)

### Robot SDF: `simple_biped.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="simple_biped">

    <!-- Base link (torso) -->
    <link name="torso">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>30.0</mass>
        <inertia>
          <ixx>1.0</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1.0</iyy><iyz>0</iyz><izz>0.5</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.3 0.2 0.5</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.3 0.2 0.5</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <!-- IMU sensor -->
      <sensor name="imu_sensor" type="imu">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros><namespace>/robot</namespace><remapping>~/out:=imu</remapping></ros>
          <frame_name>torso</frame_name>
        </plugin>
      </sensor>

      <!-- 2D LiDAR sensor -->
      <sensor type="ray" name="laser_scanner">
        <pose>0 0 0.3 0 0 0</pose>
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
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros><namespace>/robot</namespace><remapping>~/out:=scan</remapping></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>torso</frame_name>
        </plugin>
      </sensor>
    </link>

    <!-- Head link (with camera) -->
    <link name="head">
      <pose>0 0 1.6 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.15</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.15</radius></sphere></geometry>
        <material><ambient>1 0.8 0.6 1</ambient></material>
      </visual>

      <!-- RGB Camera -->
      <sensor type="camera" name="head_camera">
        <pose>0.1 0 0 0 0 0</pose>
        <update_rate>30.0</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>300</far></clip>
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
          <frame_name>head</frame_name>
        </plugin>
      </sensor>
    </link>

    <!-- Joint: torso to head -->
    <joint name="neck" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>  <!-- -90° -->
          <upper>1.57</upper>   <!-- +90° -->
        </limit>
      </axis>
    </joint>

    <!-- Left leg (simplified: single link + joint) -->
    <link name="left_leg">
      <pose>0 0.1 0.5 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.05</iyy><iyz>0</iyz><izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.05</radius><length>0.8</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.05</radius><length>0.8</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient></material>
      </visual>
    </link>

    <joint name="left_hip" type="revolute">
      <parent>torso</parent>
      <child>left_leg</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>

    <!-- Right leg (mirrored) -->
    <link name="right_leg">
      <pose>0 -0.1 0.5 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.05</iyy><iyz>0</iyz><izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.05</radius><length>0.8</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.05</radius><length>0.8</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient></material>
      </visual>
    </link>

    <joint name="right_hip" type="revolute">
      <parent>torso</parent>
      <child>right_leg</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>

  </model>
</sdf>
```

### Key Annotations

1. **IMU**: Mounted on torso; publishes to `/robot/imu` at 100 Hz with realistic noise.
2. **LiDAR**: 2D planar scan (360°, 10 Hz); publishes to `/robot/scan`.
3. **Camera**: Head-mounted (640×480, 30 Hz); publishes to `/robot/camera/image_raw`.
4. **Simplified legs**: Single-link legs (real humanoids have 3+ links per leg: thigh, shin, foot).

### Usage

1. Save as `simple_biped.sdf` in `~/.gazebo/models/simple_biped/model.sdf`.
2. Create `model.config`:

   ```xml
   <?xml version="1.0"?>
   <model>
     <name>simple_biped</name>
     <version>1.0</version>
     <sdf version="1.6">model.sdf</sdf>
     <author><name>Your Name</name></author>
     <description>Simplified bipedal robot with sensors</description>
   </model>
   ```

3. Include in world file or spawn via ROS 2.

---

## Example 3: Unity Scene Setup for Humanoid Visualization (Conceptual)

This example outlines the steps to create a Unity scene for visualizing a humanoid robot (descriptive, not C# code).

### Scenario

Import a humanoid robot (from URDF) into Unity and create a photorealistic living room environment for HRI visualization.

### Steps

#### 1. Create a New Unity Project

- Template: **3D (URP)** (Universal Render Pipeline for high-quality graphics).
- Project name: `HumanoidVisualization`.

#### 2. Install Unity Robotics Hub

- **Window → Package Manager → Add package from git URL**:
  - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
  - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

#### 3. Import Humanoid Robot

- **Assets → Import Robot from URDF**.
- Select your `humanoid.urdf` file.
- Unity creates a GameObject hierarchy with ArticulationBody components.

#### 4. Create Living Room Environment

- **GameObject → 3D Object → Plane** (floor, scale to 10×10 m).
- **Asset Store** (Window → Asset Store):
  - Search "living room" or "furniture pack".
  - Import free assets (sofa, table, lamp, rug).
- Position furniture in the scene (drag-and-drop).

#### 5. Add Lighting

- **GameObject → Light → Directional Light** (sunlight through window).
  - Intensity: 1.5, Color: Warm white (slight yellow tint).
- **GameObject → Light → Point Light** (table lamp).
  - Intensity: 2.0, Range: 5 m.
- **Window → Rendering → Lighting**:
  - Enable **Baked Global Illumination** for soft shadows.
  - Click **Generate Lighting**.

#### 6. Apply PBR Materials to Robot

- Select robot links (e.g., torso, head).
- **Inspector → Material**:
  - Albedo: Light grey (0.7, 0.7, 0.7).
  - Metallic: 0.8 (metallic look).
  - Smoothness: 0.6 (semi-glossy).
  - Normal Map: (Optional) Add surface details.

#### 7. Add Post-Processing

- **GameObject → Volume → Global Volume**.
- **Inspector → Profile → Add Override**:
  - **Bloom**: Intensity = 0.2 (soft glow on highlights).
  - **Depth of Field**: Focus Distance = 3 m (blur background).
  - **Color Grading**: Contrast = 10, Saturation = 5 (cinematic look).

#### 8. Configure ROS-TCP Connector

- **Robotics → ROS Settings**:
  - ROS IP: `127.0.0.1` (or remote machine IP).
  - ROS Port: `10000`.
- Create C# script `JointStateSubscriber.cs` (see Chapter 4) to sync robot joints with ROS.

#### 9. Add Cameras

- **GameObject → Camera** (main view).
  - Position: (5, 2, -3), Rotation: Look at robot.
- **GameObject → Camera** (robot's head camera).
  - Parent to robot's head link.
  - Attach `CameraPublisher.cs` script to publish images to ROS.

#### 10. Test the Scene

- **File → Build Settings → Build and Run** (or play in Editor).
- Launch ROS-TCP Endpoint:

  ```bash
  ros2 run ros_tcp_endpoint default_server_endpoint
  ```

- Publish joint states from ROS:

  ```bash
  ros2 topic pub /robot/joint_states sensor_msgs/msg/JointState ...
  ```

- Observe robot moving in Unity in real-time.

### Key Points

- **PBR materials**: Make the robot look realistic (not flat/cartoon).
- **Lighting**: Baked GI + real-time lights = photorealistic shadows.
- **Post-processing**: Bloom, depth-of-field, color grading = cinematic quality.
- **ROS integration**: Unity subscribes to joint states, publishes camera images.

---

## Example 4: Sensor Data Visualization with RViz

This example shows how to visualize simulated sensor data (LiDAR, camera, depth) in RViz.

### Scenario

Launch Gazebo with a humanoid robot (from Example 2), visualize all sensors in RViz.

### Steps

#### 1. Launch Gazebo with Robot

```bash
gazebo --verbose humanoid_indoor.world
ros2 run gazebo_ros spawn_entity.py -entity simple_biped -file simple_biped.sdf -x 0 -y 0 -z 1.0
```

#### 2. Launch RViz

```bash
ros2 run rviz2 rviz2
```

#### 3. Configure RViz Display

1. **Set Fixed Frame**: `torso` (or `base_link`).

2. **Add Robot Model**:
   - **Add → RobotModel**.
   - **Description Topic**: `/robot_description`.

3. **Add LaserScan** (LiDAR):
   - **Add → LaserScan**.
   - **Topic**: `/robot/scan`.
   - **Size**: 0.05 (point size).
   - **Color**: Red (255, 0, 0).

4. **Add Image** (RGB Camera):
   - **Add → Image**.
   - **Image Topic**: `/robot/camera/image_raw`.

5. **Add PointCloud2** (Depth Camera, if available):
   - **Add → PointCloud2**.
   - **Topic**: `/robot/camera/depth/points`.
   - **Color Transformer**: AxisColor (Z-axis) or Intensity.

6. **Add IMU** (optional, using rviz_imu_plugin):
   - **Add → Imu** (requires `rviz_imu_plugin` package).
   - **Topic**: `/robot/imu`.

#### 4. Verify Data Flow

- **LiDAR**: Red points should appear around the robot, outlining obstacles.
- **Camera**: RGB image should show the scene from the robot's head.
- **Depth**: Point cloud should show 3D structure of objects in view.
- **IMU**: Arrow should indicate gravity direction (pointing down).

#### 5. Save RViz Config

- **File → Save Config As**: `humanoid_sensors.rviz`.
- Reuse later:

  ```bash
  ros2 run rviz2 rviz2 -d humanoid_sensors.rviz
  ```

### Troubleshooting

| **Issue** | **Cause** | **Solution** |
|-----------|-----------|------------|
| **No LaserScan data** | Topic mismatch | Check topic name: `ros2 topic list`, update RViz topic. |
| **Camera image is black** | Sensor not publishing | Verify plugin loaded: `ros2 topic echo /robot/camera/image_raw` (should see data). |
| **PointCloud2 is empty** | Depth camera range issue | Ensure objects are within sensor range (0.3–10 m). |
| **IMU arrow points wrong** | Frame orientation | Check `frame_name` in IMU plugin matches RViz fixed frame. |

---

## Example 5: Complete ROS 2 Launch File for Gazebo + RViz

This example integrates Gazebo (simulation) and RViz (visualization) in a single launch file.

### Launch File: `humanoid_simulation.launch.py`

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('my_humanoid_description')

    # Paths to files
    world_file = os.path.join(pkg_share, 'worlds', 'humanoid_indoor.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'humanoid_sensors.rviz')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 2. Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # 3. Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # 4. Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz,
    ])
```

### Usage

```bash
ros2 launch my_humanoid_description humanoid_simulation.launch.py
```

**What happens**:
1. Gazebo starts, loads `humanoid_indoor.world`.
2. Robot is spawned at (0, 0, 1.0).
3. `robot_state_publisher` broadcasts TF transforms.
4. RViz launches with pre-configured sensor displays.

---

## Summary

These examples demonstrate:

1. **Gazebo world file**: Complete indoor environment with physics, lighting, obstacles, and stairs.
2. **SDF robot model**: Simplified bipedal robot with IMU, LiDAR, and camera plugins.
3. **Unity scene setup**: Step-by-step guide to importing a robot and creating a photorealistic environment.
4. **RViz visualization**: Configure LaserScan, Image, PointCloud2, and IMU displays.
5. **ROS 2 launch file**: Integrated Gazebo + RViz launch with robot spawning and state publishing.

Use these examples as starting points for your own humanoid simulation projects. Modify world files, sensor configurations, and visualization settings to match your specific use case.

---

**Navigation**
← [Chapter 5: Sensor Simulation](/digital-twin/sensor-simulation)
→ [Exercises](/digital-twin/exercises)
