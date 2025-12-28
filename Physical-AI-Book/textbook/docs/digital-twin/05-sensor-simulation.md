---
title: "Chapter 5: Sensor Simulation"
slug: /digital-twin/sensor-simulation
sidebar_label: "5. Sensor Simulation"
sidebar_position: 6
toc: true
description: "Master sensor simulation for humanoid robots: LiDAR ray-casting, depth cameras, IMU noise models, camera distortion, and testing perception algorithms with synthetic data."
---

# Chapter 5: Sensor Simulation

## Introduction

Modern humanoid robots are **perception-driven systems**: they use cameras to recognize objects, LiDAR to map environments, IMUs to estimate orientation, and depth sensors to plan grasps. Before deploying perception algorithms on physical hardware—where sensor failures, calibration errors, and environmental variability can cause costly failures—engineers validate these algorithms in simulation.

**Sensor simulation** is the process of generating synthetic sensor data that mimics real-world sensors as closely as possible. This chapter provides a comprehensive guide to simulating the four most critical sensor types for humanoid robotics:

1. **LiDAR (Light Detection and Ranging)**: Ray-casting for distance measurements and point clouds.
2. **Depth cameras (RGB-D)**: Depth maps and point clouds for manipulation and navigation.
3. **IMUs (Inertial Measurement Units)**: Accelerometer and gyroscope data for state estimation.
4. **RGB cameras**: Images with lens distortion and noise for vision algorithms.

For each sensor, you will learn the **underlying physics**, **simulation techniques**, **noise models**, **configuration in Gazebo/Unity**, and **validation strategies** to ensure simulated data matches real-world characteristics. By the end, you will be able to configure realistic sensor plugins, test perception algorithms with synthetic data, and identify sim-to-real gaps in sensor modeling.

## LiDAR Simulation: Ray-Casting and Point Clouds

### What is LiDAR?

**LiDAR (Light Detection and Ranging)** emits laser pulses and measures the time-of-flight (ToF) to determine distance to objects. Humanoid robots use LiDAR for:

- **SLAM (Simultaneous Localization and Mapping)**: Build 3D maps of environments.
- **Obstacle detection**: Identify walls, furniture, humans for navigation.
- **3D reconstruction**: Create dense point clouds for manipulation planning.

### Types of LiDAR

1. **2D LiDAR** (planar scan):
   - Emits lasers in a single horizontal plane (e.g., 360° around the robot).
   - Common models: SICK LMS100, Hokuyo URG-04LX.
   - **Use case**: Mobile robot navigation (floor-level obstacle detection).

2. **3D LiDAR** (volumetric scan):
   - Multiple laser beams at different vertical angles (e.g., 64 beams spanning -25° to +15°).
   - Common models: Velodyne VLP-16 (16 beams), Ouster OS1-64 (64 beams).
   - **Use case**: Autonomous vehicles, humanoid navigation in complex environments.

### Simulation Method: Ray-Casting

LiDAR simulation uses **ray-casting**: for each laser beam, trace a ray through the 3D scene and record the distance to the first intersection.

#### Algorithm

```
For each beam angle θ (horizontal) and φ (vertical):
  1. Compute ray direction: d = [cos(φ)cos(θ), cos(φ)sin(θ), sin(φ)]
  2. Cast ray from sensor origin along direction d
  3. Find first intersection with scene geometry (walls, objects)
  4. Compute distance: r = ||intersection_point - sensor_origin||
  5. Add point to point cloud: P = sensor_origin + r * d
  6. (Optional) Compute intensity based on surface reflectance
```

#### Performance Considerations

- **Ray count**: A Velodyne VLP-16 casts ~30,000 rays per second (16 beams × 360° × 5 Hz rotation).
- **Collision detection**: Each ray requires a scene intersection test—expensive for complex meshes.
- **Optimization**: Use spatial data structures (BVH, octrees) to accelerate ray-mesh intersections.

**Simulation speed**: In Gazebo, a 16-beam LiDAR typically reduces simulation speed to 0.5–1× real-time (from 2–5× without LiDAR).

### Configuring LiDAR in Gazebo

#### Example: 2D Planar LiDAR

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_scanner">
    <pose>0 0 0.1 0 0 0</pose>
    <update_rate>10</update_rate>  <!-- 10 Hz -->
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>         <!-- 720 rays (0.5° resolution) -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180° -->
          <max_angle>3.14159</max_angle>  <!-- +180° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>   <!-- Min range: 0.1 m -->
        <max>30.0</max>  <!-- Max range: 30 m -->
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1 cm standard deviation -->
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
</gazebo>
```

**Output**: Publishes to `/robot/scan` (sensor_msgs/LaserScan).

**Visualization in RViz**:

```bash
ros2 run rviz2 rviz2
# Add → LaserScan → Topic: /robot/scan
```

#### Example: 3D Volumetric LiDAR (Velodyne VLP-16)

```xml
<gazebo reference="velodyne_link">
  <sensor type="ray" name="velodyne">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>          <!-- 1800 samples × 16 beams = 28,800 rays -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>            <!-- 16 vertical beams -->
          <resolution>1</resolution>
          <min_angle>-0.2617993877991494</min_angle>  <!-- -15° -->
          <max_angle>0.2617993877991494</max_angle>   <!-- +15° -->
        </vertical>
      </scan>
      <range>
        <min>0.9</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>  <!-- 2 cm noise -->
      </noise>
    </ray>
    <plugin name="velodyne_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=velodyne/points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>velodyne_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Output**: Publishes to `/robot/velodyne/points` (sensor_msgs/PointCloud2).

**Visualization in RViz**:

```bash
# Add → PointCloud2 → Topic: /robot/velodyne/points
# Color by: Intensity or Z-axis
```

### LiDAR Noise Models

Real LiDAR sensors have several imperfections:

1. **Gaussian distance noise**: σ = 1–3 cm for most sensors.
2. **Outliers (ghost points)**: Caused by multi-path reflections, rain, fog.
   - **Simulation**: Randomly replace 1–5% of measurements with far-away points (e.g., max_range).
3. **Intensity variation**: Reflectance depends on surface material (dark surfaces return weak signals).
   - **Simulation**: Compute intensity as a function of surface color/texture (not standard in Gazebo; requires custom plugin).
4. **Missed detections**: Black or transparent surfaces may not return measurements.
   - **Simulation**: Mark rays that hit transparent materials as "invalid" (NaN or max_range).

**Example: Adding outliers (custom Gazebo plugin or post-processing)**:

```python
import numpy as np

def add_lidar_outliers(point_cloud, outlier_rate=0.02, max_range=30.0):
    """Add random outliers to simulated LiDAR point cloud."""
    num_points = len(point_cloud)
    num_outliers = int(num_points * outlier_rate)
    outlier_indices = np.random.choice(num_points, num_outliers, replace=False)

    for idx in outlier_indices:
        # Replace point with a far-away point (simulating multi-path)
        direction = point_cloud[idx] / np.linalg.norm(point_cloud[idx])
        point_cloud[idx] = direction * max_range

    return point_cloud
```

### Validating LiDAR Simulation

**Test 1: Range accuracy**

1. Place a flat wall 5 m from the LiDAR in simulation.
2. Measure distances in the point cloud (should be ~5 m ± noise).
3. Compare to real LiDAR measurements (collect 100 scans, compute mean/std).

**Test 2: Angular resolution**

1. Place two narrow objects 10° apart.
2. Verify they appear as separate clusters in the point cloud (if angular resolution < 10°).

**Test 3: Edge effects**

1. Scan a box edge (sharp corner).
2. Real LiDAR often produces "ghost points" near edges (beam hits two surfaces).
3. Compare simulated vs. real edge artifacts.

## Depth Camera Simulation: RGB-D Sensors

### What is a Depth Camera?

A **depth camera (RGB-D sensor)** provides both an RGB image and a depth map (distance to each pixel). Common technologies:

1. **Structured light** (e.g., Kinect v1, Intel RealSense D400):
   - Projects an IR pattern, measures distortion to compute depth.
   - Range: 0.3–10 m, accuracy: ±1–2 cm.

2. **Time-of-flight (ToF)** (e.g., Kinect v2, Azure Kinect):
   - Measures IR light travel time.
   - Range: 0.5–8 m, accuracy: ±1–3 cm.

3. **Stereo vision** (e.g., ZED, RealSense D435):
   - Uses two cameras to compute depth via triangulation.
   - Range: 0.3–20 m (depends on baseline), accuracy: ±2–5 cm.

### Simulation Method: Z-Buffer Rendering

Depth cameras are simulated by rendering the scene and extracting the **Z-buffer** (depth buffer) from the graphics pipeline.

#### Algorithm

```
1. Render scene from camera's viewpoint (OpenGL/Vulkan)
2. For each pixel (u, v):
   - Read Z-buffer value: z (distance in camera frame)
   - If z < min_range or z > max_range: mark as invalid (NaN)
   - Else: depth[u, v] = z
3. Add noise: depth[u, v] += N(0, σ) where σ is sensor-dependent
4. (Optional) Simulate edge artifacts, invalid regions (reflective/black surfaces)
```

### Configuring Depth Camera in Gazebo

#### Example: Intel RealSense D435-like Sensor

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30.0</update_rate>  <!-- 30 Hz -->
    <camera>
      <horizontal_fov>1.211</horizontal_fov>  <!-- 69.4° (RealSense D435 spec) -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.3</near>   <!-- Min range: 0.3 m -->
        <far>10.0</far>    <!-- Max range: 10 m -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.015</stddev>  <!-- 1.5 cm noise -->
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/rgb/image_raw</remapping>
        <remapping>depth/image_raw:=camera/depth/image_raw</remapping>
        <remapping>points:=camera/depth/points</remapping>
        <remapping>camera_info:=camera/rgb/camera_info</remapping>
        <remapping>depth/camera_info:=camera/depth/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- Stereo baseline for point cloud -->
    </plugin>
  </sensor>
</gazebo>
```

**Output**: Publishes:
- `/robot/camera/rgb/image_raw` (RGB image)
- `/robot/camera/depth/image_raw` (depth map, 32-bit float)
- `/robot/camera/depth/points` (point cloud, sensor_msgs/PointCloud2)

**Visualization in RViz**:

```bash
# RGB image: Add → Image → Topic: /robot/camera/rgb/image_raw
# Depth map: Add → Image → Topic: /robot/camera/depth/image_raw
# Point cloud: Add → PointCloud2 → Topic: /robot/camera/depth/points
```

### Depth Camera Noise Models

Real depth cameras exhibit complex error patterns:

1. **Gaussian noise**: σ = 1–3 cm (increases with distance).
   - **Model**: σ(d) = σ₀ + k × d² (noise grows quadratically with distance).

2. **Invalid pixels**: Reflective, transparent, or black surfaces return no depth.
   - **Simulation**: Mark pixels as NaN if they hit materials with low IR reflectance.

3. **Edge artifacts (flying pixels)**: Depth discontinuities (object edges) produce spurious points.
   - **Simulation**: Detect edges in depth map; add random points near edges.

4. **Temporal noise**: Depth values flicker frame-to-frame.
   - **Simulation**: Add time-correlated noise (Ornstein-Uhlenbeck process).

#### Example: Distance-Dependent Noise (Python Post-Processing)

```python
def add_depth_noise(depth_image, sigma_0=0.01, k=0.0002):
    """
    Add distance-dependent noise to depth image.

    Args:
        depth_image: HxW array of depth values (meters)
        sigma_0: Base noise (meters)
        k: Quadratic noise coefficient

    Returns:
        Noisy depth image
    """
    noise_stddev = sigma_0 + k * depth_image**2
    noise = np.random.normal(0, noise_stddev)
    noisy_depth = depth_image + noise
    noisy_depth[noisy_depth < 0.3] = np.nan  # Invalid if too close
    noisy_depth[noisy_depth > 10.0] = np.nan  # Invalid if too far
    return noisy_depth
```

### Validating Depth Camera Simulation

**Test 1: Depth accuracy**

1. Place a planar surface (wall, table) at known distances (1 m, 3 m, 5 m).
2. Measure mean depth in simulated depth map.
3. Compare to real sensor measurements (should match within ±2 cm).

**Test 2: Invalid regions**

1. Place a black cloth or mirror in the scene.
2. Verify that simulated depth map has NaN or zero values in those regions (matching real sensor behavior).

**Test 3: Edge artifacts**

1. Capture depth of a box edge (sharp depth discontinuity).
2. Real sensors produce "flying pixels" (spurious points near the edge).
3. Compare simulated edge profile to real data (may require custom post-processing).

## IMU Simulation: Accelerometers and Gyroscopes

### What is an IMU?

An **Inertial Measurement Unit (IMU)** measures:

- **Accelerometer**: Linear acceleration (including gravity) in sensor frame (m/s²).
- **Gyroscope**: Angular velocity (rotation rates) in sensor frame (rad/s).
- **(Optional) Magnetometer**: Earth's magnetic field (for heading estimation).

Humanoid robots use IMUs for:

- **State estimation**: Fuse IMU with joint encoders to estimate base orientation (pitch, roll).
- **Balance control**: Gyroscope feedback for upright posture.
- **Fall detection**: Sudden accelerations indicate collisions or falls.

### Simulation Method: Physics-Based Acceleration Computation

IMU data is computed from the robot's motion in the physics engine:

1. **Accelerometer**:
   - Measures specific force: **a_measured = a_body + g**
   - Where **a_body** is the link's acceleration (from physics), **g** is gravity.
   - Includes centrifugal and Coriolis effects if the robot is rotating.

2. **Gyroscope**:
   - Measures angular velocity: **ω_measured = ω_body**
   - Where **ω_body** is the link's angular velocity (from physics).

3. **Noise**: Real IMUs have bias, drift, and white noise.

### Configuring IMU in Gazebo

#### Example: Consumer-Grade IMU (MPU6050-like)

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>  <!-- 0.0002 rad/s = 0.011°/s -->
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00005</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00005</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00005</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- 0.017 m/s² = 0.0017 g -->
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.005</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.005</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.005</bias_stddev>
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
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Output**: Publishes to `/robot/imu` (sensor_msgs/Imu).

### IMU Noise Parameters (Typical Values)

| **Sensor** | **Noise Type** | **Gyroscope (rad/s)** | **Accelerometer (m/s²)** |
|------------|----------------|----------------------|-------------------------|
| **Consumer (MPU6050)** | White noise (σ) | 0.0002 (0.011°/s) | 0.017 (0.0017 g) |
| | Bias (mean) | 0.0001 | 0.01 |
| **Industrial (Xsens MTi)** | White noise (σ) | 0.00005 (0.003°/s) | 0.003 (0.0003 g) |
| | Bias (mean) | 0.00001 | 0.001 |
| **MEMS (high-end)** | White noise (σ) | 0.00001 (0.0006°/s) | 0.001 (0.0001 g) |
| | Bias (mean) | 0.000001 | 0.0001 |

**How to determine noise for your sensor**:
1. Place the real sensor stationary for 10 minutes.
2. Record IMU data.
3. Compute standard deviation (σ) and mean (bias) for each axis.
4. Use these values in your simulation.

### Validating IMU Simulation

**Test 1: Gravity measurement**

1. Place the robot stationary in simulation.
2. Accelerometer Z-axis should read ~9.81 m/s² (gravity).
3. Compare to real sensor (calibrate for local gravity: 9.78–9.83 m/s² depending on latitude).

**Test 2: Rotation rate**

1. Rotate the robot in simulation at 10°/s (0.1745 rad/s).
2. Gyroscope should measure ~0.1745 rad/s ± noise.
3. Compare to real sensor during a controlled rotation (e.g., on a turntable).

**Test 3: Bias drift**

1. Run simulation for 10 minutes (simulated time).
2. Plot gyroscope readings—should see slow drift due to bias.
3. Real IMUs drift 1–10°/hour; ensure simulation matches.

## RGB Camera Simulation: Lens Distortion and Noise

### What is an RGB Camera?

RGB cameras capture color images (Red, Green, Blue channels). Humanoid robots use cameras for:

- **Object recognition**: Detect humans, tools, obstacles.
- **Visual servoing**: Control arm movements based on camera feedback.
- **SLAM**: Visual odometry and feature tracking.

### Simulation Method: 3D Rendering + Post-Processing

RGB cameras are simulated by:

1. **Rendering**: Use OpenGL/Vulkan to render the scene from the camera's viewpoint.
2. **Intrinsics**: Apply perspective projection based on camera intrinsics (focal length, principal point).
3. **Distortion**: Apply radial and tangential distortion (barrel/pincushion effects).
4. **Noise**: Add pixel-level noise (Gaussian, salt-and-pepper).

### Camera Intrinsics and Distortion

A real camera's image formation is modeled by:

1. **Intrinsics matrix** (K):

   ```
   K = [ fx  0  cx ]
       [  0 fy  cy ]
       [  0  0   1 ]
   ```

   - fx, fy: Focal lengths (pixels)
   - cx, cy: Principal point (image center, pixels)

2. **Distortion coefficients**:
   - **Radial distortion** (k1, k2, k3): Barrel/pincushion effect.
   - **Tangential distortion** (p1, p2): Lens misalignment.

**Distortion model** (OpenCV convention):

```
x_distorted = x(1 + k1*r² + k2*r⁴ + k3*r⁶) + 2*p1*x*y + p2*(r² + 2*x²)
y_distorted = y(1 + k1*r² + k2*r⁴ + k3*r⁶) + p1*(r² + 2*y²) + 2*p2*x*y
```

Where r² = x² + y² (distance from center).

### Configuring RGB Camera in Gazebo

#### Example: Logitech C920 Webcam-like Sensor

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="rgb_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80° -->
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- 0.007 on [0, 1] scale → ~2 intensity on [0, 255] -->
      </noise>
      <distortion>
        <k1>-0.1</k1>   <!-- Radial distortion (barrel effect) -->
        <k2>0.05</k2>
        <k3>0.0</k3>
        <p1>0.001</p1>  <!-- Tangential distortion -->
        <p2>0.001</p2>
        <center>0.5 0.5</center>  <!-- Distortion center (normalized) -->
      </distortion>
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

**Output**: Publishes:
- `/robot/camera/image_raw` (RGB image)
- `/robot/camera/camera_info` (intrinsics, distortion)

### Camera Noise Models

1. **Gaussian noise**: Independent noise per pixel (σ = 0.005–0.01).
2. **Salt-and-pepper noise**: Random pixels set to black (0) or white (255).
3. **Motion blur**: If camera is moving (simulated by temporal averaging).
4. **Low-light noise**: Noise increases in dark regions (Poisson noise model).

**Example: Low-light noise (Python post-processing)**:

```python
def add_low_light_noise(image, gain=2.0):
    """Simulate low-light camera noise (Poisson + Gaussian)."""
    # Convert to float [0, 1]
    img = image.astype(np.float32) / 255.0

    # Poisson noise (photon shot noise)
    img_noisy = np.random.poisson(img * 255 * gain) / (255 * gain)

    # Gaussian read noise
    read_noise = np.random.normal(0, 0.01, img.shape)
    img_noisy = img_noisy + read_noise

    # Clip and convert back
    img_noisy = np.clip(img_noisy, 0, 1) * 255
    return img_noisy.astype(np.uint8)
```

### Validating Camera Simulation

**Test 1: Intrinsic calibration**

1. Capture images of a checkerboard in simulation.
2. Run OpenCV's `calibrateCamera()` on simulated images.
3. Recovered intrinsics should match the configured values (fx, fy, cx, cy).

**Test 2: Distortion model**

1. Capture images of a grid pattern in simulation.
2. Measure line curvature (radial distortion causes straight lines to curve).
3. Compare to real camera images—curvature should match if distortion coefficients are correct.

**Test 3: Noise characteristics**

1. Capture 100 images of a static scene in simulation.
2. Compute per-pixel standard deviation (temporal noise).
3. Compare to real camera (should have similar noise profile).

## Testing Perception Algorithms with Synthetic Data

Sensor simulation enables **algorithm validation** before hardware deployment.

### Workflow: SLAM Testing

1. **Create a simulation environment**: Design a 20 m × 20 m warehouse in Gazebo with boxes, walls, and clutter.
2. **Configure sensors**: Add a 2D LiDAR and RGB camera to the humanoid robot.
3. **Run SLAM algorithm**: Launch `slam_toolbox` or `cartographer_ros` in ROS 2.
4. **Teleoperate the robot**: Drive the robot through the environment (keyboard, joystick).
5. **Evaluate performance**:
   - Compare generated map to ground-truth (world file).
   - Measure localization error (simulated pose vs. estimated pose).
   - Count loop closure successes/failures.

### Workflow: Object Detection Testing

1. **Generate synthetic dataset**: Capture 10,000 images in Unity with domain randomization (lighting, object placement, textures).
2. **Train detector**: Train YOLOv8 on synthetic data.
3. **Test in simulation**: Deploy detector in Gazebo, measure detection accuracy (precision, recall).
4. **Sim-to-real transfer**: Test on real robot camera; fine-tune with 100–1,000 real images.

### Metrics for Perception Validation

| **Task** | **Metric** | **Simulation** | **Real-World Target** |
|----------|-----------|----------------|----------------------|
| **SLAM** | Map accuracy (m) | &lt;0.1 m error | &lt;0.05 m error |
| | Loop closure rate | >90% | >95% |
| **Object detection** | mAP (mean Average Precision) | 0.70–0.80 (sim) | >0.80 (real) |
| **Depth estimation** | RMSE (meters) | &lt;0.02 m | &lt;0.03 m |
| **IMU-based pose** | Roll/pitch error (deg) | &lt;2° | &lt;3° |

## Summary

Sensor simulation is critical for developing and validating perception algorithms before hardware deployment. Key takeaways:

1. **LiDAR**: Simulated via ray-casting; add Gaussian noise (1–3 cm), outliers (1–5%), and range limits. Configured in Gazebo with `<sensor type="ray">`.

2. **Depth cameras**: Simulated via Z-buffer rendering; add distance-dependent noise (σ = 1–3 cm), invalid pixels (reflective/black surfaces), and edge artifacts. Configured with `<sensor type="depth">`.

3. **IMUs**: Simulated from physics engine (acceleration, angular velocity); add white noise (gyro: 0.0002 rad/s, accel: 0.017 m/s²), bias (gyro: 0.0001 rad/s, accel: 0.01 m/s²), and drift. Configured with `<sensor type="imu">`.

4. **RGB cameras**: Simulated via 3D rendering; apply lens distortion (radial/tangential), add Gaussian noise (σ = 0.007), and (optionally) low-light or motion blur effects. Configured with `<sensor type="camera">`.

5. **Validation strategy**: Compare simulated sensor data to real hardware (measure noise, bias, artifacts); tune simulation parameters to match real-world characteristics.

6. **Testing perception algorithms**: Use synthetic data to train/test SLAM, object detection, depth estimation—achieving 70–90% of real-world performance before hardware testing.

In the next section, you will see **code examples** demonstrating sensor configuration, data visualization, and perception algorithm integration.

## Self-Check Questions

1. **Explain ray-casting for LiDAR**: How many rays does a 64-beam LiDAR cast per second if it rotates at 10 Hz with 0.2° horizontal resolution?

2. **Design a depth camera noise model**: You measure a planar wall at 2 m, 4 m, and 6 m with a real depth camera. The standard deviations are 0.015 m, 0.025 m, and 0.045 m. Derive a quadratic noise model σ(d) = σ₀ + k × d² and estimate σ₀ and k.

3. **Configure an IMU**: You have a real IMU with gyroscope white noise 0.0003 rad/s and bias 0.0002 rad/s. Write the Gazebo XML snippet to match this sensor.

4. **Validate camera distortion**: Describe a procedure to measure radial distortion coefficient k1 from simulated camera images. What test pattern would you use, and what measurements would you take?

5. **Compare LiDAR and depth cameras**: For humanoid navigation, when would you prefer a 2D LiDAR over a depth camera? When would you prefer the depth camera? List two scenarios for each.

6. **Sim-to-real gap analysis**: You train an object detector on 50,000 synthetic images (Unity) and achieve 85% mAP in simulation, but only 60% mAP on real robot images. List three likely causes of the gap and propose mitigation strategies.

## Next Steps

Proceed to **Examples** (`examples.md`) to see full Gazebo/Unity configurations, sensor visualization in RViz, and perception algorithm integration code.

---

**Chapter Navigation**
← [Chapter 4: Unity High-Fidelity Rendering](/digital-twin/unity-high-fidelity-rendering)
→ [Examples](/digital-twin/examples)
