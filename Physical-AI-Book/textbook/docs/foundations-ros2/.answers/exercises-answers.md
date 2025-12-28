---
title: "Module 1: Instructor Solutions"
sidebar_label: "Instructor Solutions"
description: "Complete solutions, grading rubrics, and common mistakes for Module 1 exercises"
---

# Module 1: Instructor Solutions (ROS 2 Foundations)

**Note**: This file is intended for instructors only and should be excluded from student-facing materials.

---

## Chapter 1: What is ROS 2?

### Exercise 1.1: Communication Paradigm Comparison 📘

**Difficulty**: Recall | **Time**: 15 min

#### Solution Overview
Students should create a comparison table highlighting the structural and behavioral differences between pub-sub (topics), request-response (services), and goal-feedback-result (actions).

#### Complete Solution

| **Aspect** | **Topic (Pub-Sub)** | **Service (Req-Res)** | **Action (Goal-Feedback-Result)** |
|------------|---------------------|-----------------------|-----------------------------------|
| **Communication** | Asynchronous, many-to-many | Synchronous, one-to-one | Asynchronous, one-to-one with feedback |
| **Blocking** | Non-blocking | Blocking (waits for response) | Non-blocking (async with callbacks) |
| **Use case** | Continuous sensor streams | One-off requests | Long-running tasks with progress |
| **Cancellation** | N/A | N/A | Built-in cancellation support |
| **Feedback** | None (fire-and-forget) | Single response only | Continuous feedback during execution |
| **Example** | `/camera/image` (30 FPS) | `/reset_odometry` | `/navigate_to_goal` (30s task) |

#### Grading Rubric (Total: 10 points)
- **Correctness** (6 pts): Accurate descriptions of each paradigm
  - Topics: asynchronous, many-to-many (2 pts)
  - Services: synchronous, one-to-one (2 pts)
  - Actions: long-running with feedback (2 pts)
- **Use cases** (2 pts): Appropriate examples for each
- **Completeness** (2 pts): All 6 comparison dimensions covered

#### Common Mistakes
- Confusing topics with services (thinking topics are synchronous)
- Not mentioning cancellation as a key feature of actions
- Using inappropriate examples (e.g., `/camera/image` as a service)

#### Extensions
- Add QoS reliability comparison (reliable vs. best-effort for topics)
- Diagram the data flow for each paradigm
- Research real-world humanoid robot architectures (Atlas, Digit)

---

### Exercise 1.2: DDS and QoS Concepts 📘

**Difficulty**: Recall | **Time**: 20 min

#### Solution Overview
Explain DDS's peer-to-peer architecture, automatic discovery, and QoS policies.

#### Complete Solution

**1. Peer-to-peer architecture**:
- No central broker (unlike ROS 1's `roscore` or MQTT broker)
- Nodes discover each other automatically via multicast
- Eliminates single point of failure
- Scales better for multi-robot systems

**2. Automatic discovery**:
- Nodes broadcast presence on network
- Subscribers match with publishers by topic name + message type
- QoS compatibility checked during discovery
- No manual IP configuration required

**3. QoS reliability policies**:

| **Policy** | **Behavior** | **Use Case** |
|------------|--------------|--------------|
| **Reliable** | Retransmits lost packets (TCP-like) | Critical data: `/robot_description`, `/joint_commands` |
| **Best Effort** | No retransmits (UDP-like) | High-frequency streams: `/camera/image` (30 FPS) |

**4. Durability policies**:

| **Policy** | **Behavior** | **Use Case** |
|------------|--------------|--------------|
| **Volatile** | Discard old messages | Real-time sensor data |
| **Transient Local** | Keep last N messages for late joiners | Configuration: `/robot_description` |

#### Grading Rubric (Total: 10 points)
- **DDS architecture** (3 pts): Peer-to-peer, no broker, discovery
- **QoS reliability** (3 pts): Reliable vs. Best Effort with examples
- **QoS durability** (2 pts): Volatile vs. Transient Local
- **Real-world context** (2 pts): Appropriate use cases for humanoid robots

#### Common Mistakes
- Thinking DDS requires a central server (it doesn't)
- Using Reliable QoS for all topics (wasteful for high-frequency data)
- Not understanding compatibility (Best Effort publisher can't serve Reliable subscriber)

---

### Exercise 1.3: Simple Publisher Implementation 📗

**Difficulty**: Application | **Time**: 30 min

#### Complete Solution

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header

class TemperatureSensorPublisher(Node):
    def __init__(self):
        super().__init__('temperature_sensor_publisher')

        # Publisher with QoS depth 10
        self.publisher_ = self.create_publisher(Temperature, '/humanoid/temperature', 10)

        # Timer: 1 Hz (every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.temperature_reading = 36.5  # Celsius (normal human body temp)
        self.get_logger().info('Temperature sensor publisher started')

    def timer_callback(self):
        msg = Temperature()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate temperature fluctuation (+/- 0.5°C)
        import random
        msg.temperature = self.temperature_reading + random.uniform(-0.5, 0.5)
        msg.variance = 0.1  # Sensor variance

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published temperature: {msg.temperature:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Explanation
1. **Node initialization**: `super().__init__('temperature_sensor_publisher')`
2. **Publisher**: `create_publisher(Temperature, '/humanoid/temperature', 10)` - QoS depth 10
3. **Timer**: `create_timer(1.0, callback)` - 1 Hz publishing rate
4. **Message**: Uses `sensor_msgs/Temperature` with header (timestamp, frame_id)
5. **Timestamp**: `self.get_clock().now()` - ROS 2 time (simulation-compatible)

#### Expected Output
```
[INFO] [temperature_sensor_publisher]: Temperature sensor publisher started
[INFO] [temperature_sensor_publisher]: Published temperature: 36.72°C
[INFO] [temperature_sensor_publisher]: Published temperature: 36.38°C
[INFO] [temperature_sensor_publisher]: Published temperature: 36.91°C
```

#### Grading Rubric (Total: 10 points)
- **Publisher creation** (3 pts): Correct message type, topic name, QoS
- **Timer setup** (2 pts): 1 Hz callback
- **Message structure** (3 pts): Header with timestamp, temperature field
- **Logging** (1 pt): Informative console output
- **Code quality** (1 pt): Clean, commented, follows conventions

#### Common Mistakes
- Forgetting to initialize ROS 2 (`rclpy.init()`)
- Not sourcing ROS 2 setup (`source /opt/ros/humble/setup.bash`)
- Using hardcoded timestamps instead of `self.get_clock().now()`
- Not setting `frame_id` in header

---

## Chapter 2: Nodes, Topics, and Services

### Exercise 2.1: QoS Introspection 📗

**Difficulty**: Application | **Time**: 20 min

#### Complete Solution

**Commands**:
```bash
# 1. List all active topics
ros2 topic list

# 2. Get detailed info on /camera/image
ros2 topic info /camera/image -v
```

**Expected Output**:
```
Topic: /camera/image
Publisher count: 1
Subscription count: 2

Publisher #1:
  Node name: camera_driver
  QoS profile:
    Reliability: BEST_EFFORT
    Durability: VOLATILE
    History (depth): KEEP_LAST (1)
    Deadline: Inf
    Lifespan: Inf
    Liveliness: AUTOMATIC

Subscriber #1:
  Node name: object_detector
  QoS profile:
    Reliability: BEST_EFFORT
    Durability: VOLATILE
    History (depth): KEEP_LAST (1)
```

**Analysis**:
- **Reliability**: BEST_EFFORT (no retransmits) - appropriate for 30 FPS image stream
- **Durability**: VOLATILE (discard old frames) - correct since only latest frame matters
- **History depth**: 1 (keep only latest) - minimizes memory usage
- **Verdict**: QoS is correctly configured for high-frequency sensor data

#### Grading Rubric (Total: 10 points)
- **Commands** (2 pts): Correct `ros2 topic` commands
- **QoS interpretation** (5 pts): Correct identification of reliability, durability, history
- **Justification** (3 pts): Explains why this QoS is appropriate for images

#### Common Mistakes
- Not using `-v` flag (verbose mode) - doesn't show QoS details
- Misinterpreting BEST_EFFORT as "low quality" (it's a latency optimization)
- Thinking VOLATILE means data is lost (it just means no history for late joiners)

---

### Exercise 2.2: Service Implementation 📗

**Difficulty**: Application | **Time**: 45 min

*(Due to length constraints, providing abbreviated solution)*

#### Solution Overview
Implement a service server that resets joint position offsets and a client that calls it.

#### Key Code Snippets

**Server**:
```python
from std_srvs.srv import Trigger

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.srv = self.create_service(Trigger, '/humanoid/reset_joints', self.reset_callback)

    def reset_callback(self, request, response):
        # Simulate reset operation
        self.joint_offsets = [0.0] * 20  # Reset all 20 joints
        response.success = True
        response.message = 'All joint offsets reset to zero'
        return response
```

**Client**:
```python
client = self.create_client(Trigger, '/humanoid/reset_joints')
client.wait_for_service()
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(self, future)
if future.result().success:
    print(future.result().message)
```

#### Grading Rubric (10 pts)
- Server implementation (4 pts)
- Client implementation (4 pts)
- Error handling (2 pts)

---

## Chapter 3: Actions and Parameters

### Exercise 3.1: Action State Diagram 📘

**Difficulty**: Recall | **Time**: 15 min

#### Complete Solution

**Action states** (from client perspective):
1. **UNKNOWN**: Initial state before sending goal
2. **ACCEPTED**: Server accepted goal, starting execution
3. **EXECUTING**: Goal in progress, receiving feedback
4. **CANCELING**: Cancel requested, server processing cancellation
5. **SUCCEEDED**: Goal completed successfully
6. **ABORTED**: Server terminated goal due to error
7. **CANCELED**: Goal canceled per client request

**State transitions**:
```
UNKNOWN → (send_goal) → ACCEPTED → EXECUTING → SUCCEEDED
                              ↓         ↓         ↓
                              ↓    CANCELING → CANCELED
                              ↓         ↓
                              └──→ ABORTED
```

#### Grading Rubric (10 pts)
- All 7 states listed (7 pts)
- Correct transitions (3 pts)

---

### Exercise 3.2: Pick-and-Place Action Server 📗

**Difficulty**: Application | **Time**: 60 min

*(Abbreviated due to length)*

#### Key Components

**Action definition** (custom):
```
# Goal
string object_id
geometry_msgs/Pose target_pose
---
# Result
bool success
string message
---
# Feedback
string current_phase  # "approaching", "grasping", "lifting", "moving", "placing"
float32 progress_percentage
```

**Server logic**:
```python
def execute_callback(self, goal_handle):
    phases = ['approaching', 'grasping', 'lifting', 'moving', 'placing']
    for i, phase in enumerate(phases):
        # Check cancellation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return PickAndPlace.Result(success=False, message='Canceled')

        # Publish feedback
        feedback = PickAndPlace.Feedback()
        feedback.current_phase = phase
        feedback.progress_percentage = (i + 1) / len(phases) * 100.0
        goal_handle.publish_feedback(feedback)

        # Simulate phase duration
        time.sleep(2.0)

    goal_handle.succeed()
    return PickAndPlace.Result(success=True, message='Object placed successfully')
```

#### Grading Rubric (10 pts)
- Action definition (2 pts)
- Feedback publishing (3 pts)
- Cancellation handling (2 pts)
- Result return (2 pts)
- Code quality (1 pt)

---

## Chapter 4: URDF and Robot Models

### Exercise 4.1: URDF Structure Analysis 📘

**Difficulty**: Recall | **Time**: 20 min

#### Complete Solution

**1. Links** (rigid bodies):
- `<link>`: Defines a rigid body component
- Contains: visual geometry (rendering), collision geometry (physics), inertial properties (mass, inertia tensor)
- Example: `shoulder_link`, `elbow_link`, `wrist_link`

**2. Joints** (connections):
- `<joint>`: Connects two links (parent → child)
- Types: `revolute` (1 DOF rotation), `prismatic` (1 DOF translation), `fixed` (0 DOF), `continuous` (unlimited rotation)
- Contains: axis, limits (position, velocity, effort)

**3. Relationship**:
```
base_link (parent) ←[shoulder_joint]→ upper_arm_link (child)
                         ↑
                    (revolute, 1 DOF)
```

**Kinematic tree example** (humanoid arm):
```
base_link
  └─ shoulder_pitch_joint (revolute)
      └─ upper_arm_link
          └─ elbow_joint (revolute)
              └─ forearm_link
                  └─ wrist_joint (revolute)
                      └─ hand_link
```

#### Grading Rubric (10 pts)
- Link definition (3 pts)
- Joint types and properties (4 pts)
- Kinematic tree structure (3 pts)

---

### Exercise 4.2: Humanoid Arm URDF 📗

**Difficulty**: Application | **Time**: 60 min

#### Complete Solution

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder joint (pitch) -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_pitch" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.356" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Wrist joint -->
  <joint name="wrist_roll" type="revolute">
    <parent link="forearm_link"/>
    <child link="hand_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- Hand link -->
  <link name="hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.05 0.02"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.05 0.02"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

#### Validation Commands
```bash
check_urdf humanoid_arm.urdf
urdf_to_graphiz humanoid_arm.urdf
ros2 launch urdf_tutorial display.launch.py model:=humanoid_arm.urdf
```

#### Grading Rubric (10 pts)
- 4 links defined (2 pts)
- 3 joints with correct types (3 pts)
- Joint limits specified (2 pts)
- Inertial properties (2 pts)
- Validates with `check_urdf` (1 pt)

---

## Chapter 5: Bridging AI Agents to ROS

### Exercise 5.1: LLM Prompt Engineering 📗

**Difficulty**: Application | **Time**: 30 min

#### Complete Solution

**Optimized Prompt**:
```python
SYSTEM_PROMPT = """You are a robotic task planner for a humanoid robot. Your job is to translate natural language commands into a structured JSON action sequence.

Available actions:
- navigate: Move to a location (args: x, y in meters)
- grasp: Pick up an object (args: object_id)
- place: Put down held object (args: x, y, z in meters)
- say: Speak a message (args: text)

Robot capabilities:
- Max reach: 0.8 meters
- Grasp force: 10-50 Newtons
- Walking speed: 0.5 m/s

Output format (JSON):
{
  "actions": [
    {"type": "navigate", "args": {"x": 1.0, "y": 2.0}},
    {"type": "grasp", "args": {"object_id": "red_block"}},
    {"type": "place", "args": {"x": 3.0, "y": 1.0, "z": 0.5}}
  ]
}

Safety constraints:
- Verify object is within reach before grasping
- Check for obstacles before navigation
- Confirm grasp success before placing
"""

USER_COMMAND = "Pick up the red block and put it on the table"

# Expected LLM output:
{
  "actions": [
    {"type": "navigate", "args": {"x": 1.0, "y": 1.0}},
    {"type": "grasp", "args": {"object_id": "red_block"}},
    {"type": "navigate", "args": {"x": 2.0, "y": 0.0}},
    {"type": "place", "args": {"x": 2.0, "y": 0.0, "z": 0.7}}
  ]
}
```

**Key elements**:
1. **Context**: Robot capabilities and constraints
2. **Action library**: Explicit list of available actions
3. **Output format**: Structured JSON (easy to parse)
4. **Safety**: Constraints embedded in prompt

#### Grading Rubric (10 pts)
- Action library defined (3 pts)
- Structured output format (3 pts)
- Safety constraints (2 pts)
- Realistic example (2 pts)

---

## Summary: Common Patterns Across Exercises

**High-scoring solutions**:
- Include error handling and edge cases
- Provide detailed comments and explanations
- Follow ROS 2 naming conventions (`snake_case` for topics, `CamelCase` for classes)
- Use appropriate message types from standard packages
- Validate URDF with `check_urdf` before submission

**Common mistakes to watch for**:
- Not sourcing ROS 2 setup (`source /opt/ros/humble/setup.bash`)
- Forgetting to call `rclpy.init()` / `rclpy.shutdown()`
- Mismatched QoS between publishers and subscribers
- Invalid URDF (missing `<inertial>` tags)
- Hardcoded values instead of parameters

**Grading philosophy**:
- Partial credit for incomplete but correct approaches
- Deduct points for missing safety checks (especially in LLM integration)
- Bonus points for creative extensions and optimizations

---

**End of Instructor Solutions**
