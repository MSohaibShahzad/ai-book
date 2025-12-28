---
title: "Chapter 2: Nodes, Topics, and Services"
slug: nodes-topics-services
sidebar_label: "2. Nodes, Topics, Services"
toc: true
description: "Deep dive into ROS 2 communication patterns: publish-subscribe (topics) and request-response (services) for humanoid robotics"
---

# Chapter 2: Nodes, Topics, and Services

## Introduction

In Chapter 1, you learned *what* ROS 2 is and *why* it exists. Now we'll dive into *how* to use it by building custom nodes that communicate via **topics** (asynchronous streaming) and **services** (synchronous request-response).

**The driving question**: How do you architect a system where a camera node, an object detector node, and a gripper controller node work together—without tightly coupling their code?

**Answer**: **Topics** for streaming data (sensor readings, detections) and **services** for one-off requests (reset state, enable motors).

**Learning Objectives**:

By the end of this chapter, you will:

1. **Write** a ROS 2 publisher node in Python that publishes sensor data
2. **Write** a ROS 2 subscriber node that processes incoming messages
3. **Configure** Quality of Service (QoS) for reliable vs. best-effort delivery
4. **Implement** a service server and client for request-response operations
5. **Debug** communication issues using ROS 2 introspection tools

**Time**: 4 hours (reading + hands-on)

---

## 2.1 Topics: Asynchronous Publish-Subscribe

### 2.1.1 The Publish-Subscribe Pattern

**Problem**: A camera produces images at 30 FPS. Multiple systems need those images:
- Object detector (needs every frame)
- Data logger (records for replay)
- Visualization tool (displays for debugging)

**Naive approach (point-to-point)**:
```
[Camera] → sends to → [Detector]
         → sends to → [Logger]
         → sends to → [Visualizer]
```
**Issues**:
- Camera must know about all consumers (tight coupling)
- Adding a new consumer requires changing camera code
- If logger crashes, does camera crash too?

**Publish-subscribe pattern**:
```
[Camera] → publishes to → /camera/image → [Detector subscribes]
                                        → [Logger subscribes]
                                        → [Visualizer subscribes]
```
**Benefits**:
- **Decoupling**: Camera doesn't know who's listening
- **Scalability**: Add subscribers without touching publisher
- **Fault isolation**: Logger crash doesn't affect camera

### 2.1.2 Writing a Publisher (Python)

Let's create a node that publishes fake joint states (pretending to be a humanoid's sensor system).

**File**: `joint_state_publisher.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Timer: call timer_callback every 0.01 seconds (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Joint state publisher started')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate 6 joints (shoulder, elbow, wrist for 2 arms)
        msg.name = ['left_shoulder', 'left_elbow', 'left_wrist',
                   'right_shoulder', 'right_elbow', 'right_wrist']

        # Fake sine wave motion (for demo purposes)
        t = self.counter * 0.01
        msg.position = [math.sin(t), math.cos(t), math.sin(2*t),
                       -math.sin(t), -math.cos(t), -math.sin(2*t)]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)  # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:

1. **`create_publisher(msg_type, topic_name, qos_depth)`**: Creates a publisher object
   - `sensor_msgs.msg.JointState`: Message type (must match subscribers)
   - `'/joint_states'`: Topic name (by convention, starts with `/`)
   - `10`: Queue size (how many messages to buffer if network is slow)

2. **`create_timer(period, callback)`**: Calls `callback` every `period` seconds
   - Alternative: Use threads, but timers are preferred in ROS 2

3. **`get_clock().now()`**: ROS 2 time (can be real-time or simulated time in Gazebo)

4. **`publish(msg)`**: Sends message to all subscribers

### 2.1.3 Writing a Subscriber (Python)

Now let's create a node that subscribes to `/joint_states` and prints received data.

**File**: `joint_state_listener.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        # Create subscriber: topic name, message type, callback, queue size
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.get_logger().info('Joint state listener started')

    def listener_callback(self, msg):
        # Extract first joint position (left shoulder)
        if len(msg.position) > 0:
            self.get_logger().info(f'Left shoulder angle: {msg.position[0]:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:

1. **`create_subscription(msg_type, topic_name, callback, qos_depth)`**: Creates a subscriber
   - `listener_callback` is called every time a message arrives

2. **Callback pattern**: Asynchronous—your code doesn't poll; ROS 2 calls your callback when data arrives

3. **`rclpy.spin(node)`**: Blocks and processes callbacks (events, timers, subscriptions)

### 2.1.4 Running the Example

**Terminal 1** (publisher):
```bash
python3 joint_state_publisher.py
```

**Terminal 2** (subscriber):
```bash
python3 joint_state_listener.py
```

**Expected output** (subscriber):
```
[INFO] [joint_state_listener]: Left shoulder angle: 0.00 rad
[INFO] [joint_state_listener]: Left shoulder angle: 0.01 rad
[INFO] [joint_state_listener]: Left shoulder angle: 0.02 rad
```

---

## 2.2 Quality of Service (QoS)

### 2.2.1 Why QoS Matters

**Scenario 1**: Publishing `/robot_description` (URDF)—this is sent once at startup and must be reliable.

**Scenario 2**: Publishing `/camera/image` at 30 FPS—if frame #47 is lost, frame #48 is coming in 33 ms. Retransmitting is wasteful.

**Solution**: Configurable QoS policies.

### 2.2.2 Key QoS Settings

| **Policy** | **Options** | **Trade-off** |
|------------|-------------|---------------|
| **Reliability** | Reliable, Best Effort | Guaranteed delivery vs. low latency |
| **Durability** | Volatile, Transient Local | Discard old messages vs. keep last N |
| **History** | Keep Last N, Keep All | Memory usage vs. completeness |
| **Deadline** | Duration | Detect if publisher is too slow |

**Example: Camera images** (high-frequency, lossy OK)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Don't retransmit
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only keep latest image
)

self.publisher_ = self.create_publisher(Image, '/camera/image', qos)
```

**Example: Robot configuration** (one-time, must be reliable)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Retransmit until ACK
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep for late joiners
    depth=1
)

self.publisher_ = self.create_publisher(String, '/robot_description', qos)
```

### 2.2.3 QoS Compatibility

**Critical rule**: Publisher and subscriber QoS must be *compatible*, not necessarily identical.

**Compatible combinations**:
- Reliable publisher → Reliable subscriber ✅
- Reliable publisher → Best Effort subscriber ✅
- Best Effort publisher → Best Effort subscriber ✅
- Best Effort publisher → Reliable subscriber ❌ (incompatible)

**Debugging**: Use `ros2 topic info /topic_name -v` to see QoS settings.

---

## 2.3 Services: Request-Response Pattern

### 2.3.1 When to Use Services vs. Topics

| **Use Topics** | **Use Services** |
|----------------|------------------|
| Continuous data streams (sensor readings) | One-off requests (reset state) |
| Many-to-many (multiple subscribers) | One-to-one (single responder) |
| Don't care if anyone is listening | Need confirmation of completion |
| Example: `/camera/image` | Example: `/robot/enable_motors` |

### 2.3.2 Writing a Service Server (Python)

**Scenario**: A service that computes forward kinematics (given joint angles, return end-effector position).

**Step 1: Define service** (`.srv` file)

ROS 2 services use request-response message types. For this example, we'll use a built-in service `example_interfaces/srv/AddTwoInts` (adds two integers).

**File structure**:
```
AddTwoInts.srv:
  int64 a
  int64 b
  ---
  int64 sum
```

(The `---` separates request from response)

**Step 2: Implement server**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Service server ready: add_two_ints')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3.3 Writing a Service Client (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b

        # Asynchronous call (returns Future)
        future = self.cli.call_async(req)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()

    # Send request
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Result: {response.sum}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3.4 Running the Example

**Terminal 1** (server):
```bash
python3 add_two_ints_server.py
```

**Terminal 2** (client):
```bash
python3 add_two_ints_client.py 5 7
# Output: Result: 12
```

**Key concepts**:

1. **`create_service(srv_type, name, callback)`**: Creates a service server
2. **`create_client(srv_type, name)`**: Creates a service client
3. **`call_async(request)`**: Non-blocking service call (returns `Future`)
4. **`spin_until_future_complete()`**: Blocks until response arrives

---

## 2.4 Debugging Communication

### 2.4.1 Command-Line Tools

**List all topics**:
```bash
ros2 topic list
```

**Show topic info** (publishers, subscribers, QoS):
```bash
ros2 topic info /joint_states -v
```

**Echo live messages**:
```bash
ros2 topic echo /joint_states
```

**Publish manually** (useful for testing subscribers):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

**Check service availability**:
```bash
ros2 service list
ros2 service type /add_two_ints  # Shows: example_interfaces/srv/AddTwoInts
```

**Call service from command line**:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
# Output: sum: 7
```

### 2.4.2 Common Issues

**Problem**: Subscriber doesn't receive messages, but `ros2 topic list` shows the topic.

**Solution**: QoS mismatch. Check with:
```bash
ros2 topic info /topic_name -v
```
Ensure publisher/subscriber QoS are compatible.

**Problem**: Service client hangs on `wait_for_service()`.

**Solution**: Server not running. Check:
```bash
ros2 service list  # Is your service listed?
ros2 node list     # Is the server node running?
```

---

## 2.5 Real-World Example: Humanoid Arm Control

Let's tie this together with a realistic scenario: controlling a humanoid arm.

**Architecture**:
```
[Motion Planner]  → publishes → /arm/joint_trajectory → [Motor Controller subscribes]
[Motor Controller] → publishes → /arm/joint_states → [Motion Planner subscribes] (for feedback)
[Safety Monitor] → calls service → /arm/emergency_stop
```

**Pseudocode** (Motion Planner):
```python
# Publisher: send desired joint positions
self.traj_pub = self.create_publisher(JointTrajectory, '/arm/joint_trajectory', 10)

# Subscriber: receive current joint states
self.state_sub = self.create_subscription(JointState, '/arm/joint_states', self.state_callback, 10)

# Service client: emergency stop if needed
self.estop_cli = self.create_client(Trigger, '/arm/emergency_stop')
```

**Pseudocode** (Motor Controller):
```python
# Subscriber: receive trajectory commands
self.traj_sub = self.create_subscription(JointTrajectory, '/arm/joint_trajectory', self.execute_trajectory, 10)

# Publisher: send current joint states
self.state_pub = self.create_publisher(JointState, '/arm/joint_states', 10)

# Service server: handle emergency stops
self.estop_srv = self.create_service(Trigger, '/arm/emergency_stop', self.emergency_stop_callback)
```

**Why this design?**
- **Decoupling**: Motion planner doesn't know how motors work (could be hardware or simulated)
- **Feedback loop**: Planner subscribes to joint states to adjust trajectory
- **Safety**: Emergency stop service is synchronous (waits for confirmation)

---

## 2.6 Summary

**What you learned**:

✅ **Topics** enable decoupled, asynchronous communication (pub-sub pattern)
✅ **Publishers** send messages; **subscribers** receive messages
✅ **QoS policies** tune reliability, latency, and memory usage
✅ **Services** provide synchronous request-response (one-to-one)
✅ **Debugging tools** (`ros2 topic`, `ros2 service`) help diagnose communication issues

**What's next**:

In **Chapter 3**, we'll explore:
- **Actions**: Long-running tasks with feedback (e.g., "navigate to goal")
- **Parameters**: Runtime reconfiguration without restarting nodes

---

## 2.7 Self-Check Questions

1. **When would you use a topic vs. a service?** (Answer: Topic for streaming data, service for one-off requests)
2. **What happens if a subscriber's QoS is RELIABLE but the publisher's is BEST_EFFORT?** (Answer: Incompatible—no messages received)
3. **How do you check if a service is available?** (Answer: `ros2 service list` and `wait_for_service()` in code)
4. **What is the purpose of `rclpy.spin()`?** (Answer: Keeps the node alive and processes callbacks)

---

## 2.8 Exercises

See **[Exercises](./exercises.md)** for hands-on challenges:
- **Recall**: Define pub-sub pattern, explain QoS policies
- **Application**: Implement a temperature sensor publisher and alert subscriber
- **Synthesis**: Design a multi-node humanoid perception pipeline

---

**Next chapter**: [Chapter 3: Actions and Parameters →](./03-actions-and-parameters.md)
