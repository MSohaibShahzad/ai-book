---
title: "Chapter 3: Actions and Parameters"
slug: actions-and-parameters
sidebar_label: "3. Actions & Parameters"
toc: true
description: "Advanced ROS 2 communication: actions for long-running tasks with feedback, and parameters for runtime configuration"
---

# Chapter 3: Actions and Parameters

## Introduction

In Chapter 2, you mastered **topics** (streaming data) and **services** (synchronous request-response). But what about tasks that take time and provide progress updates? What if you need to reconfigure a node without restarting it?

**This chapter introduces**:

1. **Actions**: Long-running tasks with feedback and cancellation (e.g., "navigate to goal", "grasp object")
2. **Parameters**: Runtime configuration (e.g., change PID gains, adjust sensor thresholds)

**The driving question**: How do you command a humanoid robot to "walk to position (5, 3)" and receive progress updates ("50% complete", "obstacle detected") while allowing cancellation mid-task?

**Answer**: **Actions** (not topics or services).

**Learning Objectives**:

By the end of this chapter, you will:

1. **Explain** when to use actions vs. topics vs. services
2. **Implement** an action server in Python with goal handling, feedback, and results
3. **Write** an action client that sends goals, monitors feedback, and handles cancellation
4. **Configure** node parameters dynamically using YAML files and command-line tools
5. **Design** lifecycle nodes for stateful systems (startup, shutdown, error recovery)

**Time**: 4 hours (reading + hands-on)

---

## 3.1 Actions: Long-Running Tasks

### 3.1.1 The Problem with Services for Long Tasks

**Scenario**: You command a humanoid robot to navigate to a goal 10 meters away (takes 30 seconds).

**Using a service** (bad idea):
```python
# Client blocks for 30 seconds waiting for response
response = nav_client.call(goal_position)
```

**Problems**:
- **No progress updates**: Is the robot stuck? 10% done? 90% done?
- **No cancellation**: What if you need to abort mid-task?
- **Timeout hell**: Did the service fail, or is it just slow?

### 3.1.2 Actions: The Solution

**Actions** extend services with three communication channels:

1. **Goal**: Client sends goal to server (like service request)
2. **Feedback**: Server periodically sends progress updates (like topic)
3. **Result**: Server sends final outcome when done (like service response)

**Bonus**: Built-in **cancellation** mechanism.

**Analogy**: Ordering food delivery:
- **Goal**: "Deliver pizza to 123 Main St"
- **Feedback**: "Order confirmed", "Pizza in oven", "Driver en route"
- **Result**: "Delivered" or "Cancelled by customer"

### 3.1.3 Action Message Structure

An action is defined with three parts:

**Example**: `NavigateToPose.action` (from Nav2 stack)

```
# Goal
geometry_msgs/PoseStamped pose
---
# Result
std_msgs/Empty result
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
duration estimated_time_remaining
```

**Breakdown**:
- **Goal**: Target position and orientation
- **Feedback**: Current position, distance left, ETA
- **Result**: Empty (just signals completion)

---

## 3.2 Writing an Action Server (Python)

### 3.2.1 Example: Fibonacci Action

Let's implement a simple action server that computes Fibonacci numbers (simulating a long-running task).

**Action definition** (use built-in `example_interfaces/action/Fibonacci`):

```
# Goal
int32 order  # Compute Fibonacci up to this index
---
# Result
int32[] sequence  # Final Fibonacci sequence
---
# Feedback
int32[] partial_sequence  # Sequence so far
```

**Server implementation**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci action server started')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal cancelled')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            # Simulate slow computation (1 second per iteration)
            time.sleep(1)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts**:

1. **`ActionServer(node, action_type, action_name, callback)`**: Creates an action server
2. **`goal_handle.request`**: Access goal parameters
3. **`goal_handle.publish_feedback(msg)`**: Send progress updates
4. **`goal_handle.is_cancel_requested`**: Check if client cancelled
5. **`goal_handle.succeed()`**: Mark goal as completed successfully
6. **`goal_handle.canceled()`**: Mark goal as cancelled
7. **Return `Result()`**: Send final result to client

---

## 3.3 Writing an Action Client (Python)

### 3.3.1 Client Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Wait for server to be available
        self._action_client.wait_for_server()

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    node.send_goal(10)  # Compute Fibonacci(10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 3.3.2 Running the Example

**Terminal 1** (server):
```bash
python3 fibonacci_action_server.py
```

**Terminal 2** (client):
```bash
python3 fibonacci_action_client.py
```

**Expected output** (client):
```
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2, 3]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

### 3.3.3 Cancelling Actions

To cancel mid-execution:

```python
# In client code, after sending goal:
goal_handle = self._action_client.send_goal_async(goal_msg)
# ... later ...
goal_handle.cancel_goal_async()
```

**In server**: Check `goal_handle.is_cancel_requested` in your loop.

---

## 3.4 When to Use Actions vs. Topics vs. Services

| **Use Case** | **Communication Pattern** | **Example** |
|--------------|---------------------------|-------------|
| Continuous sensor data | **Topic** (pub-sub) | `/camera/image` (30 FPS) |
| One-off request, instant response | **Service** (request-response) | `/reset_odometry` |
| Long-running task with feedback | **Action** (goal-feedback-result) | `/navigate_to_goal` (30 sec, progress updates) |
| Configuration | **Parameter** (get/set) | Adjust PID gains |

**Decision flowchart**:

```
Is it a continuous stream? → YES → Topic
                          ↓ NO
Does it take >1 second? → YES → Action
                       ↓ NO
Does it need a response? → YES → Service
                         ↓ NO
                         → Topic (fire-and-forget)
```

---

## 3.5 Parameters: Runtime Configuration

### 3.5.1 The Problem with Hardcoded Values

**Bad code**:
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.max_speed = 2.0  # Hardcoded!
        self.pid_p_gain = 1.5  # Hardcoded!
```

**Problems**:
- Must recompile/restart node to change values
- Different robots need different configs (requires code duplication)
- Can't tune in real-time

### 3.5.2 Using Parameters (Python)

**Declaring parameters**:
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameters with default values
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('pid_p_gain', 1.5)

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.pid_p_gain = self.get_parameter('pid_p_gain').value

        self.get_logger().info(f'max_speed: {self.max_speed}, pid_p_gain: {self.pid_p_gain}')
```

**Setting parameters at launch**:

**Option 1: Command line**
```bash
ros2 run my_package my_node --ros-args -p max_speed:=3.0 -p pid_p_gain:=2.0
```

**Option 2: YAML file** (`config/params.yaml`)
```yaml
my_node:
  ros__parameters:
    max_speed: 3.0
    pid_p_gain: 2.0
```

Then load:
```bash
ros2 run my_package my_node --ros-args --params-file config/params.yaml
```

### 3.5.3 Dynamic Parameter Updates

**Callback for parameter changes**:

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameter with constraints
        descriptor = ParameterDescriptor(
            description='Maximum speed in m/s',
            floating_point_range=[{'from_value': 0.0, 'to_value': 5.0, 'step': 0.1}]
        )
        self.declare_parameter('max_speed', 2.0, descriptor)

        # Register callback for parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                if 0.0 <= param.value <= 5.0:
                    self.max_speed = param.value
                    self.get_logger().info(f'Updated max_speed to {param.value}')
                    return SetParametersResult(successful=True)
                else:
                    return SetParametersResult(successful=False, reason='Out of range')
        return SetParametersResult(successful=True)
```

**Update parameter at runtime**:
```bash
ros2 param set /my_node max_speed 3.5
```

**List all parameters**:
```bash
ros2 param list
```

**Dump parameters to file**:
```bash
ros2 param dump /my_node > my_params.yaml
```

---

## 3.6 Lifecycle Nodes

### 3.6.1 Why Lifecycle Nodes?

**Problem**: Complex systems need coordinated startup/shutdown:
- Initialize sensors before starting controllers
- Gracefully stop motors before shutting down
- Recover from errors without restarting

**Solution**: **Lifecycle nodes** (managed nodes with state machines).

### 3.6.2 Lifecycle States

```
Unconfigured → Configure → Inactive → Activate → Active
                    ↓          ↓           ↓
                Cleanup    Deactivate   Shutdown
```

**States**:
- **Unconfigured**: Node exists but not ready
- **Inactive**: Configured (resources allocated) but not processing data
- **Active**: Fully operational
- **Finalized**: Shut down cleanly

### 3.6.3 Example: Lifecycle Node (Python)

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        # Allocate resources (e.g., initialize hardware)
        self.create_publisher(String, '/topic', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Start processing (e.g., enable motors)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Stop processing (e.g., disable motors)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        # Release resources
        self.destroy_publisher(self.publisher_)
        return TransitionCallbackReturn.SUCCESS
```

**Managing lifecycle** (command-line):
```bash
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
ros2 lifecycle get /my_node  # Check current state
```

---

## 3.7 Real-World Example: Humanoid Navigation

**Architecture**:

```
[LLM Planner] → sends action goal → /humanoid/navigate_to_pose (Action Server in Nav2)
                                           ↓
                                    [Nav2 Stack]
                                    - Publishes feedback: current pose, distance remaining
                                    - Subscribes to /scan (lidar), /odom
                                    - Configured via parameters (max_speed, inflation_radius)
                                           ↓
                                    [Motor Controllers] (lifecycle nodes)
                                    - Inactive until Nav2 activates them
                                    - Emergency stop → deactivate transition
```

**Why this design?**
- **Action**: Navigation takes time (30+ seconds), needs progress updates, cancellable
- **Parameters**: Tune max speed, obstacle inflation without restarting
- **Lifecycle**: Ensure motors are initialized before navigation starts

---

## 3.8 Debugging Actions and Parameters

### 3.8.1 Command-Line Tools for Actions

**List actions**:
```bash
ros2 action list
```

**Send goal from command line**:
```bash
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

**Check action server status**:
```bash
ros2 action info /fibonacci
```

### 3.8.2 Command-Line Tools for Parameters

**List parameters**:
```bash
ros2 param list
```

**Get parameter value**:
```bash
ros2 param get /my_node max_speed
```

**Set parameter**:
```bash
ros2 param set /my_node max_speed 3.0
```

**Describe parameter** (constraints, type):
```bash
ros2 param describe /my_node max_speed
```

---

## 3.9 Summary

**What you learned**:

✅ **Actions** are for long-running tasks with feedback and cancellation
✅ **Action servers** execute goals, publish feedback, return results
✅ **Action clients** send goals, monitor progress, handle cancellation
✅ **Parameters** enable runtime configuration without restarts
✅ **Lifecycle nodes** provide coordinated startup/shutdown for complex systems

**What's next**:

In **Chapter 4**, we'll model robots using **URDF** (Unified Robot Description Format) and visualize them in RViz.

---

## 3.10 Self-Check Questions

1. **When would you use an action instead of a service?** (Answer: Long-running tasks with feedback/cancellation)
2. **What are the three communication channels in an action?** (Answer: Goal, Feedback, Result)
3. **How do you dynamically update a parameter?** (Answer: `ros2 param set` or `add_on_set_parameters_callback`)
4. **What are the lifecycle node states?** (Answer: Unconfigured, Inactive, Active, Finalized)

---

## 3.11 Exercises

See **[Exercises](./exercises.md)** for hands-on challenges:
- **Recall**: Explain action vs. service, list lifecycle states
- **Application**: Implement a "pick and place" action with feedback
- **Synthesis**: Design a multi-node system with coordinated lifecycle management

---

**Next chapter**: [Chapter 4: URDF and Robot Models →](./04-urdf-and-robot-models.md)
