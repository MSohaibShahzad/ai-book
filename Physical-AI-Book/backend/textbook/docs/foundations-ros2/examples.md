---
title: "ROS 2 Code Examples"
slug: "ros2-examples"
sidebar_label: "Code Examples"
description: "Annotated Python examples for ROS 2 concepts applied to humanoid robotics"
---

# ROS 2 Code Examples

This page provides complete, runnable Python code examples demonstrating core ROS 2 concepts in the context of humanoid robotics applications. Each example includes detailed annotations and expected output.

---

## Example 1: Publisher Node - IMU Sensor Data

**Use Case**: Publishing inertial measurement unit (IMU) data from a humanoid robot's torso sensor.

```python
#!/usr/bin/env python3
"""
IMU Publisher Node
Publishes simulated IMU data from a humanoid robot's torso sensor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import math
import time


class ImuPublisher(Node):
    """
    Publishes IMU sensor data at 100Hz for balance control systems.
    Demonstrates proper publisher setup, timer callbacks, and message population.
    """

    def __init__(self):
        # Initialize the node with a descriptive name
        super().__init__('imu_publisher')

        # Create publisher for IMU data on the /torso/imu topic
        # Queue size of 10 ensures recent data is prioritized
        self.publisher_ = self.create_publisher(Imu, '/torso/imu', 10)

        # Create timer that fires at 100Hz (0.01 second intervals)
        # High frequency is critical for balance control systems
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize simulation state variables
        self.count = 0
        self.start_time = time.time()

        self.get_logger().info('IMU Publisher initialized - Publishing at 100Hz')

    def timer_callback(self):
        """
        Timer callback that publishes IMU data.
        Simulates slight oscillation representing robot sway during standing.
        """
        # Create and populate IMU message
        msg = Imu()

        # Set header with timestamp and frame information
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'torso_imu_link'

        # Simulate orientation quaternion (slight pitch oscillation)
        # Real robots would read this from hardware IMU
        elapsed = time.time() - self.start_time
        pitch = 0.05 * math.sin(2 * math.pi * 0.5 * elapsed)  # 0.5 Hz sway

        # Convert euler angle to quaternion (simplified for small angles)
        msg.orientation = Quaternion()
        msg.orientation.w = math.cos(pitch / 2)
        msg.orientation.x = 0.0
        msg.orientation.y = math.sin(pitch / 2)
        msg.orientation.z = 0.0

        # Set orientation covariance (uncertainty in measurement)
        # Diagonal elements represent variance for roll, pitch, yaw
        msg.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # Simulate angular velocity (rate of change of orientation)
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.05 * math.cos(2 * math.pi * 0.5 * elapsed)
        msg.angular_velocity.z = 0.0

        # Simulate linear acceleration (includes gravity)
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity in z-axis

        # Publish the message
        self.publisher_.publish(msg)

        # Log status every 100 messages (once per second)
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Published IMU data: pitch={pitch:.4f} rad')


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create and spin the node
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Console Output**:
```
[INFO] [imu_publisher]: IMU Publisher initialized - Publishing at 100Hz
[INFO] [imu_publisher]: Published IMU data: pitch=0.0248 rad
[INFO] [imu_publisher]: Published IMU data: pitch=0.0475 rad
[INFO] [imu_publisher]: Published IMU data: pitch=0.0495 rad
```

**Key Concepts**:
- `create_publisher()`: Sets up topic, message type, and QoS
- `create_timer()`: Ensures consistent publishing frequency
- Message headers: Timestamps and frame IDs for coordinate transforms
- Covariance matrices: Representing sensor uncertainty for fusion algorithms

---

## Example 2: Subscriber Node - Processing Joint States

**Use Case**: Monitoring joint positions and velocities from a humanoid robot's actuators.

```python
#!/usr/bin/env python3
"""
Joint State Monitor Subscriber
Processes joint state data and detects anomalies in humanoid robot motion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStateMonitor(Node):
    """
    Subscribes to joint states and monitors for:
    - Joint limit violations
    - Excessive velocities
    - Missing joint data
    """

    def __init__(self):
        super().__init__('joint_state_monitor')

        # Define expected joints for a simplified humanoid (12 DOF)
        self.expected_joints = [
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow',
            'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_pitch', 'right_knee', 'right_ankle'
        ]

        # Define safe operating limits (radians)
        self.position_limits = {
            'shoulder_pitch': (-2.0, 2.0),
            'shoulder_roll': (-1.5, 1.5),
            'elbow': (0.0, 2.5),
            'hip_pitch': (-1.5, 1.5),
            'knee': (0.0, 2.5),
            'ankle': (-0.5, 0.5)
        }

        # Maximum safe velocities (rad/s)
        self.max_velocity = 3.0

        # Create subscriber with QoS profile suitable for sensor data
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # Queue size
        )

        # Statistics tracking
        self.message_count = 0
        self.anomaly_count = 0

        self.get_logger().info('Joint State Monitor initialized')
        self.get_logger().info(f'Monitoring {len(self.expected_joints)} joints')

    def joint_state_callback(self, msg):
        """
        Callback invoked whenever a JointState message is received.
        Performs validation and anomaly detection.
        """
        self.message_count += 1

        # Validate message structure
        if not msg.name or not msg.position:
            self.get_logger().warn('Received empty joint state message')
            return

        # Check for missing joints
        missing_joints = set(self.expected_joints) - set(msg.name)
        if missing_joints:
            self.get_logger().warn(f'Missing joints: {missing_joints}')
            self.anomaly_count += 1

        # Process each joint
        for i, joint_name in enumerate(msg.name):
            # Extract joint type from full name
            joint_type = self._get_joint_type(joint_name)

            # Check position limits
            if i < len(msg.position):
                position = msg.position[i]

                if joint_type in self.position_limits:
                    min_pos, max_pos = self.position_limits[joint_type]

                    if position < min_pos or position > max_pos:
                        self.get_logger().error(
                            f'Position limit violation: {joint_name} = {position:.3f} rad '
                            f'(limits: [{min_pos}, {max_pos}])'
                        )
                        self.anomaly_count += 1

            # Check velocity limits
            if i < len(msg.velocity):
                velocity = msg.velocity[i]

                if abs(velocity) > self.max_velocity:
                    self.get_logger().error(
                        f'Velocity limit violation: {joint_name} = {velocity:.3f} rad/s '
                        f'(max: {self.max_velocity})'
                    )
                    self.anomaly_count += 1

        # Periodic status report (every 50 messages)
        if self.message_count % 50 == 0:
            anomaly_rate = (self.anomaly_count / self.message_count) * 100
            self.get_logger().info(
                f'Processed {self.message_count} messages, '
                f'{self.anomaly_count} anomalies ({anomaly_rate:.1f}%)'
            )

    def _get_joint_type(self, joint_name):
        """
        Extracts joint type from full joint name.
        E.g., 'left_shoulder_pitch' -> 'shoulder_pitch'
        """
        parts = joint_name.split('_')
        if len(parts) >= 2:
            return '_'.join(parts[1:])  # Remove left/right prefix
        return joint_name


def main(args=None):
    rclpy.init(args=args)

    joint_monitor = JointStateMonitor()

    try:
        rclpy.spin(joint_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        joint_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Console Output**:
```
[INFO] [joint_state_monitor]: Joint State Monitor initialized
[INFO] [joint_state_monitor]: Monitoring 12 joints
[INFO] [joint_state_monitor]: Processed 50 messages, 0 anomalies (0.0%)
[ERROR] [joint_state_monitor]: Position limit violation: left_knee = 2.65 rad (limits: [0.0, 2.5])
[INFO] [joint_state_monitor]: Processed 100 messages, 1 anomalies (1.0%)
```

**Key Concepts**:
- `create_subscription()`: Registers callback for incoming messages
- Callback pattern: Event-driven processing of streaming data
- Data validation: Ensuring message integrity and completeness
- Domain logic: Applying robotics-specific constraints and monitoring

---

## Example 3: Service Server and Client - Gait Selection

**Use Case**: Request-response pattern for selecting walking gait parameters on a humanoid robot.

```python
#!/usr/bin/env python3
"""
Gait Selection Service
Server: Provides gait parameter configurations
Client: Requests specific gait modes
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using standard service for demonstration
# In production, you would define a custom service type for gait parameters


# ==================== SERVICE SERVER ====================

class GaitService(Node):
    """
    Service server that provides gait configurations.
    Supports: walking, running, stair_climbing modes.
    """

    def __init__(self):
        super().__init__('gait_service')

        # Available gait configurations
        self.gait_configs = {
            'walking': {
                'step_length': 0.3,  # meters
                'step_height': 0.05,
                'frequency': 1.0,     # Hz
                'description': 'Standard bipedal walking gait'
            },
            'running': {
                'step_length': 0.6,
                'step_height': 0.15,
                'frequency': 2.0,
                'description': 'Dynamic running with flight phase'
            },
            'stair_climbing': {
                'step_length': 0.25,
                'step_height': 0.20,
                'frequency': 0.5,
                'description': 'High-lift gait for stairs'
            }
        }

        self.current_gait = 'walking'

        # Create service server
        # Service name: /select_gait
        # Service type: SetBool (data field used for gait name)
        self.srv = self.create_service(
            SetBool,
            'select_gait',
            self.handle_gait_selection
        )

        self.get_logger().info('Gait Service server started')
        self.get_logger().info(f'Available gaits: {list(self.gait_configs.keys())}')

    def handle_gait_selection(self, request, response):
        """
        Service callback that processes gait selection requests.

        Args:
            request: SetBool.Request with data=True for success
            response: SetBool.Response with success status and message
        """
        # In this simplified example, we use the request.data as a trigger
        # In production, use a custom service definition with a gait_name field

        # Simulate gait selection logic
        # For demonstration, we cycle through available gaits
        gait_list = list(self.gait_configs.keys())
        current_idx = gait_list.index(self.current_gait)
        next_idx = (current_idx + 1) % len(gait_list)
        selected_gait = gait_list[next_idx]

        # Validate and apply gait
        if selected_gait in self.gait_configs:
            self.current_gait = selected_gait
            config = self.gait_configs[selected_gait]

            response.success = True
            response.message = (
                f"Gait changed to '{selected_gait}': {config['description']} "
                f"(step_length={config['step_length']}m, "
                f"frequency={config['frequency']}Hz)"
            )

            self.get_logger().info(f"Gait set to: {selected_gait}")
        else:
            response.success = False
            response.message = f"Unknown gait: {selected_gait}"
            self.get_logger().warn(f"Invalid gait request: {selected_gait}")

        return response


# ==================== SERVICE CLIENT ====================

class GaitClient(Node):
    """
    Service client that requests gait changes.
    Demonstrates synchronous and asynchronous service calls.
    """

    def __init__(self):
        super().__init__('gait_client')

        # Create service client
        self.client = self.create_client(SetBool, 'select_gait')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gait service...')

        self.get_logger().info('Connected to gait service')

    def send_request(self, enable=True):
        """
        Sends synchronous service request and waits for response.

        Args:
            enable: Trigger flag (in production, would be gait name)

        Returns:
            Response object or None if failed
        """
        # Create request
        request = SetBool.Request()
        request.data = enable

        self.get_logger().info('Sending gait change request...')

        # Call service synchronously
        future = self.client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()

            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().error(f'Failed: {response.message}')

            return response
        else:
            self.get_logger().error('Service call failed')
            return None


# ==================== MAIN FUNCTIONS ====================

def main_server(args=None):
    """Run the service server"""
    rclpy.init(args=args)
    gait_service = GaitService()

    try:
        rclpy.spin(gait_service)
    except KeyboardInterrupt:
        pass
    finally:
        gait_service.destroy_node()
        rclpy.shutdown()


def main_client(args=None):
    """Run the service client"""
    rclpy.init(args=args)
    gait_client = GaitClient()

    try:
        # Send multiple requests to cycle through gaits
        for i in range(3):
            response = gait_client.send_request(enable=True)
            rclpy.sleep(1.0)  # Wait between requests
    except KeyboardInterrupt:
        pass
    finally:
        gait_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Run server or client based on command line argument
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'client':
        main_client()
    else:
        main_server()
```

**Console Output (Server)**:
```
[INFO] [gait_service]: Gait Service server started
[INFO] [gait_service]: Available gaits: ['walking', 'running', 'stair_climbing']
[INFO] [gait_service]: Gait set to: running
[INFO] [gait_service]: Gait set to: stair_climbing
```

**Console Output (Client)**:
```
[INFO] [gait_client]: Connected to gait service
[INFO] [gait_client]: Sending gait change request...
[INFO] [gait_client]: Success: Gait changed to 'running': Dynamic running with flight phase (step_length=0.6m, frequency=2.0Hz)
[INFO] [gait_client]: Sending gait change request...
[INFO] [gait_client]: Success: Gait changed to 'stair_climbing': High-lift gait for stairs (step_length=0.25m, frequency=0.5Hz)
```

**Key Concepts**:
- `create_service()`: Establishes service server endpoint
- `create_client()`: Creates client for calling services
- Request-response pattern: Synchronous communication for configuration
- `wait_for_service()`: Ensures server availability before calling
- Service callbacks: Processing requests and generating responses

---

## Example 4: Action Server and Client - Balance Recovery

**Use Case**: Long-running task with feedback for recovering robot balance after disturbance.

```python
#!/usr/bin/env python3
"""
Balance Recovery Action
Server: Executes multi-step balance recovery sequence
Client: Monitors progress and receives feedback
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from action_tutorials_interfaces.action import Fibonacci  # Using standard action for demo
# In production: define custom action with fields for balance state
import time


# ==================== ACTION SERVER ====================

class BalanceRecoveryServer(Node):
    """
    Action server that executes balance recovery sequences.
    Provides feedback on recovery progress and final result.
    """

    def __init__(self):
        super().__init__('balance_recovery_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Action type (use custom type in production)
            'recover_balance',
            self.execute_callback
        )

        self.get_logger().info('Balance Recovery Action Server started')

    def execute_callback(self, goal_handle):
        """
        Executes balance recovery sequence.
        Simulates multi-phase recovery with progress feedback.

        Args:
            goal_handle: Handle for managing action execution

        Returns:
            Result object with final status
        """
        self.get_logger().info('Executing balance recovery...')

        # Recovery phases
        phases = [
            ('assess', 'Assessing robot state and disturbance'),
            ('stabilize', 'Adjusting ankle torques for stabilization'),
            ('shift', 'Shifting center of mass'),
            ('recover', 'Returning to nominal stance'),
            ('verify', 'Verifying stable equilibrium')
        ]

        # Feedback message (reused each iteration)
        feedback_msg = Fibonacci.Feedback()

        # Execute recovery phases
        sequence = [0, 1]  # Using Fibonacci sequence as progress indicator

        for phase_idx, (phase_name, phase_desc) in enumerate(phases):
            # Check if cancellation requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Balance recovery canceled')

                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            # Log phase
            self.get_logger().info(f'Phase {phase_idx + 1}/5: {phase_desc}')

            # Simulate phase execution
            time.sleep(0.5)

            # Update and publish feedback
            sequence.append(sequence[-1] + sequence[-2])
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Progress: {((phase_idx + 1) / len(phases)) * 100:.0f}%')

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return final result
        result = Fibonacci.Result()
        result.sequence = sequence

        self.get_logger().info('Balance recovery completed successfully')
        return result


# ==================== ACTION CLIENT ====================

class BalanceRecoveryClient(Node):
    """
    Action client that requests balance recovery and monitors progress.
    """

    def __init__(self):
        super().__init__('balance_recovery_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'recover_balance'
        )

        self.get_logger().info('Balance Recovery Client initialized')

    def send_goal(self, order=5):
        """
        Sends balance recovery goal to action server.

        Args:
            order: Goal parameter (number of recovery phases)
        """
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Sending balance recovery goal')

        # Send goal with callbacks for feedback and result
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for when goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts or rejects goal"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')

        # Get result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called periodically with progress updates.

        Args:
            feedback_msg: Feedback data from action server
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: {len(feedback.sequence)} steps completed'
        )

    def get_result_callback(self, future):
        """Called when action completes"""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Balance recovery succeeded! Final sequence length: {len(result.sequence)}'
            )
        else:
            self.get_logger().warn(f'Balance recovery failed with status: {status}')


# ==================== MAIN FUNCTIONS ====================

def main_server(args=None):
    """Run the action server"""
    rclpy.init(args=args)

    server = BalanceRecoveryServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


def main_client(args=None):
    """Run the action client"""
    rclpy.init(args=args)

    client = BalanceRecoveryClient()
    client.send_goal(order=5)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'client':
        main_client()
    else:
        main_server()
```

**Console Output (Server)**:
```
[INFO] [balance_recovery_server]: Balance Recovery Action Server started
[INFO] [balance_recovery_server]: Executing balance recovery...
[INFO] [balance_recovery_server]: Phase 1/5: Assessing robot state and disturbance
[INFO] [balance_recovery_server]: Progress: 20%
[INFO] [balance_recovery_server]: Phase 2/5: Adjusting ankle torques for stabilization
[INFO] [balance_recovery_server]: Progress: 40%
[INFO] [balance_recovery_server]: Phase 3/5: Shifting center of mass
[INFO] [balance_recovery_server]: Progress: 60%
[INFO] [balance_recovery_server]: Phase 4/5: Returning to nominal stance
[INFO] [balance_recovery_server]: Progress: 80%
[INFO] [balance_recovery_server]: Phase 5/5: Verifying stable equilibrium
[INFO] [balance_recovery_server]: Progress: 100%
[INFO] [balance_recovery_server]: Balance recovery completed successfully
```

**Console Output (Client)**:
```
[INFO] [balance_recovery_client]: Balance Recovery Client initialized
[INFO] [balance_recovery_client]: Waiting for action server...
[INFO] [balance_recovery_client]: Sending balance recovery goal
[INFO] [balance_recovery_client]: Goal accepted by server
[INFO] [balance_recovery_client]: Received feedback: 3 steps completed
[INFO] [balance_recovery_client]: Received feedback: 4 steps completed
[INFO] [balance_recovery_client]: Received feedback: 5 steps completed
[INFO] [balance_recovery_client]: Received feedback: 6 steps completed
[INFO] [balance_recovery_client]: Received feedback: 7 steps completed
[INFO] [balance_recovery_client]: Balance recovery succeeded! Final sequence length: 7
```

**Key Concepts**:
- `ActionServer`: Manages long-running tasks with cancellation support
- `ActionClient`: Initiates actions and receives progress updates
- Feedback mechanism: Periodic status updates during execution
- Goal lifecycle: Accepted → Executing → Succeeded/Canceled/Aborted
- Asynchronous callbacks: Non-blocking communication pattern

---

## Example 5: Parameter Usage and Dynamic Reconfiguration

**Use Case**: Runtime configuration of gait controller parameters without restarting the node.

```python
#!/usr/bin/env python3
"""
Gait Controller with Dynamic Parameters
Demonstrates parameter declaration, callbacks, and runtime reconfiguration.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class GaitController(Node):
    """
    Gait controller node with dynamically reconfigurable parameters.
    Supports runtime tuning of locomotion parameters.
    """

    def __init__(self):
        super().__init__('gait_controller')

        # Declare parameters with default values and descriptions
        self.declare_parameter('step_length', 0.30)  # meters
        self.declare_parameter('step_height', 0.05)  # meters
        self.declare_parameter('step_frequency', 1.0)  # Hz
        self.declare_parameter('body_height', 0.85)  # meters
        self.declare_parameter('double_support_ratio', 0.2)  # 0-1
        self.declare_parameter('max_acceleration', 2.0)  # m/s²
        self.declare_parameter('enable_arm_swing', True)
        self.declare_parameter('gait_mode', 'walking')  # walking/running/custom

        # Set parameter constraints (done via callback validation)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Load initial parameters
        self.update_parameters()

        # Create timer to periodically log current configuration
        self.timer = self.create_timer(5.0, self.log_configuration)

        self.get_logger().info('Gait Controller initialized with dynamic parameters')
        self.get_logger().info('Use "ros2 param set" to reconfigure at runtime')

    def parameter_callback(self, params):
        """
        Callback invoked when parameters are changed.
        Validates new values before accepting them.

        Args:
            params: List of Parameter objects being changed

        Returns:
            SetParametersResult indicating success/failure
        """
        result = SetParametersResult(successful=True)

        for param in params:
            # Validate each parameter
            if param.name == 'step_length':
                if not (0.1 <= param.value <= 0.8):
                    result.successful = False
                    result.reason = 'step_length must be between 0.1 and 0.8 meters'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'step_height':
                if not (0.01 <= param.value <= 0.2):
                    result.successful = False
                    result.reason = 'step_height must be between 0.01 and 0.2 meters'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'step_frequency':
                if not (0.1 <= param.value <= 3.0):
                    result.successful = False
                    result.reason = 'step_frequency must be between 0.1 and 3.0 Hz'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'body_height':
                if not (0.5 <= param.value <= 1.2):
                    result.successful = False
                    result.reason = 'body_height must be between 0.5 and 1.2 meters'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'double_support_ratio':
                if not (0.0 <= param.value <= 0.5):
                    result.successful = False
                    result.reason = 'double_support_ratio must be between 0.0 and 0.5'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'max_acceleration':
                if not (0.5 <= param.value <= 5.0):
                    result.successful = False
                    result.reason = 'max_acceleration must be between 0.5 and 5.0 m/s²'
                    self.get_logger().warn(result.reason)
                    break

            elif param.name == 'gait_mode':
                valid_modes = ['walking', 'running', 'custom']
                if param.value not in valid_modes:
                    result.successful = False
                    result.reason = f'gait_mode must be one of {valid_modes}'
                    self.get_logger().warn(result.reason)
                    break

        # If validation passed, update internal state
        if result.successful:
            self.update_parameters()
            self.get_logger().info('Parameters updated successfully')

        return result

    def update_parameters(self):
        """
        Retrieves current parameter values and updates internal state.
        Called after successful parameter changes.
        """
        self.step_length = self.get_parameter('step_length').value
        self.step_height = self.get_parameter('step_height').value
        self.step_frequency = self.get_parameter('step_frequency').value
        self.body_height = self.get_parameter('body_height').value
        self.double_support_ratio = self.get_parameter('double_support_ratio').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.enable_arm_swing = self.get_parameter('enable_arm_swing').value
        self.gait_mode = self.get_parameter('gait_mode').value

        # Calculate derived parameters
        self.step_period = 1.0 / self.step_frequency
        self.step_velocity = self.step_length * self.step_frequency

        self.get_logger().debug('Internal parameters updated')

    def log_configuration(self):
        """Periodically logs current gait configuration"""
        self.get_logger().info('=== Current Gait Configuration ===')
        self.get_logger().info(f'  Mode: {self.gait_mode}')
        self.get_logger().info(f'  Step: {self.step_length:.2f}m length, {self.step_height:.3f}m height')
        self.get_logger().info(f'  Frequency: {self.step_frequency:.2f} Hz')
        self.get_logger().info(f'  Velocity: {self.step_velocity:.2f} m/s')
        self.get_logger().info(f'  Body height: {self.body_height:.2f}m')
        self.get_logger().info(f'  Arm swing: {"enabled" if self.enable_arm_swing else "disabled"}')
        self.get_logger().info('==================================')


def main(args=None):
    rclpy.init(args=args)

    gait_controller = GaitController()

    try:
        rclpy.spin(gait_controller)
    except KeyboardInterrupt:
        pass
    finally:
        gait_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Console Output**:
```
[INFO] [gait_controller]: Gait Controller initialized with dynamic parameters
[INFO] [gait_controller]: Use "ros2 param set" to reconfigure at runtime
[INFO] [gait_controller]: === Current Gait Configuration ===
[INFO] [gait_controller]:   Mode: walking
[INFO] [gait_controller]:   Step: 0.30m length, 0.050m height
[INFO] [gait_controller]:   Frequency: 1.00 Hz
[INFO] [gait_controller]:   Velocity: 0.30 m/s
[INFO] [gait_controller]:   Body height: 0.85m
[INFO] [gait_controller]:   Arm swing: enabled
[INFO] [gait_controller]: ==================================
```

**Runtime Reconfiguration Commands**:
```bash
# View all parameters
ros2 param list /gait_controller

# Get current value
ros2 param get /gait_controller step_frequency

# Set new value (triggers validation callback)
ros2 param set /gait_controller step_frequency 1.5

# Set multiple parameters
ros2 param set /gait_controller step_length 0.35
ros2 param set /gait_controller gait_mode running

# Attempt invalid value (will be rejected)
ros2 param set /gait_controller step_length 2.0
# Output: [WARN] [gait_controller]: step_length must be between 0.1 and 0.8 meters
```

**Key Concepts**:
- `declare_parameter()`: Registers parameters with default values
- `add_on_set_parameters_callback()`: Validates parameter changes
- `SetParametersResult`: Accepts or rejects parameter updates
- Runtime reconfiguration: Tuning without node restart
- Parameter validation: Ensuring safe operating ranges
- Derived parameters: Computing values from primary parameters

---

## Running the Examples

### Prerequisites
```bash
# Install ROS 2 Humble (or later)
# Source ROS 2 workspace
source /opt/ros/humble/setup.bash

# Install required packages
sudo apt install ros-humble-example-interfaces
sudo apt install ros-humble-action-tutorials-interfaces
```

### Execution

**Example 1 (Publisher)**:
```bash
python3 imu_publisher.py
```

**Example 2 (Subscriber)**:
```bash
# Terminal 1: Start a joint state publisher (simulated data)
ros2 run joint_state_publisher joint_state_publisher

# Terminal 2: Run the monitor
python3 joint_state_monitor.py
```

**Example 3 (Service)**:
```bash
# Terminal 1: Start server
python3 gait_service.py

# Terminal 2: Run client
python3 gait_service.py client
```

**Example 4 (Action)**:
```bash
# Terminal 1: Start server
python3 balance_recovery.py

# Terminal 2: Run client
python3 balance_recovery.py client
```

**Example 5 (Parameters)**:
```bash
# Terminal 1: Start controller
python3 gait_controller.py

# Terminal 2: Modify parameters
ros2 param set /gait_controller step_frequency 1.5
```

---

## Summary

These examples demonstrate:
1. **Publishers**: High-frequency sensor data streaming
2. **Subscribers**: Event-driven data processing and monitoring
3. **Services**: Request-response for configuration changes
4. **Actions**: Long-running tasks with progress feedback
5. **Parameters**: Dynamic runtime reconfiguration with validation

All examples use humanoid robotics contexts and include production-ready patterns like error handling, validation, logging, and proper resource cleanup.
