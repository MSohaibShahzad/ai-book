---
title: "Module 1 Exercises"
slug: "ros2-exercises"
sidebar_label: "Exercises"
description: "Practice exercises for ROS 2 foundations covering recall, application, and synthesis tasks"
---

# Module 1: ROS 2 Foundations - Exercises

This page contains 15 exercises organized by Bloom's Taxonomy levels to reinforce learning objectives from all five chapters of Module 1. Exercises progress from basic recall through hands-on application to open-ended synthesis challenges.

**Legend**:
- 📘 **Recall**: Basic comprehension, definitions, explanations
- 📗 **Application**: Hands-on implementation, code writing
- 📕 **Synthesis**: Open-ended challenges, system design

---

## Chapter 1: ROS 2 Concepts and Architecture

### Exercise 1.1: ROS 2 Communication Paradigms 📘

**Difficulty**: Recall
**Time**: 15 minutes
**Chapter**: 1 - ROS 2 Concepts and Architecture

**Problem**:
Explain the three primary communication patterns in ROS 2 (topics, services, actions) and identify which pattern is most appropriate for each of the following humanoid robot scenarios:

1. Streaming continuous IMU data from the torso at 100Hz
2. Requesting the current battery level from the power management system
3. Executing a 5-second arm reaching motion with progress updates
4. Broadcasting emergency stop signals to all robot subsystems
5. Querying available gait patterns from the locomotion controller

**Learning objectives**:
- Distinguish between publish-subscribe, request-response, and goal-oriented communication
- Apply communication pattern selection criteria to real robotics scenarios
- Understand latency, feedback, and cardinality characteristics of each pattern

**Deliverable**:
Submit a document with:
- Brief description (2-3 sentences) of each communication pattern
- For each scenario, state the appropriate pattern and justify your choice (1 sentence per scenario)
- One advantage and one limitation of each communication pattern

---

### Exercise 1.2: DDS Middleware and QoS Policies 📘

**Difficulty**: Recall
**Time**: 20 minutes
**Chapter**: 1 - ROS 2 Concepts and Architecture

**Problem**:
ROS 2 uses DDS (Data Distribution Service) as its middleware layer. Answer the following questions about DDS and Quality of Service (QoS) policies:

1. What problem does DDS solve that was present in ROS 1?
2. Define the following QoS policies and give one use case for each:
   - **Reliability**: BEST_EFFORT vs RELIABLE
   - **Durability**: VOLATILE vs TRANSIENT_LOCAL
   - **History**: KEEP_LAST vs KEEP_ALL
3. Why might you use BEST_EFFORT reliability for joint state messages but RELIABLE for goal commands?
4. What is QoS compatibility, and what happens when a publisher and subscriber have incompatible QoS settings?

**Learning objectives**:
- Understand the role of middleware in distributed robotics systems
- Recognize QoS policy tradeoffs between performance and guarantees
- Identify appropriate QoS configurations for different data types

**Deliverable**:
A short technical memo (1-2 pages) answering all questions with examples from humanoid robotics applications.

---

### Exercise 1.3: Build a Simple ROS 2 Publisher 📗

**Difficulty**: Application
**Time**: 45 minutes
**Chapter**: 1 - ROS 2 Concepts and Architecture

**Problem**:
Create a ROS 2 Python node that publishes simulated battery status for a humanoid robot. The node should:

1. Publish to the `/battery_status` topic using the `sensor_msgs/BatteryState` message type
2. Simulate battery discharge at a rate of 0.5% per second
3. Publish updates at 1 Hz
4. Include voltage (starts at 48.0V, decreases proportionally with percentage)
5. Set appropriate QoS settings for battery monitoring
6. Log warnings when battery drops below 20% and critical alerts below 10%

**Learning objectives**:
- Set up a ROS 2 workspace and create a Python node
- Use standard ROS 2 message types from `sensor_msgs`
- Implement timer-based callbacks for periodic publishing
- Apply appropriate QoS policies for sensor data
- Add logging for operational awareness

**Hints**:
- Use `self.create_publisher()` with queue size 10
- Battery percentage ranges from 0.0 to 1.0 (not 0-100)
- Voltage calculation: `voltage = 48.0 * (percentage / 1.0)`
- Use `self.get_logger().warn()` for warnings

**Deliverable**:
- Complete Python source code file
- Screenshot of `ros2 topic echo /battery_status` showing at least 5 messages
- Brief explanation (2-3 sentences) of your QoS choice

---

## Chapter 2: Topics, Services, and QoS

### Exercise 2.1: Topic Introspection and Analysis 📘

**Difficulty**: Recall
**Time**: 25 minutes
**Chapter**: 2 - Topics, Services, and QoS

**Problem**:
Given the following ROS 2 topic list from a running humanoid robot system:

```
/joint_states (sensor_msgs/msg/JointState)
/camera/rgb/image_raw (sensor_msgs/msg/Image)
/cmd_vel (geometry_msgs/msg/Twist)
/imu (sensor_msgs/msg/Imu)
/battery_status (sensor_msgs/msg/BatteryState)
/odom (nav_msgs/msg/Odometry)
```

Answer the following:

1. Which topics should use RELIABLE reliability and why?
2. Which topics should use BEST_EFFORT reliability and why?
3. For the `/camera/rgb/image_raw` topic, would you use KEEP_LAST(1) or KEEP_LAST(10)? Justify your answer.
4. Why would `/odom` benefit from TRANSIENT_LOCAL durability?
5. Describe a potential issue if `/cmd_vel` uses BEST_EFFORT reliability.

**Learning objectives**:
- Analyze data characteristics to determine appropriate QoS settings
- Understand the consequences of QoS policy choices
- Recognize patterns in standard robotics topic configurations

**Deliverable**:
Written responses to all five questions with technical justifications (2-3 sentences each).

---

### Exercise 2.2: Implement a Service for Joint Calibration 📗

**Difficulty**: Application
**Time**: 60 minutes
**Chapter**: 2 - Topics, Services, and QoS

**Problem**:
Design and implement a ROS 2 service for calibrating a humanoid robot's joints. Create both the service definition and a server/client implementation:

1. **Custom Service Definition** (`CalibrateJoint.srv`):
   - Request: `string joint_name`, `float64 calibration_offset`
   - Response: `bool success`, `string message`, `float64 final_position`

2. **Service Server**:
   - Accept calibration requests
   - Validate joint names (use: `left_knee`, `right_knee`, `left_ankle`, `right_ankle`)
   - Simulate applying calibration offset (store in a dictionary)
   - Return success status and confirmation message

3. **Service Client**:
   - Send calibration request for `left_knee` with offset `0.05` radians
   - Print the response

**Learning objectives**:
- Define custom ROS 2 service interfaces
- Implement service servers with request validation
- Create service clients with error handling
- Understand request-response communication patterns

**Hints**:
- Place `.srv` file in `srv/` folder and update `CMakeLists.txt`/`package.xml`
- Use `self.create_service()` for server creation
- Use `self.create_client()` for client creation
- Wait for service with `client.wait_for_service()`

**Deliverable**:
- Service definition file (`.srv`)
- Server Python code
- Client Python code
- Screenshot showing successful service call

---

### Exercise 2.3: QoS Compatibility Debugging 📗

**Difficulty**: Application
**Time**: 40 minutes
**Chapter**: 2 - Topics, Services, and QoS

**Problem**:
A humanoid robot has a publisher and subscriber on the `/force_sensor` topic, but they are not communicating. Debug and fix the QoS incompatibility:

**Publisher QoS**:
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)
```

**Subscriber QoS**:
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Tasks**:
1. Identify which QoS policies are incompatible
2. Explain why each incompatibility prevents communication
3. Propose two solutions: (a) modify publisher, (b) modify subscriber
4. Recommend which solution is better for force sensor data and justify

**Learning objectives**:
- Diagnose QoS compatibility issues
- Understand QoS matching rules
- Make informed decisions about QoS policy selection
- Apply domain knowledge to policy configuration

**Deliverable**:
Technical report with:
- List of incompatible policies with explanations
- Two corrected QoS configurations
- Recommendation with justification (3-4 sentences)

---

## Chapter 3: Actions, Parameters, and Lifecycle

### Exercise 3.1: Action Goal Lifecycle States 📘

**Difficulty**: Recall
**Time**: 20 minutes
**Chapter**: 3 - Actions, Parameters, and Lifecycle

**Problem**:
Draw a state diagram showing the complete lifecycle of a ROS 2 action goal from client request to completion. Include the following states and transitions:

**States**: UNKNOWN, ACCEPTED, EXECUTING, CANCELING, SUCCEEDED, CANCELED, ABORTED

**Scenarios to illustrate**:
1. Normal successful execution
2. Client requests cancellation mid-execution
3. Server aborts due to error
4. Server rejects goal before execution

For each transition, label it with the event that triggers it (e.g., "goal accepted", "cancel requested", "execution error", "completion").

**Learning objectives**:
- Understand action goal lifecycle states
- Recognize valid state transitions
- Identify events that trigger state changes
- Apply knowledge to debug action failures

**Deliverable**:
- State diagram (hand-drawn or digital)
- Brief description (1 sentence) for each of the 4 scenarios
- Explanation of when a goal reaches ABORTED vs CANCELED

---

### Exercise 3.2: Implement a Grasping Action Server 📗

**Difficulty**: Application
**Time**: 75 minutes
**Chapter**: 3 - Actions, Parameters, and Lifecycle

**Problem**:
Create a ROS 2 action server that simulates a humanoid robot grasping an object. Use the standard `control_msgs/action/GripperCommand` action or create a custom action with:

**Goal**: Target finger position (0.0 = fully open, 1.0 = fully closed)
**Feedback**: Current position, applied force
**Result**: Final position, grasp success (bool)

**Requirements**:
1. Simulate gradual finger closing over 3 seconds
2. Publish feedback at 10 Hz showing incremental progress
3. Detect successful grasp when position reaches within 0.02 of target
4. Support cancellation at any point during execution
5. Simulate grasp failure if target is > 0.95 (object too large)

**Learning objectives**:
- Implement action servers with feedback loops
- Handle goal cancellation requests
- Manage long-running tasks with progress reporting
- Distinguish between success and failure conditions

**Hints**:
- Use `time.sleep(0.1)` for 10 Hz feedback
- Check `goal_handle.is_cancel_requested` in loop
- Interpolate position: `current_pos += (target - current_pos) * 0.033`
- Call `goal_handle.succeed()` or `goal_handle.abort()`

**Deliverable**:
- Action server Python code
- Screenshot of client receiving feedback messages
- Brief test report showing success and failure cases

---

### Exercise 3.3: Design a Multi-Phase Action System 📕

**Difficulty**: Synthesis
**Time**: 90 minutes
**Chapter**: 3 - Actions, Parameters, and Lifecycle

**Problem**:
Design an action-based system for a humanoid robot to perform a "pick and place" task. The system should coordinate multiple actions:

1. **Navigate** to object location (navigation action)
2. **Reach** arm to pre-grasp pose (motion planning action)
3. **Grasp** object (gripper action)
4. **Lift** object (trajectory execution action)
5. **Navigate** to placement location
6. **Lower** and **release** object

**Design Requirements**:
- Create action interfaces for each phase (at least 3 custom actions)
- Design a state machine to coordinate action sequence
- Handle failures at each phase (e.g., failed grasp → retry or abort)
- Provide overall progress feedback (e.g., "Phase 2/6: Reaching")
- Support task cancellation with safe cleanup

**Learning objectives**:
- Design complex multi-action workflows
- Coordinate asynchronous action sequences
- Implement error recovery strategies
- Create hierarchical action systems

**Deliverable**:
- System architecture diagram showing all actions and their relationships
- State machine diagram with states, transitions, and error paths
- Pseudocode or Python outline for the coordinator node
- Written explanation (1 page) of failure handling strategy

---

## Chapter 4: URDF Modeling and Transforms

### Exercise 4.1: URDF Structure and Conventions 📘

**Difficulty**: Recall
**Time**: 25 minutes
**Chapter**: 4 - URDF Modeling and Transforms

**Problem**:
Answer the following questions about URDF (Unified Robot Description Format):

1. What is the difference between a `<link>` and a `<joint>` in URDF?
2. List and describe the four types of joints commonly used in humanoid robots
3. What are the three required properties of a `<joint>` element?
4. Why must every URDF have exactly one root link with no parent joint?
5. What is the purpose of the `<origin>` element in a joint definition?
6. Explain the difference between visual and collision geometry

**Learning objectives**:
- Understand URDF structural components
- Recognize joint types and their applications
- Comprehend kinematic tree organization
- Distinguish between visual and collision representations

**Deliverable**:
Written responses to all six questions with examples from humanoid robotics (e.g., shoulder joint type, leg collision geometry).

---

### Exercise 4.2: Create a URDF for a Simplified Arm 📗

**Difficulty**: Application
**Time**: 60 minutes
**Chapter**: 4 - URDF Modeling and Transforms

**Problem**:
Create a URDF file for a simplified 3-DOF humanoid robot arm with the following specifications:

**Links**:
- `shoulder_link`: 0.1m × 0.1m × 0.15m box
- `upper_arm_link`: 0.3m length, 0.05m radius cylinder
- `forearm_link`: 0.25m length, 0.04m radius cylinder
- `hand_link`: 0.08m × 0.06m × 0.02m box

**Joints**:
- `shoulder_pitch`: Revolute, Y-axis, range [-π/2, π/2]
- `shoulder_roll`: Revolute, X-axis, range [-π/4, π/4]
- `elbow`: Revolute, Y-axis, range [0, 3π/4]

**Requirements**:
1. Define all links with visual and collision geometry
2. Set reasonable mass and inertia values
3. Position joints at anatomically correct locations
4. Visualize in RViz to verify correctness

**Learning objectives**:
- Write valid URDF XML
- Define geometric primitives (box, cylinder, sphere)
- Set joint limits and axes
- Calculate basic inertial properties

**Hints**:
- Cylinder length is along Z-axis by default
- Use `<origin xyz="0 0 length/2"/>` to position visual geometry
- Mass of upper_arm ≈ 2.5 kg, forearm ≈ 1.5 kg
- Test with: `ros2 launch urdf_tutorial display.launch.py model:=arm.urdf`

**Deliverable**:
- Complete URDF file
- Screenshot from RViz showing the arm in default pose
- Screenshot showing the arm with joints at limit positions

---

### Exercise 4.3: Transform Tree Debugging 📗

**Difficulty**: Application
**Time**: 45 minutes
**Chapter**: 4 - URDF Modeling and Transforms

**Problem**:
A humanoid robot's transform tree has errors preventing proper visualization. The robot should have the following structure:

```
base_link → torso → left_shoulder → left_upper_arm → left_forearm → left_hand
         → right_shoulder → right_upper_arm → right_forearm → right_hand
```

However, running `ros2 run tf2_tools view_frames` produces warnings:

```
Warning: TF_REPEATED_DATA ignoring data from left_forearm to left_hand
Warning: TF_NO_CHILD_FRAME_ID for transform from torso
```

**Tasks**:
1. Explain what each warning means
2. Identify the likely causes of each error
3. Describe how to debug using `tf2_echo` and `tf2_monitor`
4. Provide corrected broadcaster code snippets

**Learning objectives**:
- Debug transform tree issues
- Use tf2 command-line tools
- Understand transform broadcaster requirements
- Recognize common tf2 errors and solutions

**Deliverable**:
Technical debugging report with:
- Explanation of each warning
- Commands used for debugging
- Root cause analysis
- Corrected code with explanations

---

## Chapter 5: LLM Integration and Voice Control

### Exercise 5.1: LLM Prompt Engineering for Robotics 📘

**Difficulty**: Recall
**Time**: 30 minutes
**Chapter**: 5 - LLM Integration and Voice Control

**Problem**:
You are designing prompts for an LLM to control a humanoid robot through natural language. Evaluate the following user commands and explain what information the LLM needs to extract to generate proper ROS 2 commands:

**User Commands**:
1. "Walk forward three steps"
2. "Pick up the red block on the table"
3. "Wave hello with your right hand"
4. "Turn 90 degrees to the left"
5. "Stand on one leg for balance training"

For each command:
- Identify required parameters (e.g., direction, distance, object, speed)
- List any ambiguities that need clarification
- Specify which ROS 2 topic/service/action should be called
- Suggest error cases the LLM should handle

**Learning objectives**:
- Understand natural language to robot command translation
- Identify parameters and constraints in verbal instructions
- Recognize ambiguities requiring clarification
- Map high-level intentions to low-level robot actions

**Deliverable**:
Table with columns: User Command | Parameters | Ambiguities | ROS 2 Interface | Error Cases

---

### Exercise 5.2: Build a Voice Command Publisher 📗

**Difficulty**: Application
**Time**: 70 minutes
**Chapter**: 5 - LLM Integration and Voice Control

**Problem**:
Create a ROS 2 node that accepts voice commands and publishes them for robot control:

**Components**:
1. **Speech Recognition**: Use Google Speech Recognition or Whisper API
2. **Command Parser**: Extract intent and parameters using regex or simple NLP
3. **ROS 2 Publisher**: Publish parsed commands to `/voice_commands`

**Supported Commands**:
- "walk [forward|backward|left|right] [number] steps"
- "turn [left|right] [number] degrees"
- "wave [left|right] hand"
- "stop" (emergency stop)

**Message Type**: Create custom message `VoiceCommand.msg`:
```
string raw_text
string intent
string[] parameters
float64 confidence
```

**Requirements**:
1. Continuously listen for voice input
2. Parse commands into structured format
3. Handle unrecognized commands gracefully
4. Log confidence scores from speech recognition

**Learning objectives**:
- Integrate speech recognition with ROS 2
- Parse natural language commands
- Design custom message types for semantic information
- Handle uncertain or ambiguous input

**Hints**:
- Use `speech_recognition` Python library: `pip install SpeechRecognition`
- For testing without microphone, use `recognize_sphinx` (offline)
- Regex pattern for "walk forward 3 steps": `r"walk (forward|backward|left|right) (\d+) steps?"`
- Set confidence threshold (e.g., 0.7) to filter low-quality recognition

**Deliverable**:
- Python node code
- Custom message definition
- Test results showing 5 correctly parsed commands
- Error handling demonstration (unrecognized command)

---

### Exercise 5.3: Design an LLM-Powered Robot Task Planner 📕

**Difficulty**: Synthesis
**Time**: 120 minutes
**Chapter**: 5 - LLM Integration and Voice Control

**Problem**:
Design and prototype a system where an LLM decomposes high-level user goals into sequences of ROS 2 actions for a humanoid robot. The system should:

**Input**: Natural language task description (e.g., "Clean the table and put the dishes in the sink")

**Output**: Structured task plan as a sequence of robot primitives:
- Navigate(location)
- Grasp(object)
- Place(location)
- Manipulate(action, parameters)

**System Architecture**:
1. **LLM Planner**: Uses GPT-4, Claude, or Llama to generate task plans
2. **Plan Validator**: Checks feasibility (e.g., object reachability, collision)
3. **Execution Monitor**: Tracks progress and handles failures
4. **Feedback Loop**: Reports execution status back to LLM for replanning

**Design Challenges to Address**:
- How to represent robot capabilities and environment state for the LLM
- How to validate LLM-generated plans before execution
- How to handle execution failures (e.g., failed grasp)
- How to provide feedback to improve future plans
- How to ensure safety constraints are never violated

**Learning objectives**:
- Design human-robot interaction systems using LLMs
- Create task planning architectures for robotics
- Implement validation and safety checks
- Handle uncertainty in AI-generated plans
- Design feedback loops for iterative improvement

**Deliverable**:
- System architecture diagram with all components
- LLM prompt template showing how to query for plans
- Plan representation format (e.g., JSON schema)
- Validation algorithm pseudocode
- Failure recovery strategy document (1-2 pages)
- Prototype implementation (Python outline with key functions)

**Bonus Challenge**:
Implement a working prototype using OpenAI API or local LLM, and demonstrate it executing a simple multi-step task in simulation.

---

## Exercise Progression Summary

| Chapter | Recall 📘 | Application 📗 | Synthesis 📕 |
|---------|-----------|----------------|--------------|
| Ch 1: ROS 2 Concepts | Ex 1.1, 1.2 | Ex 1.3 | - |
| Ch 2: Topics & Services | Ex 2.1 | Ex 2.2, 2.3 | - |
| Ch 3: Actions & Lifecycle | Ex 3.1 | Ex 3.2 | Ex 3.3 |
| Ch 4: URDF & Transforms | Ex 4.1 | Ex 4.2, 4.3 | - |
| Ch 5: LLM Integration | Ex 5.1 | Ex 5.2 | Ex 5.3 |

---

## Submission Guidelines

For all exercises:

1. **Code submissions**: Include requirements.txt, README with setup instructions
2. **Diagrams**: Use draw.io, Lucidchart, or hand-drawn (clearly scanned)
3. **Screenshots**: Annotate to highlight relevant information
4. **Written responses**: Use clear technical language, cite sources where applicable
5. **Testing**: Verify all code runs on ROS 2 Humble or later

**Evaluation Criteria**:
- **Correctness**: Does it solve the problem as specified?
- **Completeness**: Are all requirements addressed?
- **Code Quality**: Proper structure, comments, error handling
- **Understanding**: Written explanations demonstrate deep comprehension
- **Creativity** (synthesis exercises): Novel approaches and thorough analysis

---

## Additional Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- URDF Tutorials: https://wiki.ros.org/urdf/Tutorials
- tf2 Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- ROS 2 Action Design: https://design.ros2.org/articles/actions.html
- Speech Recognition Library: https://github.com/Uberi/speech_recognition

Good luck with the exercises! Remember to test incrementally and use `ros2 topic echo`, `ros2 service call`, and `ros2 action send_goal` for debugging.
