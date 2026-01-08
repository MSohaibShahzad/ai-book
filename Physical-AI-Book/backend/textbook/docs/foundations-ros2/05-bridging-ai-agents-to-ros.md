---
title: "Chapter 5: Bridging AI Agents to ROS 2"
slug: bridging-ai-agents-to-ros
sidebar_label: "5. AI Agents to ROS 2"
toc: true
description: "Integrating LLMs, voice commands, and AI planners with ROS 2 for intelligent humanoid robot control"
---

# Chapter 5: Bridging AI Agents to ROS 2

## Introduction

You've mastered ROS 2 communication (topics, services, actions) and robot modeling (URDF). But how do you enable a humanoid robot to understand commands like "Pick up the red block" or "Walk to the kitchen"?

**The answer**: **Bridge AI agents** (LLMs, vision models, planners) to ROS 2, creating an intelligent control layer that translates natural language into robot actions.

**The driving question**: How do you safely integrate large language models (LLMs) with ROS 2 to enable voice-controlled humanoid robots while preventing unsafe commands?

**Learning Objectives**:

By the end of this chapter, you will:

1. **Architect** an AI-to-ROS 2 integration pipeline (voice → LLM → ROS 2 actions)
2. **Implement** LLM-to-ROS 2 message translation in Python
3. **Apply** safety constraints and action validation
4. **Design** a voice-to-action system for humanoid robots
5. **Debug** AI-robot communication failures

**Time**: 4 hours (reading + hands-on)

---

## 5.1 The AI-Robot Integration Challenge

### 5.1.1 Why Integrate AI with ROS 2?

**Traditional robot programming**:
```python
# Hard-coded motion sequence
move_arm_to_position([0.5, 0.2, 0.3])
close_gripper()
move_arm_to_position([0.3, 0.1, 0.5])
```

**AI-powered programming**:
```python
# Natural language command
execute_command("Pick up the red block and place it on the table")
```

**Benefits of AI integration**:
- **Natural interfaces**: Voice commands, gesture recognition
- **Adaptability**: Handle novel situations ("pick up the *red* block" vs. "pick up the *blue* block")
- **Planning**: LLMs can break complex tasks into subtasks
- **Learning**: Improve performance from demonstrations

**Challenges**:
- **Safety**: How do you prevent "Jump off the building" from executing?
- **Reliability**: LLMs hallucinate—how do you validate outputs?
- **Latency**: LLM inference takes 1-5 seconds (too slow for real-time control)
- **Grounding**: How does "the red block" map to object ID `obj_47` in the perception system?

### 5.1.2 Architecture Overview

**High-level pipeline**:

```
[Microphone] → [Whisper ASR] → [LLM Planner] → [Action Validator] → [ROS 2 Action Client] → [Robot]
                                        ↓                                       ↓
                                [Scene Context]                       [Feedback to LLM]
                                (object poses, joint states)
```

**Components**:

1. **ASR (Automatic Speech Recognition)**: Voice → text (Whisper, Google STT)
2. **LLM Planner**: Text → structured action (GPT-4, Claude, LLaMA)
3. **Action Validator**: Check safety constraints, verify object existence
4. **ROS 2 Action Client**: Send validated action to robot controllers
5. **Scene Context**: Provide LLM with current robot/world state

---

## 5.2 Voice-to-Action Pipeline

### 5.2.1 ASR: Speech-to-Text

**Option 1: Whisper** (OpenAI, open-source)

```python
import whisper

model = whisper.load_model("base")

def transcribe_audio(audio_file):
    result = model.transcribe(audio_file)
    return result["text"]

# Example
command = transcribe_audio("microphone_input.wav")
print(command)  # "Pick up the red block"
```

**Option 2: ROS 2 integration** (subscribe to audio topic)

```python
from audio_common_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.sub = self.create_subscription(
            AudioData,
            '/audio',
            self.audio_callback,
            10
        )
        self.whisper_model = whisper.load_model("base")

    def audio_callback(self, msg):
        # Convert ROS audio message to WAV
        audio_np = np.frombuffer(msg.data, dtype=np.int16)
        # Transcribe
        text = self.whisper_model.transcribe(audio_np)["text"]
        self.get_logger().info(f"Heard: {text}")
        self.process_command(text)
```

### 5.2.2 LLM Planning: Text → Structured Actions

**Goal**: Convert natural language to structured robot actions.

**Example**:
- **Input**: "Pick up the red block"
- **Output**:
  ```json
  {
    "action": "pick_and_place",
    "object": "red_block",
    "target": "gripper",
    "constraints": ["collision_free", "stable_grasp"]
  }
  ```

**LLM Prompt Engineering** (critical for reliability):

```python
import openai

def llm_to_action(command: str, scene_context: dict) -> dict:
    prompt = f"""
You are a humanoid robot control system. Convert natural language commands to structured actions.

Available actions:
- pick_and_place: Pick object and move to target
- navigate: Move to location
- manipulate: Execute joint trajectory
- observe: Look at object

Current scene:
{scene_context}

User command: "{command}"

Output JSON:
{{
  "action": "<action_type>",
  "parameters": {{...}},
  "preconditions": [...],
  "safety_checks": [...]
}}

Output:
"""

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.1  # Low temperature for deterministic output
    )

    action_json = response.choices[0].message.content
    return json.loads(action_json)

# Example usage
scene = {
    "objects": [
        {"id": "obj_1", "type": "block", "color": "red", "pose": [0.5, 0.2, 0.1]},
        {"id": "obj_2", "type": "block", "color": "blue", "pose": [0.6, 0.3, 0.1]}
    ],
    "robot_state": {
        "gripper_open": True,
        "arm_position": [0.3, 0.1, 0.4]
    }
}

action = llm_to_action("Pick up the red block", scene)
print(action)
# {
#   "action": "pick_and_place",
#   "parameters": {"object_id": "obj_1", "grasp_pose": [0.5, 0.2, 0.15]},
#   "preconditions": ["gripper_empty", "object_reachable"],
#   "safety_checks": ["collision_free"]
# }
```

**Key techniques**:
- **Few-shot learning**: Include examples in prompt
- **Structured output**: Request JSON for easy parsing
- **Context injection**: Provide scene graph, robot capabilities
- **Temperature tuning**: Low (0.1) for deterministic actions, high (0.8) for creative planning

---

## 5.3 Action Validation and Safety

### 5.3.1 Why Validation is Critical

**Problem**: LLMs can generate unsafe or infeasible commands.

**Examples**:
- "Move arm to position [10, 10, 10]" (outside workspace)
- "Apply 500 N force" (exceeds actuator limits)
- "Disable collision checking" (violates safety policies)

**Solution**: **Action validator** that checks constraints before execution.

### 5.3.2 Safety Constraint System

**Define constraints**:

```python
class ActionValidator:
    def __init__(self):
        self.workspace_bounds = {
            "x": (-0.5, 0.8),
            "y": (-0.6, 0.6),
            "z": (0.0, 1.0)
        }
        self.max_joint_velocities = {
            "shoulder": 2.0,  # rad/s
            "elbow": 2.0,
            "wrist": 3.0
        }
        self.forbidden_actions = ["disable_safety", "override_limits"]

    def validate_pick_and_place(self, action: dict) -> tuple[bool, str]:
        """Returns (is_valid, error_message)"""

        # Check 1: Object exists
        if not self.object_exists(action["parameters"]["object_id"]):
            return False, "Object not found in scene"

        # Check 2: Target position in workspace
        target = action["parameters"]["grasp_pose"]
        if not self.in_workspace(target):
            return False, f"Target {target} outside workspace"

        # Check 3: Gripper state
        if not self.robot_state["gripper_open"]:
            return False, "Gripper not empty"

        # Check 4: Collision check
        if not self.collision_free(target):
            return False, "Collision detected"

        return True, ""

    def in_workspace(self, pos: list) -> bool:
        x, y, z = pos
        return (self.workspace_bounds["x"][0] <= x <= self.workspace_bounds["x"][1] and
                self.workspace_bounds["y"][0] <= y <= self.workspace_bounds["y"][1] and
                self.workspace_bounds["z"][0] <= z <= self.workspace_bounds["z"][1])

    def object_exists(self, obj_id: str) -> bool:
        # Query perception system
        return obj_id in self.scene_objects

    def collision_free(self, target_pose: list) -> bool:
        # Use MoveIt 2 collision checking
        # Simplified here
        return True  # Placeholder
```

**Validation workflow**:

```python
def execute_command_safely(command: str):
    # 1. LLM generates action
    action = llm_to_action(command, scene_context)

    # 2. Validate action
    validator = ActionValidator()
    is_valid, error_msg = validator.validate_pick_and_place(action)

    if not is_valid:
        logger.error(f"Invalid action: {error_msg}")
        return

    # 3. Execute via ROS 2
    execute_ros_action(action)
```

### 5.3.3 Human-in-the-Loop Confirmation

**For high-risk actions**, require human approval:

```python
def execute_with_confirmation(action: dict):
    print(f"Planning to execute: {action['action']}")
    print(f"Parameters: {action['parameters']}")
    response = input("Confirm? (y/n): ")

    if response.lower() == 'y':
        execute_ros_action(action)
    else:
        print("Action cancelled by user")
```

**Example**: "Walk down the stairs" → requires confirmation (high fall risk).

---

## 5.4 ROS 2 Integration: LLM to Action Client

### 5.4.1 Complete Example: Pick and Place

**ROS 2 action client** that translates LLM output to ROS 2 actions:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
import openai
import json

class LLMToROSBridge(Node):
    def __init__(self):
        super().__init__('llm_to_ros_bridge')

        # ROS 2 action client (MoveIt 2)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # Scene context (updated by perception system)
        self.scene_context = {"objects": []}

        self.get_logger().info('LLM-to-ROS bridge ready')

    def process_voice_command(self, command: str):
        """Main pipeline: voice → LLM → validation → ROS 2"""

        self.get_logger().info(f"Processing command: {command}")

        # Step 1: LLM planning
        action = self.llm_to_action(command)
        self.get_logger().info(f"LLM output: {action}")

        # Step 2: Validation
        is_valid, error_msg = self.validate_action(action)
        if not is_valid:
            self.get_logger().error(f"Invalid action: {error_msg}")
            return

        # Step 3: Execute ROS 2 action
        self.execute_pick_and_place(action)

    def llm_to_action(self, command: str) -> dict:
        """Convert natural language to structured action using LLM"""

        prompt = f"""
You are a humanoid robot. Convert this command to a pick-and-place action.

Scene:
{json.dumps(self.scene_context, indent=2)}

Command: "{command}"

Output JSON:
{{
  "action": "pick_and_place",
  "object_id": "<object_id>",
  "grasp_pose": [x, y, z],
  "place_pose": [x, y, z]
}}
"""

        # Call LLM (OpenAI GPT-4)
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        action_json = response.choices[0].message.content
        return json.loads(action_json)

    def validate_action(self, action: dict) -> tuple[bool, str]:
        """Safety checks"""

        # Check object exists
        obj_id = action.get("object_id")
        if not any(obj["id"] == obj_id for obj in self.scene_context["objects"]):
            return False, f"Object {obj_id} not found"

        # Check workspace bounds
        grasp_pose = action["grasp_pose"]
        if not self.in_workspace(grasp_pose):
            return False, "Grasp pose outside workspace"

        return True, ""

    def in_workspace(self, pose: list) -> bool:
        x, y, z = pose
        return (-0.5 <= x <= 0.8 and -0.6 <= y <= 0.6 and 0.0 <= z <= 1.0)

    def execute_pick_and_place(self, action: dict):
        """Send goal to MoveIt 2 action server"""

        goal_msg = MoveGroup.Goal()

        # Configure grasp pose
        goal_msg.request.group_name = "arm"
        goal_msg.request.goal_constraints.append(
            self.pose_to_constraint(action["grasp_pose"])
        )

        # Send goal asynchronously
        self.get_logger().info("Sending goal to MoveIt 2...")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Progress: {feedback_msg.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed: {result}')

    def pose_to_constraint(self, pose: list):
        """Convert [x, y, z] to MoveIt constraint (simplified)"""
        # In real implementation, use moveit_msgs/Constraints
        pass


def main(args=None):
    rclpy.init(args=args)

    node = LLMToROSBridge()

    # Simulate voice command
    node.process_voice_command("Pick up the red block")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.4.2 Scene Context Management

**Problem**: LLM needs up-to-date world state (object poses, robot state).

**Solution**: Subscribe to perception topics and maintain scene graph:

```python
class LLMToROSBridge(Node):
    def __init__(self):
        # ... (previous code)

        # Subscribe to object detections
        self.create_subscription(
            DetectedObjects,
            '/perception/objects',
            self.update_scene_context,
            10
        )

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.update_robot_state,
            10
        )

    def update_scene_context(self, msg):
        """Update scene graph from perception system"""
        self.scene_context["objects"] = [
            {
                "id": obj.id,
                "type": obj.type,
                "color": obj.color,
                "pose": [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            }
            for obj in msg.objects
        ]

    def update_robot_state(self, msg):
        """Update robot state from joint states"""
        self.scene_context["robot_state"] = {
            "joint_positions": dict(zip(msg.name, msg.position))
        }
```

---

## 5.5 Error Handling and Recovery

### 5.5.1 Common Failure Modes

| **Failure** | **Cause** | **Recovery Strategy** |
|-------------|-----------|----------------------|
| **LLM hallucination** | Model generates non-existent object | Validate object IDs against scene graph |
| **Action failure** | Grasp fails (object slips) | Retry with adjusted grasp pose |
| **Timeout** | LLM takes >10s | Use cached responses, fallback to rule-based planner |
| **Network error** | Lost connection to LLM API | Switch to local model (LLaMA), queue commands |

### 5.5.2 Retry Logic

```python
def execute_with_retry(action: dict, max_retries: int = 3):
    for attempt in range(max_retries):
        result = execute_ros_action(action)

        if result.success:
            logger.info("Action succeeded")
            return

        logger.warning(f"Attempt {attempt+1} failed: {result.error}")

        # Adjust action (e.g., change grasp angle)
        action = adjust_action_parameters(action, result.error)

    logger.error("Action failed after max retries")
```

### 5.5.3 Fallback to Rule-Based Planning

```python
def process_command_with_fallback(command: str):
    try:
        # Try LLM planning
        action = llm_to_action(command, timeout=5.0)
    except TimeoutError:
        logger.warning("LLM timeout, using rule-based planner")
        action = rule_based_planner(command)

    execute_ros_action(action)
```

---

## 5.6 Real-World Example: Voice-Controlled Humanoid

### 5.6.1 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                          USER INTERFACE                         │
│  - Voice input (microphone)                                     │
│  - Visual feedback (screen/LEDs)                                │
└───────────────────────┬─────────────────────────────────────────┘
                        ↓
┌───────────────────────▼─────────────────────────────────────────┐
│                      PERCEPTION LAYER                            │
│  - Whisper ASR (speech-to-text)                                  │
│  - Object detection (YOLOv8, Detic)                              │
│  - Pose estimation (RGB-D camera)                                │
└───────────────────────┬─────────────────────────────────────────┘
                        ↓ (text command + scene context)
┌───────────────────────▼─────────────────────────────────────────┐
│                      AI PLANNING LAYER                           │
│  - LLM planner (GPT-4, Claude)                                   │
│  - Action validator (safety checks)                              │
│  - Task decomposition (break complex tasks into subtasks)        │
└───────────────────────┬─────────────────────────────────────────┘
                        ↓ (validated ROS 2 actions)
┌───────────────────────▼─────────────────────────────────────────┐
│                      ROS 2 MIDDLEWARE                            │
│  - Action clients (MoveIt 2, Nav2)                               │
│  - Topic publishers (/cmd_vel, /joint_trajectory)                │
│  - Service clients (/reset_state, /enable_motors)                │
└───────────────────────┬─────────────────────────────────────────┘
                        ↓
┌───────────────────────▼─────────────────────────────────────────┐
│                      CONTROL LAYER                               │
│  - Motor controllers (joint position/velocity)                   │
│  - Gripper controller (force feedback)                           │
│  - Balance controller (ZMP, COM stabilization)                   │
└───────────────────────┬─────────────────────────────────────────┘
                        ↓
┌───────────────────────▼─────────────────────────────────────────┐
│                         HARDWARE                                 │
│  - 40 joint actuators (arms, legs, neck)                         │
│  - 6 cameras (stereo, depth, wrist)                              │
│  - IMU, force/torque sensors                                     │
└─────────────────────────────────────────────────────────────────┘
```

### 5.6.2 Example Interaction

**User**: "Pick up the red block and place it in the box"

**System**:
1. **Whisper ASR** → Text: "Pick up the red block and place it in the box"
2. **Perception** → Scene context:
   ```json
   {
     "objects": [
       {"id": "obj_1", "type": "block", "color": "red", "pose": [0.5, 0.2, 0.1]},
       {"id": "obj_2", "type": "box", "pose": [0.3, -0.1, 0.05]}
     ]
   }
   ```
3. **LLM Planner** → Action:
   ```json
   {
     "action": "pick_and_place",
     "subtasks": [
       {"action": "grasp", "object_id": "obj_1", "pose": [0.5, 0.2, 0.15]},
       {"action": "move", "target_pose": [0.3, -0.1, 0.2]},
       {"action": "release"}
     ]
   }
   ```
4. **Validator** → Checks workspace bounds, object existence ✅
5. **ROS 2 Action Client** → Sends goal to MoveIt 2
6. **MoveIt 2** → Computes trajectory, executes motion
7. **Feedback to User** → "Task completed"

---

## 5.7 Latency Optimization

### 5.7.1 The Latency Problem

**LLM inference**: 1-5 seconds (unacceptable for real-time control)

**Solution strategies**:

1. **Asynchronous planning**: LLM plans while robot executes previous action
2. **Caching**: Store common command → action mappings
3. **Local models**: Use fine-tuned LLaMA (faster than GPT-4 API)
4. **Hybrid approach**: LLM for high-level planning, reactive controller for low-level execution

### 5.7.2 Caching Example

```python
class CachedLLMPlanner:
    def __init__(self):
        self.cache = {
            "pick up the red block": {
                "action": "pick_and_place",
                "object_color": "red"
            },
            "move forward": {
                "action": "navigate",
                "direction": "forward",
                "distance": 1.0
            }
        }

    def plan(self, command: str) -> dict:
        # Check cache first
        if command in self.cache:
            return self.cache[command]

        # Fallback to LLM
        action = llm_to_action(command)
        self.cache[command] = action
        return action
```

---

## 5.8 Debugging AI-Robot Integration

### 5.8.1 Command-Line Tools

**Test LLM output** (without robot):

```bash
# Mock scene context
echo '{"objects": [{"id": "obj_1", "color": "red"}]}' > scene.json

# Test LLM prompt
python3 test_llm.py "Pick up the red block" scene.json
```

**Monitor ROS 2 actions**:

```bash
ros2 action list
ros2 action info /move_action
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{...}" --feedback
```

### 5.8.2 Common Issues

**Problem 1**: LLM generates action for non-existent object

**Solution**: Add object validation in prompt:
```
Available objects: [list objects from scene context]
Only use objects from this list.
```

**Problem 2**: Action executes but robot doesn't move

**Solution**: Check ROS 2 action server is running:
```bash
ros2 node list  # Is controller node running?
ros2 topic echo /joint_states  # Are joint states publishing?
```

**Problem 3**: LLM output is not valid JSON

**Solution**: Add JSON validation and retry:
```python
def parse_llm_output(text: str) -> dict:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # Extract JSON from markdown code block
        match = re.search(r'```json\n(.*?)\n```', text, re.DOTALL)
        if match:
            return json.loads(match.group(1))
        raise ValueError("Invalid JSON output")
```

---

## 5.9 Summary

**What you learned**:

✅ **AI-to-ROS 2 architecture**: Voice → ASR → LLM → Validator → ROS 2 actions
✅ **LLM integration**: Prompt engineering for structured action output
✅ **Safety validation**: Constraint checking before execution
✅ **Action clients**: Translate LLM output to ROS 2 action goals
✅ **Error handling**: Retry logic, fallback planning, timeout management
✅ **Scene context**: Subscribe to perception topics, maintain world state

**What's next**:

You've completed **Module 1: ROS 2 Foundations**! Next modules:
- **Module 2**: Vision-Language-Action (VLA) models
- **Module 3**: Manipulation and grasping
- **Module 4**: Bipedal locomotion

---

## 5.10 Self-Check Questions

Before proceeding, ensure you can answer:

1. **What are the main components of an AI-to-ROS 2 pipeline?** (Answer: ASR, LLM planner, validator, ROS 2 action client)
2. **Why is action validation necessary?** (Answer: Prevent unsafe/infeasible commands from executing)
3. **How do you provide scene context to an LLM?** (Answer: Subscribe to perception topics, include in prompt)
4. **What is the difference between high-level and low-level control?** (Answer: High-level = task planning (LLM), low-level = motor commands (controller))
5. **How do you handle LLM timeouts?** (Answer: Use cached responses, fallback to rule-based planner)

If you're unsure on any, re-read the relevant section.

---

## 5.11 Exercises

See **[Exercises](./exercises.md)** for hands-on challenges:
- **Recall**: Explain voice-to-action pipeline, list safety checks
- **Application**: Implement a simple LLM-to-ROS 2 bridge for navigation commands
- **Synthesis**: Design a multi-modal interface (voice + gesture) for humanoid control

---

## 5.12 Further Reading

**Papers**:
- "LLMs for Robot Control: A Survey" (arXiv 2023)
- "SayCan: Grounding Language in Robotic Affordances" (Google, ICRA 2022)
- "Code as Policies: Language Model Programs for Embodied Control" (Google, ICRA 2023)

**Open-source projects**:
- [ros-llm](https://github.com/example/ros-llm): LLM integration for ROS 2
- [whisper-ros](https://github.com/example/whisper-ros): Whisper ASR node for ROS 2

**Videos**:
- "Building Voice-Controlled Robots with ROS 2 and GPT-4" (ROSCon 2023)

---

**End of Module 1!** Congratulations—you now understand ROS 2 foundations and AI integration. Take a break, then proceed to **Module 2: Vision-Language-Action Models**.

**Next resources**:
- [Worked Examples](./examples.md): Complete code for LLM-ROS 2 bridge
- [Practice Exercises](./exercises.md): Build your own voice-controlled robot
