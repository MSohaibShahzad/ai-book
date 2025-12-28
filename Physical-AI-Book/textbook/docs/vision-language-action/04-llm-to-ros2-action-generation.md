---
title: "Chapter 4: LLM-to-ROS 2 Action Generation"
slug: /vision-language-action/llm-to-ros2-action-generation
sidebar_label: "4. LLM-to-ROS 2 Action Generation"
sidebar_position: 5
toc: true
description: "Bridge LLM planning with ROS 2 execution by converting structured JSON action plans into ROS 2 action server calls with error handling and replanning."
---

# Chapter 4: LLM-to-ROS 2 Action Generation

## Introduction

In the previous chapter, we saw how LLMs decompose high-level commands ("Bring me a cup") into structured JSON action plans:

```json
[
  {"action": "navigate_to", "args": {"location": "kitchen"}},
  {"action": "detect_object", "args": {"class": "cup"}},
  {"action": "grasp", "args": {"object_id": "$detected_cup"}},
  {"action": "navigate_to", "args": {"location": "user"}},
  {"action": "handoff", "args": {"recipient": "user"}}
]
```

But this JSON is just a symbolic representation—it doesn't move motors or actuate grippers. The **action executor** layer bridges LLM planning (symbolic) with ROS 2 execution (physical). This chapter covers:

1. **Action primitives library**: Standardized robot actions (navigate, grasp, place)
2. **JSON-to-ROS 2 translation**: Parsing JSON → calling ROS 2 action servers
3. **Variable binding**: Passing outputs from one action (object detection) to the next (grasping)
4. **Error handling**: Timeouts, failures, replanning triggers
5. **LangChain for robotics**: Using LangChain framework for tool calling and memory

By the end, you'll implement an **action_executor_node** that subscribes to `/action_plan` (JSON from LLM) and orchestrates ROS 2 action servers (Nav2, MoveIt, custom actions) to execute tasks autonomously.

## Action Primitives Library

An **action primitive** is an atomic, reusable robot capability. Primitives abstract low-level complexity (path planning, inverse kinematics) behind simple interfaces.

### Design Principles

1. **Atomic**: Each primitive performs one logical operation (grasp object, navigate to pose)
2. **Idempotent**: Calling twice yields same result (e.g., `navigate_to("kitchen")` → already in kitchen? Do nothing.)
3. **Composable**: Primitives chain to form complex behaviors
4. **Observable**: Return success/failure status + structured feedback

### Core Primitives for Humanoid Robotics

| Primitive               | Description                                     | ROS 2 Action                    | Typical Duration |
|-------------------------|-------------------------------------------------|---------------------------------|------------------|
| `navigate_to(location)` | Move to named location (e.g., "kitchen")        | `nav2_msgs/NavigateToPose`      | 5–30 seconds     |
| `navigate_to_pose(x, y, θ)` | Move to exact coordinates                       | `nav2_msgs/NavigateToPose`      | 5–30 seconds     |
| `detect_object(class, color)` | Use vision to find object                       | Custom service                  | 0.1–1 second     |
| `grasp(object_id)`      | Grasp detected object                           | `moveit_msgs/MoveGroup` + gripper | 5–15 seconds     |
| `place(x, y, z)`        | Place held object at coordinates                | `moveit_msgs/MoveGroup`         | 5–15 seconds     |
| `handoff(recipient)`    | Extend arm toward person, open gripper          | Custom action                   | 2–5 seconds      |
| `speak(text)`           | Text-to-speech output                           | `audio_msgs/PlayAudio`          | 1–5 seconds      |
| `ask_user(question)`    | Speak + wait for voice response                 | Custom (TTS + Whisper)          | 5–30 seconds     |

### Example: `navigate_to` Primitive

**High-Level API** (what LLM calls):
```json
{"action": "navigate_to", "args": {"location": "kitchen"}}
```

**Implementation** (ROS 2 action client):
```python
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavigateToPrimitive:
    def __init__(self, node, semantic_map):
        self.node = node
        self.semantic_map = semantic_map  # Dict: location_name → (x, y, theta)
        self.nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    def execute(self, args):
        location = args["location"]

        # Look up location in semantic map
        if location not in self.semantic_map:
            return {"success": False, "error": f"Unknown location: {location}"}

        pose = self.semantic_map[location]

        # Create ROS 2 action goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose["x"]
        goal.pose.pose.position.y = pose["y"]
        goal.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        theta = pose["theta"]
        goal.pose.pose.orientation.z = math.sin(theta / 2)
        goal.pose.pose.orientation.w = math.cos(theta / 2)

        # Send goal to Nav2
        self.node.get_logger().info(f"Navigating to {location} ({pose['x']}, {pose['y']})")
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"success": False, "error": "Nav2 rejected goal"}

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result:
            return {"success": True, "final_pose": pose}
        else:
            return {"success": False, "error": "Navigation timeout"}
```

**Usage in Action Executor**:
```python
result = navigate_to_primitive.execute({"location": "kitchen"})
if result["success"]:
    print("Reached kitchen!")
else:
    print(f"Navigation failed: {result['error']}")
    # Trigger replanning
```

## JSON-to-ROS 2 Translation

The **action executor** parses LLM-generated JSON and dispatches to appropriate primitives.

### Action Executor Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                    LLM PLANNER NODE                           │
│  Input: Voice command                                         │
│  Output: JSON action plan                                     │
│  Topic: /action_plan (ActionPlan.msg)                         │
└─────────────────────────┬─────────────────────────────────────┘
                          │
                          ▼
┌───────────────────────────────────────────────────────────────┐
│                  ACTION EXECUTOR NODE                         │
│  - Subscribes to /action_plan                                 │
│  - Parses JSON                                                │
│  - Dispatches to primitives                                   │
│  - Handles errors, replanning                                 │
└─────────────────────────┬─────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┬─────────────────┐
        ▼                 ▼                 ▼                 ▼
┌────────────┐   ┌────────────┐   ┌────────────┐   ┌────────────┐
│ Nav2       │   │ MoveIt     │   │ Isaac ROS  │   │ Custom     │
│ (navigate) │   │ (grasp)    │   │ (detect)   │   │ (handoff)  │
└────────────┘   └────────────┘   └────────────┘   └────────────┘
```

### Implementation: Action Executor Node

**File**: `action_executor_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import ActionPlan, ActionResult
import json

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Semantic map (hardcoded for demo; load from file in production)
        self.semantic_map = {
            "kitchen": {"x": 5.0, "y": 3.0, "theta": 0.0},
            "living_room": {"x": 2.0, "y": 1.5, "theta": 1.57},
            "bedroom": {"x": 8.0, "y": 6.0, "theta": 3.14},
            "user": {"x": 0.0, "y": 0.0, "theta": 0.0}  # Updated dynamically
        }

        # Initialize primitives
        self.primitives = {
            "navigate_to": NavigateToPrimitive(self, self.semantic_map),
            "detect_object": DetectObjectPrimitive(self),
            "grasp": GraspPrimitive(self),
            "handoff": HandoffPrimitive(self),
            "speak": SpeakPrimitive(self)
        }

        # State
        self.execution_context = {}  # Stores variables like $detected_object

        # Subscribers
        self.plan_sub = self.create_subscription(
            ActionPlan, '/action_plan', self.execute_plan, 10)

        # Publishers
        self.result_pub = self.create_publisher(ActionResult, '/action_result', 10)

        self.get_logger().info("Action Executor ready")

    def execute_plan(self, msg):
        self.get_logger().info(f"Received plan with {len(msg.actions)} actions")

        for i, action_msg in enumerate(msg.actions):
            action_name = action_msg.name
            args = json.loads(action_msg.args)

            self.get_logger().info(f"Step {i+1}: {action_name}({args})")

            # Resolve variables (e.g., $detected_cup → actual object_id)
            args = self.resolve_variables(args)

            # Execute primitive
            if action_name not in self.primitives:
                self.get_logger().error(f"Unknown action: {action_name}")
                self.publish_failure(action_name, "Unknown action")
                return

            result = self.primitives[action_name].execute(args)

            if result["success"]:
                # Store output variables
                if "output" in result:
                    for key, value in result["output"].items():
                        self.execution_context[key] = value
                        self.get_logger().info(f"Stored {key} = {value}")
            else:
                # Action failed → trigger replanning
                self.get_logger().error(f"Action failed: {result['error']}")
                self.publish_failure(action_name, result["error"])
                self.trigger_replan(action_name, result["error"])
                return

        # All actions succeeded
        self.publish_success()

    def resolve_variables(self, args):
        """Replace $variable with actual values from execution_context."""
        resolved = {}
        for key, value in args.items():
            if isinstance(value, str) and value.startswith("$"):
                var_name = value[1:]  # Remove $
                if var_name in self.execution_context:
                    resolved[key] = self.execution_context[var_name]
                else:
                    self.get_logger().warn(f"Variable {value} not found in context")
                    resolved[key] = value
            else:
                resolved[key] = value
        return resolved

    def trigger_replan(self, failed_action, error_msg):
        # TODO: Call LLM for replanning (Chapter 3 techniques)
        self.get_logger().info("Triggering replanning...")

    def publish_success(self):
        msg = ActionResult()
        msg.success = True
        msg.message = "Task completed successfully"
        self.result_pub.publish(msg)

    def publish_failure(self, action, error):
        msg = ActionResult()
        msg.success = False
        msg.message = f"Action '{action}' failed: {error}"
        self.result_pub.publish(msg)

def main():
    rclpy.init()
    node = ActionExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Message Definitions

**File**: `robot_interfaces/msg/ActionPlan.msg`
```
std_msgs/Header header
Action[] actions
```

**File**: `robot_interfaces/msg/Action.msg`
```
string name       # Action primitive name (e.g., "navigate_to")
string args       # JSON-encoded arguments (e.g., '{"location": "kitchen"}')
```

**File**: `robot_interfaces/msg/ActionResult.msg`
```
bool success
string message
```

## Variable Binding: Passing Data Between Actions

LLM plans often have data dependencies: action B uses output from action A.

**Example**:
```json
[
  {"action": "detect_object", "args": {"class": "cup"}},
  {"action": "grasp", "args": {"object_id": "$detected_object"}}
]
```

**Step 1**: `detect_object` outputs `detected_object = "cup_001"`
**Step 2**: `grasp` receives `object_id = "cup_001"` (after variable resolution)

### Implementation: Variable Storage

**DetectObjectPrimitive**:
```python
class DetectObjectPrimitive:
    def execute(self, args):
        object_class = args["class"]

        # Call Isaac ROS object detection
        detections = self.call_isaac_ros_detection(object_class)

        if len(detections) == 0:
            return {"success": False, "error": f"No {object_class} detected"}

        detected = detections[0]  # Take first detection

        # Return output variable
        return {
            "success": True,
            "output": {
                "detected_object": detected["id"],  # e.g., "cup_001"
                "detected_pose": detected["pose"]   # (x, y, z, qx, qy, qz, qw)
            }
        }
```

**Action Executor**:
```python
# After detect_object succeeds:
result = detect_object_primitive.execute({"class": "cup"})
# result["output"] = {"detected_object": "cup_001", "detected_pose": {...}}

# Store in context
self.execution_context["detected_object"] = "cup_001"
self.execution_context["detected_pose"] = {...}

# Later, grasp action:
args = {"object_id": "$detected_object"}
resolved_args = self.resolve_variables(args)
# resolved_args = {"object_id": "cup_001"}

grasp_result = grasp_primitive.execute(resolved_args)
```

### Advanced: Conditional Execution

LLM plans can include conditionals:

```json
{
  "action": "conditional",
  "condition": "num_detected_objects > 1",
  "true_branch": [
    {"action": "ask_user", "args": {"question": "Which cup?"}}
  ],
  "false_branch": [
    {"action": "grasp", "args": {"object_id": "$detected_object"}}
  ]
}
```

**Implementation**:
```python
def execute_conditional(self, action):
    condition = action["condition"]
    result = eval(condition, {}, self.execution_context)  # Evaluate condition

    if result:
        branch = action["true_branch"]
    else:
        branch = action["false_branch"]

    for sub_action in branch:
        self.execute_action(sub_action)
```

**Security Note**: Using `eval()` is dangerous (arbitrary code execution). In production, use a safe expression evaluator (e.g., `simpleeval` library).

## Error Handling and Replanning

Actions fail frequently in real-world robotics. The executor must detect failures, categorize them, and trigger appropriate recovery.

### Failure Categories

| Failure Type           | Example                           | Recovery Strategy            |
|------------------------|-----------------------------------|------------------------------|
| **Perception failure** | Object not detected               | Search alternative locations |
| **Manipulation failure** | Grasp failed (object slipped)     | Retry with different grasp   |
| **Navigation failure** | Path blocked                      | Replan path or wait          |
| **Timeout**            | Action took >60 seconds           | Abort and replan             |
| **Safety violation**   | Force limit exceeded              | Emergency stop               |

### Timeout Handling

**Problem**: Nav2 navigation might take 10 seconds or 10 minutes (if stuck).

**Solution**: Set action timeout; abort if exceeded:

```python
from rclpy.task import Future

def execute_with_timeout(self, primitive, args, timeout_sec=60.0):
    future = Future()

    def callback():
        result = primitive.execute(args)
        future.set_result(result)

    thread = threading.Thread(target=callback)
    thread.start()

    rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

    if future.done():
        return future.result()
    else:
        # Timeout
        return {"success": False, "error": "Action timeout"}
```

### Retry Logic

Some failures are transient (e.g., camera glare causes detection failure).

**Strategy**: Retry up to 3 times before declaring permanent failure:

```python
def execute_with_retry(self, primitive, args, max_retries=3):
    for attempt in range(max_retries):
        result = primitive.execute(args)

        if result["success"]:
            return result
        else:
            self.get_logger().warn(f"Attempt {attempt+1} failed: {result['error']}")
            if attempt < max_retries - 1:
                time.sleep(1.0)  # Brief pause before retry

    return {"success": False, "error": f"Failed after {max_retries} attempts"}
```

### Replanning Trigger

When action fails after retries, send error feedback to LLM:

```python
def trigger_replan(self, failed_action, error_msg, original_plan):
    replan_prompt = f"""
    Task: {original_plan["task"]}
    Failed action: {failed_action}
    Error: {error_msg}
    Current state: {self.get_robot_state()}

    Generate alternative plan.
    """

    # Call LLM (GPT-4 or Llama)
    new_plan = self.llm_client.replan(replan_prompt)

    # Execute new plan
    self.execute_plan(new_plan)
```

## LangChain for Robotics

**LangChain** is a Python framework for building LLM applications with tools, agents, and memory. It provides abstractions for robotics:

### LangChain Tools (Action Primitives)

Define primitives as LangChain tools:

```python
from langchain.tools import Tool

def navigate_to_tool(location: str) -> str:
    """Navigate robot to named location (kitchen, living_room, etc.)."""
    result = navigate_to_primitive.execute({"location": location})
    return f"Navigated to {location}" if result["success"] else f"Failed: {result['error']}"

def grasp_tool(object_id: str) -> str:
    """Grasp object by ID."""
    result = grasp_primitive.execute({"object_id": object_id})
    return f"Grasped {object_id}" if result["success"] else f"Failed: {result['error']}"

tools = [
    Tool(name="navigate_to", func=navigate_to_tool, description="Navigate to location"),
    Tool(name="grasp", func=grasp_tool, description="Grasp object by ID")
]
```

### LangChain Agent

Create an agent that uses tools:

```python
from langchain.agents import initialize_agent, AgentType
from langchain.chat_models import ChatOpenAI

llm = ChatOpenAI(model="gpt-4-turbo", temperature=0)

agent = initialize_agent(
    tools=tools,
    llm=llm,
    agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
    verbose=True
)

# Execute command
response = agent.run("Go to the kitchen and bring me a cup")
```

**How it works**:
1. Agent receives command: "Go to the kitchen and bring me a cup"
2. Agent reasons: "Need to navigate → use navigate_to tool"
3. Agent calls: `navigate_to("kitchen")`
4. Agent reasons: "Need to find cup → use detect_object tool"
5. Agent calls: `detect_object("cup")`
6. Agent reasons: "Need to grasp → use grasp tool"
7. Agent calls: `grasp("detected_object_id")`
8. Agent returns: "Task complete"

### LangChain Memory (Conversation Context)

For multi-turn interactions:

```python
from langchain.memory import ConversationBufferMemory

memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)

agent = initialize_agent(
    tools=tools,
    llm=llm,
    agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
    memory=memory,
    verbose=True
)

# Turn 1
agent.run("Go to the kitchen")

# Turn 2 (remembers previous command)
agent.run("Now bring me a cup from there")
# Agent knows "there" = kitchen from memory
```

### Advantages of LangChain

1. **Rapid prototyping**: Define tools once, agent figures out how to use them
2. **Built-in error handling**: Agent retries, asks clarifying questions automatically
3. **Memory management**: Multi-turn conversations without manual context tracking

### Disadvantages

1. **Less control**: Agent makes autonomous decisions (might call wrong tools)
2. **Latency**: Each agent reasoning step = 1 LLM call (can be 3–5 calls per command)
3. **Cost**: More LLM API calls = higher cost

**Recommendation**: Use LangChain for rapid prototyping, then optimize with custom action executor for production.

## Example: Full Pipeline (Whisper → LLM → Action Executor)

### Step 1: User Speaks

**User**: "Go to the kitchen and bring me a cup"

### Step 2: Whisper Transcribes

**Output** (to `/transcription` topic):
```
"Go to the kitchen and bring me a cup"
```

### Step 3: LLM Plans

**LLM Planner Node** receives transcription, generates JSON:

```json
{
  "task": "Bring cup from kitchen",
  "steps": [
    {"action": "navigate_to", "args": {"location": "kitchen"}},
    {"action": "detect_object", "args": {"class": "cup"}},
    {"action": "grasp", "args": {"object_id": "$detected_object"}},
    {"action": "navigate_to", "args": {"location": "user"}},
    {"action": "handoff", "args": {"recipient": "user"}}
  ]
}
```

**Published to** `/action_plan`.

### Step 4: Action Executor Executes

**Action Executor Node**:

```python
# Step 1: navigate_to("kitchen")
nav_result = navigate_to_primitive.execute({"location": "kitchen"})
# Success → robot at (5.0, 3.0)

# Step 2: detect_object("cup")
detect_result = detect_object_primitive.execute({"class": "cup"})
# Success → detected_object = "cup_001", pose = (5.2, 3.5, 0.8)

# Step 3: grasp("cup_001")
grasp_result = grasp_primitive.execute({"object_id": "cup_001"})
# Success → gripper closed around cup

# Step 4: navigate_to("user")
nav_result = navigate_to_primitive.execute({"location": "user"})
# Success → robot at (0.0, 0.0)

# Step 5: handoff("user")
handoff_result = handoff_primitive.execute({"recipient": "user"})
# Success → arm extended, gripper opens, user takes cup
```

**Total Time**: 35 seconds (navigation: 15s + 12s, detection: 0.5s, grasp: 8s, handoff: 3s)

### Step 5: Feedback to User

**Text-to-Speech**: "Here's your cup!"

## Performance Optimization

### Parallel Action Execution

Some actions can run in parallel:

**Example**: While navigating, start object detection model warmup.

```python
import concurrent.futures

with concurrent.futures.ThreadPoolExecutor() as executor:
    nav_future = executor.submit(navigate_to_primitive.execute, {"location": "kitchen"})
    warmup_future = executor.submit(object_detector.warmup)

    nav_result = nav_future.result()
    warmup_future.result()  # Ensure detector ready before detect_object
```

### Action Caching

Cache action results for repeated commands:

```python
# User says "Go to the kitchen" 5 times
# First call: 15 seconds (path planning + execution)
# Subsequent calls: Check if already in kitchen → 0 seconds
```

### Predictive Planning

While executing action N, start planning action N+1:

```python
# Execute action 1 (navigate_to kitchen)
# Meanwhile, prefetch object detection model weights
# When action 1 completes, action 2 (detect_object) starts instantly
```

## Summary

This chapter covered **LLM-to-ROS 2 action generation**, the execution layer of VLA systems:

- **Action primitives library**: Standardized robot actions (navigate, detect, grasp, handoff)
- **JSON-to-ROS 2 translation**: Action executor parses LLM plans, dispatches to primitives
- **Variable binding**: Pass data between actions (`$detected_object` from detection to grasp)
- **Error handling**: Timeouts, retries, replanning triggers
- **LangChain for robotics**: Tools, agents, memory for rapid prototyping
- **Example pipeline**: Whisper → LLM → Action Executor → ROS 2 → 35-second task completion

Next chapter: **Capstone—Voice-Controlled Humanoid Autonomy**, integrating all modules (ROS 2, simulation, perception, VLA) into a full autonomous system.

---

**Navigation**
← [Chapter 3: LLMs for Cognitive Planning](/vision-language-action/llms-for-cognitive-planning)
→ [Chapter 5: Capstone—Humanoid Autonomy](/vision-language-action/capstone-humanoid-autonomy)
