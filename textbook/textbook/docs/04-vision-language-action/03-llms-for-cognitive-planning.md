---
title: "Chapter 3: LLMs for Cognitive Planning"
slug: /vision-language-action/llms-for-cognitive-planning
sidebar_label: "3. LLMs for Cognitive Planning"
sidebar_position: 4
toc: true
description: "Leverage large language models (GPT-4, Claude, Llama) for zero-shot task decomposition, chain-of-thought reasoning, and dynamic replanning in humanoid robotics."
---

# Chapter 3: LLMs for Cognitive Planning

## Introduction

Traditional robot task planning uses symbolic AI (PDDL planners, state machines, behavior trees). Given a goal state and available actions, these systems search for a valid action sequence. This works for well-defined problems (e.g., "Move block A onto block B"), but fails for natural language commands like "Make the room more comfortable" (too ambiguous), "Bring me something to drink" (requires common-sense reasoning about drinkable objects), or "Clean up this mess" (ill-defined goal state).

**Large language models (LLMs)** offer a paradigm shift. Trained on billions of tokens of internet text (Wikipedia, books, Reddit, Stack Overflow), LLMs encode vast common-sense knowledge about the world. They can:

1. **Decompose high-level goals** into action primitives ("Make coffee" → grind beans → boil water → pour → serve)
2. **Handle ambiguity** via clarifying questions or reasonable defaults
3. **Replan dynamically** when actions fail ("Cup not found → search other rooms")
4. **Generalize zero-shot** to novel tasks never seen during training

This chapter explores how to use LLMs (GPT-4, Claude 3.5 Sonnet, Llama 3) as **cognitive planners** for humanoid robotics. We'll cover chain-of-thought prompting, task decomposition strategies, handling uncertainty, and trade-offs between cloud APIs (GPT-4) vs. local models (Llama 3 on Nvidia Jetson). By the end, you'll understand how to design LLM-powered planning systems that integrate with ROS 2.

## Why LLMs for Robotics?

### 1. **Zero-Shot Task Decomposition**

**Problem**: Programming every possible task is infeasible. A home assistant robot must handle "Make breakfast," "Do the laundry," "Set the table," etc.—thousands of tasks.

**LLM Solution**: LLMs decompose novel tasks without task-specific training:

**Example**: User says "Make me a sandwich."

**Traditional Planner** (requires pre-programming):
```python
def make_sandwich():
    navigate_to("kitchen")
    grasp("bread")
    place("counter")
    grasp("knife")
    # ... 50 more lines of hard-coded steps
```

**LLM Planner** (zero-shot):
```python
prompt = """
You are a robot task planner. Decompose this command into action primitives:
- navigate_to(location)
- grasp(object)
- place(location)
- use_tool(tool, object)

User command: "Make me a sandwich"
"""

response = llm.complete(prompt)
# LLM Output:
# 1. navigate_to("kitchen")
# 2. grasp("bread")
# 3. place("counter")
# 4. grasp("knife")
# 5. use_tool("knife", "bread")  # Slice bread
# 6. grasp("lettuce")
# 7. place("bread")
# 8. grasp("tomato")
# 9. place("bread")
# 10. grasp("bread_top")
# 11. place("sandwich")
# 12. navigate_to("user")
# 13. handoff("user")
```

**Why it works**: LLMs have seen thousands of sandwich-making tutorials, recipes, and discussions during pretraining. They learn the implicit sequence (bread → spread → toppings → close).

### 2. **Common-Sense Reasoning**

**Problem**: Humans assume implicit constraints ("Bring me coffee" → hot, in a mug, not spilled).

**LLM Solution**: LLMs encode common-sense constraints from pretraining.

**Example**: User says "Pour me water."

**Traditional Planner**: Needs explicit constraints:
```python
assert container.is_clean()
assert container.is_right_side_up()
assert water_temp < 50  # Celsius (safety check)
```

**LLM Planner**: Implicitly reasons:
```
User wants water → need clean cup → fill from tap or bottle → hand to user upright (don't spill)
```

LLM generates:
```json
[
  {"action": "grasp", "args": {"object": "cup", "constraint": "clean"}},
  {"action": "navigate_to", "args": {"location": "sink"}},
  {"action": "use_tool", "args": {"tool": "faucet", "target": "cup"}},
  {"action": "navigate_to", "args": {"location": "user"}},
  {"action": "handoff", "args": {"object": "cup", "orientation": "upright"}}
]
```

**Constraint satisfaction**: LLM adds "constraint: clean" and "orientation: upright" without explicit rules.

### 3. **Handling Ambiguity via Dialogue**

**Problem**: Real-world commands are often underspecified ("Bring me the book" → which book?).

**LLM Solution**: Generate clarifying questions.

**Example**:
```python
user_command = "Bring me the book"

# LLM detects ambiguity
llm_response = llm.plan(user_command, context={"books_visible": 3})

if llm_response["ambiguous"]:
    clarification = llm_response["question"]
    # Output: "I see three books: 'Robotics Handbook', 'Novel', 'Cookbook'. Which one?"
    robot_ask(clarification)
    user_response = wait_for_user_input()  # "The robotics one"
    llm_response = llm.plan(f"Bring me {user_response}", context={...})
```

**Dialogue loop**:
1. User: "Bring me the book"
2. Robot: "Which book? I see three."
3. User: "The robotics one"
4. Robot: Executes grasp("Robotics Handbook")

### 4. **Dynamic Replanning**

**Problem**: Actions fail in the real world (object not found, grasp fails, obstacle blocks path).

**LLM Solution**: Replan based on error feedback.

**Example**: Original plan: grasp("cup_from_table")

**Error**: Object detection fails (cup not on table).

**LLM Replanning**:
```python
error_feedback = "Object 'cup' not detected on table."

replan_prompt = f"""
Original plan failed: {error_feedback}

Alternative strategies:
1. Search other locations (counter, shelf, dishwasher)
2. Ask user for help
3. Abort task

Choose best strategy and generate new plan.
"""

new_plan = llm.complete(replan_prompt)
# Output: "Search counter, then shelf. If still not found, ask user."
```

**Execution**:
```json
[
  {"action": "navigate_to", "args": {"location": "counter"}},
  {"action": "detect_object", "args": {"class": "cup"}},
  {"action": "conditional", "condition": "object_found", "true_branch": "grasp", "false_branch": "search_shelf"}
]
```

### 5. **Multimodal Understanding** (Vision + Language)

Modern LLMs (GPT-4 Vision, Claude 3.5 Sonnet) accept images as input.

**Example**: User says "Bring me the red object."

**Problem**: Which red object? (red cup, red book, red toy)

**Solution**: Pass camera image to LLM:
```python
image = camera.capture()
prompt = "User wants 'the red object'. Which object should I grasp? Output object_id."

response = llm.complete(prompt, images=[image])
# LLM analyzes image, identifies "red cup" as most relevant
# Output: {"object_id": "cup_001", "rationale": "Red cup is drinkware, likely what user needs"}
```

## Chain-of-Thought Prompting for Robotics

**Chain-of-thought (CoT)** prompting encourages LLMs to reason step-by-step before outputting final answers. For robotics, CoT improves plan quality by making the LLM's reasoning explicit.

### Example: Without Chain-of-Thought

```python
prompt = "Decompose: 'Go to the kitchen and bring me a cup'"

response = llm.complete(prompt)
# Output (direct):
# [navigate_to(kitchen), grasp(cup), navigate_to(user)]
```

**Problem**: Missing object detection step (how does robot know where cup is?).

### Example: With Chain-of-Thought

```python
prompt = """
Decompose this command into action primitives. Think step-by-step.

User command: "Go to the kitchen and bring me a cup"

Step-by-step reasoning:
1. What is the goal? Deliver cup to user.
2. Where is the cup? Kitchen (mentioned in command).
3. What actions are needed?
   a. Navigate to kitchen
   b. Detect cup (vision)
   c. Grasp cup
   d. Navigate back to user
   e. Hand cup to user

Action plan (JSON):
"""

response = llm.complete(prompt)
# Output:
# [
#   {"action": "navigate_to", "args": {"location": "kitchen"}},
#   {"action": "detect_object", "args": {"class": "cup"}},
#   {"action": "grasp", "args": {"object_id": "$detected_cup"}},
#   {"action": "navigate_to", "args": {"location": "user_location"}},
#   {"action": "handoff", "args": {"recipient": "user"}}
# ]
```

**Why it works**: Explicit reasoning forces LLM to consider intermediate steps (object detection, user location).

### Few-Shot Prompting (In-Context Learning)

Provide examples in the prompt to guide LLM behavior:

```python
system_prompt = """
You are a robot task planner. Decompose commands into these primitives:
- navigate_to(location)
- detect_object(class, color)
- grasp(object_id)
- place(x, y, z)
- handoff(recipient)

Examples:

User: "Bring me the red box"
Plan:
1. detect_object(class="box", color="red")
2. grasp(object_id="$detected_box")
3. navigate_to(location="user")
4. handoff(recipient="user")

User: "Put the apple on the table"
Plan:
1. detect_object(class="apple")
2. grasp(object_id="$detected_apple")
3. navigate_to(location="table")
4. place(x=1.0, y=0.5, z=0.8)  # Table height

Now decompose:
User: "Go to the kitchen and bring me a cup"
Plan:
"""

response = llm.complete(system_prompt)
```

**Benefits**: Few-shot examples constrain output format (JSON structure, variable naming like `$detected_cup`).

## Task Decomposition: "Bring Me a Cup"

Let's walk through how an LLM decomposes a real command.

### User Command
"Go to the kitchen and bring me a cup"

### LLM Reasoning (Chain-of-Thought)

**Step 1: Parse Intent**
- Goal: Deliver cup to user
- Object: cup
- Location: kitchen
- Recipient: user (implicit "me")

**Step 2: Identify Required Actions**
- Navigation: kitchen (destination) → user (return)
- Perception: detect cup (vision)
- Manipulation: grasp cup
- Handoff: give cup to user

**Step 3: Order Actions**
```
navigate_to(kitchen) → detect_object(cup) → grasp(cup) → navigate_to(user) → handoff(user)
```

**Step 4: Add Failure Handling**
- If cup not detected → search other locations (counter, shelf)
- If grasp fails → retry with different grasp pose
- If user not found → say "I'm ready, where are you?"

### Final JSON Output

```json
{
  "task": "Bring cup from kitchen",
  "steps": [
    {
      "id": 1,
      "action": "navigate_to",
      "args": {"location": "kitchen"},
      "success_condition": "robot_in_kitchen",
      "failure_strategy": "abort"
    },
    {
      "id": 2,
      "action": "detect_object",
      "args": {"class": "cup"},
      "success_condition": "object_detected",
      "failure_strategy": "search_alternative_locations"
    },
    {
      "id": 3,
      "action": "grasp",
      "args": {"object_id": "$detected_cup"},
      "success_condition": "object_grasped",
      "failure_strategy": "retry_different_grasp"
    },
    {
      "id": 4,
      "action": "navigate_to",
      "args": {"location": "user_location"},
      "success_condition": "near_user",
      "failure_strategy": "announce_location"
    },
    {
      "id": 5,
      "action": "handoff",
      "args": {"recipient": "user"},
      "success_condition": "user_has_cup",
      "failure_strategy": "place_on_surface"
    }
  ],
  "estimated_duration_seconds": 35,
  "risk_factors": ["cup_fragile", "user_location_dynamic"]
}
```

### Key Features
- **Variable binding**: `$detected_cup` (output of step 2 feeds into step 3)
- **Failure strategies**: Each step has fallback behavior
- **Success conditions**: Measurable checks (used by action executor)

## LLM Options for Robotics

### 1. Cloud APIs (GPT-4, Claude)

**Pros**:
- **Best accuracy**: GPT-4 Turbo (0125) has ~85% planning accuracy on robotics benchmarks
- **No local compute**: Offload to OpenAI/Anthropic servers
- **Regular updates**: Models improve over time (GPT-4 → GPT-4 Turbo → GPT-5)

**Cons**:
- **Latency**: 1–3 seconds per API call (network + inference)
- **Cost**: $0.01–0.03 per request (GPT-4 Turbo: $0.01/1K input tokens)
- **Privacy**: Sending robot sensor data to cloud (HIPAA/GDPR concerns)
- **Dependency**: Requires internet (fails in offline environments)

**Example: GPT-4 Integration**
```python
import openai

openai.api_key = "sk-..."

def plan_task(user_command):
    response = openai.ChatCompletion.create(
        model="gpt-4-turbo-preview",
        messages=[
            {"role": "system", "content": ROBOTICS_SYSTEM_PROMPT},
            {"role": "user", "content": f"Decompose: {user_command}"}
        ],
        temperature=0.0,  # Deterministic
        response_format={"type": "json_object"}  # Force JSON output
    )

    return json.loads(response.choices[0].message.content)
```

**Cost Analysis**:
- Average command: 500 input tokens, 200 output tokens
- GPT-4 Turbo: $0.01/1K input + $0.03/1K output = $0.011 per command
- 1000 commands/day = $11/day = $330/month per robot

### 2. Local Models (Llama 3, Mistral)

**Pros**:
- **Low latency**: 100–300ms on Nvidia Jetson Orin (vs. 1–3s for cloud)
- **No cost**: Free inference after initial setup
- **Privacy**: All data stays on device
- **Offline**: Works without internet

**Cons**:
- **Lower accuracy**: Llama 3 70B ≈ 75% planning accuracy (vs. GPT-4's 85%)
- **Compute requirements**: Llama 3 70B needs 40 GB VRAM (requires A100 or 2× RTX 4090)
- **Deployment complexity**: Model quantization, inference optimization (TensorRT, vLLM)

**Example: Llama 3 70B Integration (via Ollama)**
```bash
# Install Ollama (local LLM inference server)
curl -fsSL https://ollama.com/install.sh | sh

# Download Llama 3 70B (quantized to 4-bit, ~40 GB)
ollama pull llama3:70b
```

```python
import requests

def plan_task_local(user_command):
    response = requests.post("http://localhost:11434/api/generate", json={
        "model": "llama3:70b",
        "prompt": f"{ROBOTICS_SYSTEM_PROMPT}\n\nUser: {user_command}\nPlan:",
        "stream": False
    })

    return json.loads(response.json()["response"])
```

**Latency Benchmark**:
| Model           | Hardware       | Latency | Accuracy |
|-----------------|----------------|---------|----------|
| GPT-4 Turbo     | Cloud          | 1.8s    | 85%      |
| Claude 3 Opus   | Cloud          | 2.3s    | 87%      |
| Llama 3 70B     | A100 (80GB)    | 0.3s    | 75%      |
| Llama 3 8B      | Jetson Orin    | 0.15s   | 62%      |

**Recommendation**:
- **Tethered robots** (connected to server): Use Llama 3 70B on local server (best latency + privacy)
- **Mobile robots** (battery-powered): Use GPT-4 API (avoid carrying A100 on robot!)
- **Offline requirements**: Use Llama 3 8B on Jetson (acceptable for simple commands)

### 3. Hybrid Approach (Best of Both)

Use **local model for common commands**, cloud for complex/ambiguous tasks:

```python
def plan_task_hybrid(user_command):
    # Try local model first (fast)
    local_plan = plan_task_local(user_command)

    # Check confidence
    if local_plan["confidence"] > 0.9:
        return local_plan
    else:
        # Fallback to cloud (higher accuracy)
        return plan_task_cloud(user_command)
```

**Example**: 80% of commands are simple ("Go to kitchen," "Grasp cup") → local model handles these in 0.3s. 20% are complex ("Make the room comfortable") → fallback to GPT-4.

## Handling Uncertainty and Partial Information

Real-world commands often lack complete information:

### Case 1: Spatial References Without Coordinates

**User**: "Bring me the cup on the table"
**Problem**: Which table? (living room table, kitchen table, coffee table)

**LLM Solution**: Make reasonable assumption or ask:
```python
# LLM reasoning:
# "on the table" → likely nearest table in current room
# Or: ask user if multiple tables visible

plan = [
    {"action": "detect_object", "args": {"class": "table"}},
    {"action": "conditional", "condition": "num_tables > 1", "true": "ask_user", "false": "use_nearest"}
]
```

### Case 2: Object Attributes Without Exact Specification

**User**: "Bring me something to drink"
**Problem**: Many drinkable liquids (water, juice, soda, coffee)

**LLM Solution**: Use common-sense defaults + ask if ambiguous:
```python
# LLM reasoning:
# "something to drink" → default to water (most common, neutral)
# If specific beverages visible, ask user preference

plan = [
    {"action": "detect_object", "args": {"class": ["water_bottle", "juice", "soda"]}},
    {"action": "conditional", "condition": "num_options > 1", "true": "ask_preference", "false": "grasp_default"}
]
```

### Case 3: Temporal Constraints

**User**: "Remind me to take medication in 30 minutes"
**Problem**: Not immediate action; need scheduling

**LLM Solution**: Generate timer + delayed action:
```json
{
  "action": "schedule",
  "args": {
    "delay_seconds": 1800,
    "callback": {
      "action": "speak",
      "args": {"text": "Time to take your medication"}
    }
  }
}
```

## Error Recovery: Replanning When Actions Fail

### Failure Scenario: Object Not Found

**Original Plan**:
```json
[
  {"action": "navigate_to", "args": {"location": "kitchen"}},
  {"action": "detect_object", "args": {"class": "cup"}},
  {"action": "grasp", "args": {"object_id": "$detected_cup"}}
]
```

**Execution**: Step 2 fails (no cup detected in kitchen).

**LLM Replanning Prompt**:
```python
replan_prompt = f"""
Task: Bring cup to user
Current state: In kitchen, cup not detected
Error: Object detection failed for 'cup'

Possible strategies:
1. Search other locations (counter, shelf, dining room)
2. Ask user where cup is
3. Use alternative object (glass, bottle)
4. Abort task

Choose best strategy and generate new plan.
"""

new_plan = llm.complete(replan_prompt)
```

**LLM Output**:
```json
{
  "strategy": "search_alternative_locations",
  "reasoning": "Cup might be on counter or shelf. Search before asking user.",
  "new_plan": [
    {"action": "navigate_to", "args": {"location": "counter"}},
    {"action": "detect_object", "args": {"class": "cup"}},
    {"action": "conditional", "condition": "object_found", "true": "grasp", "false": "search_shelf"}
  ]
}
```

### Failure Scenario: Grasp Fails

**Error**: Gripper fails to secure object (slips).

**Replanning**:
```python
replan_prompt = """
Task: Grasp cup
Error: Gripper failed to secure object (cup slipped)

Possible causes:
1. Grasp pose incorrect (angle, position)
2. Gripper force too weak
3. Object too slippery

Strategies:
1. Retry with different grasp pose
2. Increase gripper force
3. Use two-handed grasp
4. Ask user for help

Choose strategy and generate new plan.
"""
```

**LLM Output**:
```json
{
  "strategy": "retry_different_grasp",
  "reasoning": "First grasp was from side; try top-down grasp for better stability.",
  "new_plan": [
    {"action": "grasp", "args": {"object_id": "cup_001", "approach": "top_down", "force": 15}}
  ]
}
```

## Prompt Engineering for Robotics

### System Prompt Template

```python
ROBOTICS_SYSTEM_PROMPT = """
You are a task planner for a humanoid robot with these capabilities:

**Sensors**:
- RGB-D cameras (640×480, 30 FPS)
- LiDAR (10m range)
- Microphone array (voice input)

**Actuators**:
- Mobile base (differential drive, max speed 1.5 m/s)
- 7-DOF arm with parallel gripper (max payload 2 kg)
- Head pan-tilt (for camera orientation)

**Action Primitives**:
1. navigate_to(location: str) - Move to named location (e.g., "kitchen")
2. navigate_to_pose(x: float, y: float, theta: float) - Move to coordinates
3. detect_object(class: str, color: str = None) - Returns object_id and pose
4. grasp(object_id: str, approach: str = "side") - Grasp object
5. place(x: float, y: float, z: float) - Place held object
6. handoff(recipient: str) - Hand object to person
7. speak(text: str) - Text-to-speech output
8. ask_user(question: str) - Get user input via voice

**Constraints**:
- Max payload: 2 kg (reject heavy objects)
- Workspace: 0.5m radius around base
- Navigation: Requires 0.8m clearance (door width)

**Output Format**:
Generate JSON action sequence:
{
  "task": "Brief description",
  "steps": [
    {"action": "action_name", "args": {...}, "failure_strategy": "..."},
    ...
  ]
}

**Failure Strategies**:
- abort: Stop task, report error
- retry: Try same action again (max 3 times)
- replan: Generate alternative approach
- ask_user: Request human intervention

Now decompose the user's command.
"""
```

### Enforcing Output Format (JSON Schema)

Use OpenAI's function calling or JSON mode:

```python
response = openai.ChatCompletion.create(
    model="gpt-4-turbo-preview",
    messages=[...],
    response_format={"type": "json_object"},  # Force JSON
    functions=[{
        "name": "generate_robot_plan",
        "parameters": {
            "type": "object",
            "properties": {
                "task": {"type": "string"},
                "steps": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string"},
                            "args": {"type": "object"},
                            "failure_strategy": {"type": "string"}
                        },
                        "required": ["action", "args"]
                    }
                }
            },
            "required": ["task", "steps"]
        }
    }]
)
```

## Summary

This chapter explored **LLMs for cognitive planning** in humanoid robotics:

- **Why LLMs?** Zero-shot task decomposition, common-sense reasoning, ambiguity handling, dynamic replanning
- **Chain-of-thought prompting**: Explicit step-by-step reasoning improves plan quality
- **Task decomposition**: "Bring me a cup" → 5-step action sequence with failure strategies
- **LLM options**: Cloud (GPT-4, Claude) for accuracy; Local (Llama 3) for latency/privacy
- **Uncertainty handling**: Make reasonable assumptions, ask clarifying questions
- **Error recovery**: Replan when actions fail (object not found, grasp fails)
- **Prompt engineering**: System prompts define robot capabilities, output format, constraints

Next chapter: **LLM-to-ROS 2 action generation**, where we convert JSON plans into executable ROS 2 action server calls.

---

**Navigation**
← [Chapter 2: Whisper Speech Recognition](/vision-language-action/whisper-speech-recognition)
→ [Chapter 4: LLM-to-ROS 2 Action Generation](/vision-language-action/llm-to-ros2-action-generation)
