---
title: "Module 4: Instructor Solutions"
sidebar_label: "Instructor Solutions"
description: "Complete solutions, grading rubrics, and common mistakes for Module 4 exercises (Vision-Language-Action Systems)"
---

# Module 4: Instructor Solutions (Vision-Language-Action Systems)

**Note**: This file is intended for instructors only and should be excluded from student-facing materials.

---

## 📘 Recall Exercises

### Exercise 1: VLA Architecture Comparison

**Difficulty**: Recall | **Time**: 30 min

#### Complete Solution

**Comparison Table**:

| **Dimension** | **Pipeline Architecture** | **End-to-End Architecture** | **Hybrid Architecture** |
|---------------|---------------------------|----------------------------|-------------------------|
| **Data Flow** | Vision → features → LLM → text plan → action mapping → robot | Raw sensor data → neural network → motor commands (direct) | Vision → LLM planning → learned policy → robot (combined) |
| **Latency** | High (500-3000ms): sequential processing through multiple stages | Low (10-100ms): single forward pass | Medium (200-800ms): parallel LLM + policy execution |
| **Sample Efficiency** | High: leverages pre-trained vision models, LLMs (minimal robot data needed) | Low: requires 10K-1M robot demonstrations for end-to-end learning | Medium: pre-trained LLM + RL policy (1K-10K demos) |
| **Interpretability** | High: explicit text plans, action primitives traceable | Very low: black-box neural network (hard to debug) | Medium: LLM plans are interpretable, policy is learned |
| **Generalization** | High: LLM generalizes to novel tasks via language | Low: limited to training distribution (must retrain for new tasks) | High: LLM provides zero-shot generalization, policy adapts |
| **Hardware Requirements** | CPU sufficient for LLM API (cloud); GPU optional for vision | GPU required for real-time inference (RTX 3090+) | GPU for vision + policy; LLM can be cloud |

**Advantages and Disadvantages**:

**Pipeline Architecture**:
- ✅ Advantages:
  1. Modularity: Easy to debug, swap components (e.g., change LLM without retraining vision)
  2. Sample efficiency: Leverages internet-scale pre-trained models (GPT-4, CLIP)
- ❌ Disadvantages:
  1. Latency: Sequential processing through vision → text → action (500-3000ms total)
  2. Error propagation: Vision errors cascade through LLM to wrong actions

**End-to-End Architecture (RT-1, RT-2)**:
- ✅ Advantages:
  1. Low latency: Single neural network forward pass (10-100ms)
  2. Optimized for task: Network learns task-specific features (not generic vision)
- ❌ Disadvantages:
  1. Data-hungry: Requires 100K+ robot demonstrations for robust performance
  2. Limited generalization: Struggles with tasks not in training distribution (requires fine-tuning)

**Hybrid Architecture (SayCan)**:
- ✅ Advantages:
  1. Combines LLM's language understanding with learned policy's robustness
  2. Zero-shot generalization: LLM handles novel commands, policy executes reliably
- ❌ Disadvantages:
  1. Complexity: Requires designing interface between LLM and policy (value function scoring)
  2. Policy training: Still needs RL to train low-level policies for action primitives

**Scenario Matching**:

1. **Pipeline Architecture**: **Warehouse inventory robot** ("Count boxes in aisle B3, report status")
   - Why: Task requires language reporting (LLM generates natural language summaries), flexibility to handle novel counting tasks without retraining.

2. **End-to-End Architecture**: **High-speed bin picking** (pick 1,000 objects/hour)
   - Why: Latency-critical (need <100ms per action), task is narrow and well-defined (extensive demos available from similar factories).

3. **Hybrid Architecture**: **Household service robot** ("Make me breakfast")
   - Why: Requires both high-level planning (LLM decomposes into fry eggs, toast bread) and robust low-level manipulation (learned grasping, pouring policies).

**Primary Failure Modes**:

- **Pipeline**: Vision misidentifies object → LLM plans with wrong object → wrong action (e.g., "grasp red cup" but vision sees blue cup as red).
- **End-to-End**: Out-of-distribution input (e.g., trained on tables, deployed in kitchen) → unpredictable outputs (random actions).
- **Hybrid**: LLM-policy interface failure (e.g., LLM plans "pour milk" but no learned policy for pouring) → execution stalls.

#### Grading Rubric (Total: 20 points)
- **Table completeness** (5 pts): All 6 dimensions accurately filled for 3 architectures
- **Advantages/Disadvantages** (5 pts): 2 advantages + 2 disadvantages per architecture, technically accurate
- **Scenario matching** (5 pts): Appropriate use cases with clear justifications (not vague)
- **Failure mode analysis** (3 pts): Realistic failure modes specific to each architecture
- **Diagram quality** (2 pts): Clear information flow diagrams (hand-drawn or digital acceptable)

#### Common Mistakes
- Confusing end-to-end with pipeline (thinking "end-to-end" means vision → LLM → action)
- Claiming one architecture is "always better" (ignoring trade-offs)
- Vague scenario matching ("works for robotics"—not specific enough)
- Missing data flow diagrams (text-only answers)

---

### Exercise 2: Whisper Accuracy Metrics

**Difficulty**: Recall | **Time**: 25 min

#### Complete Solution

**1. Word Error Rate (WER)**

**Definition**: Percentage of words in the transcription that are substituted, deleted, or inserted compared to ground truth.

**Formula**:
```
WER = (S + D + I) / N
```
Where:
- S = Substitutions (wrong word)
- D = Deletions (missing word)
- I = Insertions (extra word)
- N = Total words in ground truth

**Interpretation**: Lower is better (0% = perfect, 100% = completely wrong). WER > 30% is generally unusable for robotics commands.

**Example Calculation**:
- Ground truth: "Navigate to the kitchen and open the fridge"
- Hypothesis: "Navigate to the chicken and open fridge"
- S=1 ("kitchen" → "chicken"), D=1 ("the" missing), I=0
- N=8 words
- WER = (1 + 1 + 0) / 8 = 25%

**2. Character Error Rate (CER)**

**Definition**: Similar to WER but operates at character level (useful for languages without clear word boundaries or acronyms).

**Formula**:
```
CER = (S_char + D_char + I_char) / N_char
```

**When CER is more useful**:
- **Technical domains**: Robotics commands with model numbers ("RT-X-2-b"), URLs, or serial codes where character-level accuracy matters.
- **Languages without spaces**: Chinese, Japanese (word segmentation is ambiguous).
- **Acronyms and codes**: "ROS2" vs. "ROS 2" (WER counts as error, CER more forgiving).

**3. Confidence Scores**

**How Whisper computes confidence**:
- Whisper outputs log-probabilities for each token from the decoder.
- Confidence = exp(mean(log_probs)) for the entire sequence.
- Range: 0.0 (no confidence) to 1.0 (perfect confidence).

**Threshold selection**:
- **Conservative (robotics)**: Threshold = 0.8 (reject low-confidence transcriptions to avoid dangerous misinterpretations).
- **Balanced**: Threshold = 0.6 (accept most transcriptions, log low-confidence ones for human review).
- **Permissive**: Threshold = 0.4 (accept all but clearly garbled audio).

For robotics, **0.7-0.8** is recommended to balance usability and safety.

**4. Real-Time Factor (RTF)**

**Definition**: Ratio of processing time to audio duration.

**Formula**:
```
RTF = Processing Time / Audio Duration
```

**Why RTF < 1.0 matters**:
- RTF < 1.0 means **real-time processing** (can transcribe audio as fast or faster than it's spoken).
- RTF > 1.0 means **lagging** (audio buffers up, latency increases indefinitely).
- Example: Whisper `base` model achieves RTF = 0.3 on GPU (processes 10s audio in 3s → real-time).

For VLA systems, RTF < 0.5 is ideal (allows headroom for other processing: LLM planning, action execution).

**Impact of environmental factors**:

| **Factor** | **Impact on WER** | **Impact on CER** | **Impact on Confidence** |
|------------|-------------------|-------------------|--------------------------|
| **Background noise** (60+ dB) | +15-30% (words masked by noise) | +10-20% (consonants lost) | -0.2 to -0.3 (lower confidence) |
| **Domain-specific vocabulary** (e.g., "Nav2", "MoveIt") | +10-20% (Whisper trained on general English, not robotics jargon) | +5-10% (acronyms misspelled) | -0.1 to -0.2 (out-of-vocabulary words) |
| **Speaker accent** (non-native English) | +5-15% (phoneme variations) | +3-8% (pronunciation differences) | -0.1 to -0.15 (accent patterns rare in training) |

**Mitigation Strategies**:

1. **Noise filtering preprocessing**:
   - Apply bandpass filter (300-3400 Hz, speech frequency range).
   - Use Noise Suppression (RNNoise, Krisp) before Whisper.
   - Expected improvement: -5 to -10% WER in noisy environments.

2. **Fine-tuning on domain data**:
   - Collect 10-50 hours of robotics command audio (transcribed).
   - Fine-tune Whisper `small` or `base` on robotics vocabulary.
   - Expected improvement: -10 to -15% WER for domain-specific terms.

3. **Adaptive confidence thresholding**:
   - Measure background noise (dB), adjust confidence threshold dynamically.
   - In quiet (< 40 dB): threshold = 0.6; in noisy (> 60 dB): threshold = 0.85.
   - Reduces false accepts from garbled audio.

**Whisper Model Performance Table**:

| **Model** | **Parameters** | **WER (LibriSpeech)** | **RTF (GPU)** | **RTF (CPU)** | **Use Case** |
|-----------|----------------|-----------------------|---------------|---------------|--------------|
| **tiny** | 39M | 7.5% | 0.1 | 1.5 | Embedded (Jetson Nano), latency-critical |
| **base** | 74M | 5.5% | 0.3 | 3.2 | Real-time on GPU, good accuracy/speed |
| **small** | 244M | 4.5% | 0.8 | 8.5 | Balanced (cloud GPU or edge RTX) |
| **medium** | 769M | 3.8% | 1.5 | 18.0 | High-accuracy (not real-time on CPU) |
| **large** | 1550M | 3.0% | 2.8 | 35.0 | Offline transcription (batch processing) |

**Recommendation for robotics**: `base` for edge deployment (Jetson), `small` for cloud/workstation GPU.

#### Grading Rubric (Total: 15 points)
- **Metric definitions** (5 pts): Accurate formulas for WER, CER, confidence, RTF
- **Example calculations** (4 pts): Correct WER/CER calculations with shown work
- **Performance table** (3 pts): Accurate Whisper model comparison (within 20% of published benchmarks)
- **Mitigation strategies** (2 pts): 2-3 realistic strategies with expected impact (not vague)
- **Technical writing** (1 pt): Clear explanations, proper units (dB, %, ms)

#### Common Mistakes
- Confusing WER and accuracy (WER is error rate, higher = worse)
- Claiming RTF > 1.0 is acceptable for real-time (it's not—causes unbounded latency)
- Unrealistic WER improvement claims (e.g., "fine-tuning reduces WER from 20% to 2%"—too optimistic)
- Missing units (dB, ms, %)

---

### Exercise 3: LLM Prompting Strategies for Task Decomposition

**Difficulty**: Recall | **Time**: 35 min

#### Complete Solution

**1. Zero-Shot Prompting**

**Example Prompt**:
```
You are a household robot with the following action primitives: navigate(location), detect_object(object_type), grasp(object_id), pour(source, destination), heat(appliance, duration). Decompose this task into a valid action sequence:

Task: "Make me a cup of coffee"

Output the plan as a numbered JSON list.
```

**When most effective**:
- Pre-trained LLM has strong common-sense knowledge of the task domain (e.g., cooking, household chores).
- Task is standard and well-represented in internet text (coffee-making tutorials are common).

**Potential failure modes**:
- **Hallucinated actions**: LLM invents actions not in the primitive set (e.g., "brew(coffee_machine)" instead of using `heat`).
- **Missing prerequisites**: Assumes coffee grounds are already in the machine (doesn't plan to detect/grasp them).
- **Incorrect ordering**: Pours water before heating (physically invalid).

**Latency and compute**:
- Latency: ~500-1500ms (depends on LLM size, provider: GPT-4 = 1-2s, Llama-2-7B local = 300-500ms).
- No additional prompt engineering effort (minimal tokens).

**2. Few-Shot Prompting**

**Example Prompt**:
```
You are a household robot. Here are examples of task decompositions:

Example 1:
Task: "Bring me water"
Plan:
1. navigate(location="kitchen")
2. detect_object(object_type="water_bottle")
3. grasp(object_id="$detected_object")
4. navigate(location="user")
5. place(location="table")

Example 2:
Task: "Clean the table"
Plan:
1. navigate(location="table")
2. detect_object(object_type="dirty_dish")
3. grasp(object_id="$detected_object")
4. navigate(location="sink")
5. place(location="sink")
6. navigate(location="table")
7. repeat(steps=2-6, condition="no_dirty_dishes")

Example 3:
Task: "Turn on the light"
Plan:
1. navigate(location="light_switch")
2. press(object_id="switch_button")

Now decompose this task:
Task: "Make me a cup of coffee"
Plan:
```

**When most effective**:
- Task requires specific formatting (JSON, numbered lists) or uses domain-specific primitives.
- LLM needs guidance on abstraction level (e.g., how granular should actions be?).
- 3-5 examples cover most task variations.

**Potential failure modes**:
- **Overfitting to examples**: LLM copies example structure too literally (e.g., always navigates to kitchen first, even if already there).
- **Incomplete coverage**: If no example shows "heating," LLM might skip heating step.

**Latency and compute**:
- Latency: +200-500ms over zero-shot (longer prompts = more tokens to process).
- Prompt engineering effort: Medium (must craft 3-5 representative examples).

**3. Chain-of-Thought (CoT) Prompting**

**Example Prompt**:
```
You are a household robot. Before generating an action plan, think step-by-step about:
1. What objects are needed?
2. Where are they likely located?
3. What are the preconditions for each action?
4. What could go wrong?

Task: "Make me a cup of coffee"

Reasoning:
- Objects needed: coffee grounds, water, mug, coffee machine
- Locations: coffee grounds (pantry), water (fridge/sink), mug (cabinet), coffee machine (kitchen counter)
- Preconditions: Coffee machine must be on, mug must be under spout
- Risks: Coffee grounds might be empty, machine might be broken

Plan:
1. navigate(location="kitchen")
2. detect_object(object_type="coffee_machine")
3. check_status(object_id="coffee_machine") # If off, turn on
4. detect_object(object_type="coffee_mug")
5. grasp(object_id="$mug")
6. place(location="machine_spout")
7. detect_object(object_type="coffee_grounds")
8. grasp(object_id="$coffee_grounds")
9. pour(source="$coffee_grounds", destination="machine_filter")
10. press(object_id="brew_button")
11. wait(duration=60) # Brewing time
12. grasp(object_id="$mug")
13. navigate(location="user")
14. place(location="table")
```

**When most effective**:
- Complex tasks with multi-step dependencies (order matters).
- Need to handle edge cases (missing ingredients, appliance failures).
- LLM's reasoning improves plan quality (GPT-4 benefits more than smaller models).

**Potential failure modes**:
- **Over-thinking**: LLM generates excessive reasoning (200+ tokens) without improving plan quality (adds latency).
- **Reasoning-action mismatch**: Reasoning identifies a problem (e.g., "coffee might be empty") but plan doesn't handle it (no check for emptiness).

**Latency and compute**:
- Latency: +500-1000ms over zero-shot (must generate reasoning + plan).
- Prompt engineering effort: High (must design reasoning structure, teach LLM what to consider).

**Comparison Table**:

| **Strategy** | **Accuracy** | **Robustness to Novel Tasks** | **Prompt Engineering Effort** | **Inference Latency** |
|--------------|--------------|-------------------------------|-------------------------------|----------------------|
| **Zero-Shot** | 60-70% (baseline) | High (generalizes well) | Low (minimal prompt) | ~500-1500ms (GPT-4) |
| **Few-Shot** | 75-85% (improved) | Medium (limited to example patterns) | Medium (3-5 examples needed) | ~700-2000ms (longer prompts) |
| **Chain-of-Thought** | 80-90% (best) | High (reasoning generalizes) | High (complex prompt design) | ~1000-2500ms (reasoning overhead) |

**Recommendations**:

- **Use Zero-Shot**: Rapid prototyping, simple tasks (1-3 steps), latency-critical (< 1s total).
- **Use Few-Shot**: Domain-specific formatting, need consistent output structure, 5-8 step tasks.
- **Use CoT**: Safety-critical tasks (need to reason about failures), complex multi-step tasks (10+ actions).

**Python Implementation Snippet** (OpenAI API):

```python
import openai

def zero_shot_plan(task: str) -> list:
    prompt = f"""You are a household robot with primitives: navigate, detect_object, grasp, place.
    Task: {task}
    Output JSON list of actions."""
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.2  # Lower temperature for more consistent plans
    )
    return parse_json(response.choices[0].message.content)

def few_shot_plan(task: str) -> list:
    examples = """Example 1: Task="Bring water" → [navigate("kitchen"), detect_object("bottle"), grasp("$detected"), navigate("user"), place("table")]"""
    prompt = f"{examples}\nTask: {task}\nPlan:"
    # ... (similar OpenAI API call)

def cot_plan(task: str) -> list:
    prompt = f"""Think step-by-step:
    1. What objects are needed?
    2. What are the preconditions?

    Task: {task}
    Reasoning:"""
    # ... (similar OpenAI API call, parse reasoning + plan)
```

#### Grading Rubric (Total: 15 points)
- **Prompt quality** (5 pts): All 3 prompts are well-structured, elicit valid action sequences
- **Strategy understanding** (4 pts): Accurate description of when/why each strategy works
- **Failure mode analysis** (3 pts): Realistic failure modes with concrete examples (not vague)
- **Trade-off analysis** (2 pts): Balanced comparison (not claiming one is always best)
- **Code implementation** (1 pt): Working Python code demonstrating each strategy

#### Common Mistakes
- Prompts that don't specify output format (LLM returns prose instead of JSON)
- Confusing few-shot with fine-tuning (few-shot is in-context, fine-tuning updates model weights)
- Claiming CoT is always better (ignores latency trade-off)
- Missing code examples (text-only answers)

---

### Exercise 4: Action Primitive Design Principles

**Difficulty**: Recall | **Time**: 40 min

#### Complete Solution

**Action Primitive Library (12+ primitives)**:

| **Primitive** | **Parameters** | **Preconditions** | **Postconditions** | **Duration** |
|---------------|----------------|-------------------|-------------------|--------------|
| `navigate(target: Pose)` | target: Pose (x, y, yaw) | Map loaded, localization active | Robot at target pose (tolerance ±5cm, ±5°) | 5-30s (depends on distance) |
| `grasp(object_id: str, approach: GraspApproach)` | object_id (from detection), approach (top/side/front) | Object detected, arm can reach, gripper open | Object in gripper (force sensor confirms contact) | 3-8s |
| `place(location: Pose OR str)` | location (pose or semantic: "table") | Object in gripper, location clear | Object at location, gripper open | 2-5s |
| `detect_object(object_type: str, store_as: str)` | object_type (class name), store_as (variable name) | Camera active, object in view | Variable `$store_as` set to object_id | 0.5-2s |
| `open_drawer(handle_id: str)` | handle_id (from detection) | Handle detected, arm can reach | Drawer open (>50% extension) | 4-10s |
| `close_drawer(handle_id: str)` | handle_id | Drawer open, arm can reach | Drawer closed (<10% extension) | 3-8s |
| `pour(source: str, destination: str, volume: float)` | source/destination (object IDs), volume (liters) | Source in gripper, destination detected, tilt possible | `volume` liters transferred (weight sensor confirms) | 5-15s |
| `press(button_id: str)` | button_id (from detection) | Button detected, arm can reach | Button pressed (force sensor confirms) | 1-3s |
| `wait(duration: float)` | duration (seconds) | None | `duration` seconds elapsed | Variable (as specified) |
| `say(message: str)` | message (text to speak) | TTS system active | Audio played, message logged | 2-10s (depends on length) |
| `scan_area(region: BoundingBox)` | region (3D bounding box to search) | Camera active, arm can move | Objects in `region` detected, stored in `$scan_results` | 5-20s (depends on area) |
| `check_status(device_id: str, property: str)` | device_id (appliance), property ("on", "temperature") | Device has smart interface (API/sensor) | Variable `$device_id.property` set | 0.2-1s |

**Detailed Design Rationale (3 primitives)**:

**1. `grasp(object_id: str, approach: GraspApproach)` → GraspResult**

**Abstraction Level**:
- **Not too high-level**: Doesn't abstract away grasp approach (top vs. side matters for physics—can't top-grasp a plate on a table).
- **Not too low-level**: Doesn't expose joint angles or force control (those are handled by MoveIt motion planning + gripper controller).
- **Just right**: Exposes semantic parameters (object ID, approach direction) that an LLM can reason about.

**Timeout and Retry Logic**:
```python
def grasp(object_id: str, approach: GraspApproach, timeout: float = 8.0, max_retries: int = 3) -> GraspResult:
    for attempt in range(max_retries):
        result = execute_grasp(object_id, approach, timeout=timeout)
        if result == GraspResult.SUCCESS:
            return result
        elif result == GraspResult.OBJECT_MOVED:
            # Object shifted, re-detect and retry
            new_pose = detect_object_pose(object_id)
            if new_pose is None:
                return GraspResult.OBJECT_LOST
            continue  # Retry with updated pose
        elif result == GraspResult.COLLISION:
            # Arm hit obstacle, cannot retry safely
            return GraspResult.FAILED_COLLISION
        elif result == GraspResult.TIMEOUT:
            # Motion planning took too long, possibly unreachable
            if attempt < max_retries - 1:
                time.sleep(1.0)  # Wait for environment to settle
                continue
            return GraspResult.FAILED_TIMEOUT
    return GraspResult.FAILED_MAX_RETRIES
```

**Failure Modes and Detection**:
- **Object moved**: Force sensor shows no contact after closing gripper → re-detect object pose.
- **Collision during approach**: Joint torques exceed threshold → abort, report collision.
- **Unreachable pose**: Motion planning fails for 5 seconds → timeout, report unreachable.
- **Gripper malfunction**: Gripper doesn't close (encoder shows no movement) → return hardware error.

**2. `navigate(target: Pose)` → NavigationResult**

**Abstraction Level**:
- **Not too high-level**: Doesn't abstract away target pose (LLM needs control over exact position for tasks like "align with table edge").
- **Not too low-level**: Doesn't expose path waypoints or velocity profiles (Nav2 handles those).
- **Just right**: Exposes goal pose, hides path planning and local obstacle avoidance.

**Timeout and Retry Logic**:
```python
def navigate(target: Pose, timeout: float = 30.0, max_retries: int = 2) -> NavigationResult:
    for attempt in range(max_retries):
        result = nav2_navigate_to_pose(target, timeout=timeout)
        if result == NavigationResult.SUCCESS:
            return result
        elif result == NavigationResult.BLOCKED:
            # Path blocked by dynamic obstacle (e.g., person)
            if attempt < max_retries - 1:
                wait_for_obstacle_clear(timeout=10.0)  # Wait for person to move
                continue
            return NavigationResult.FAILED_BLOCKED
        elif result == NavigationResult.NO_PATH:
            # Global planner failed (target unreachable or not in map)
            return NavigationResult.FAILED_NO_PATH  # Don't retry (won't fix itself)
        elif result == NavigationResult.TIMEOUT:
            # Navigation took too long (robot stuck?)
            return NavigationResult.FAILED_TIMEOUT
    return NavigationResult.FAILED_MAX_RETRIES
```

**Failure Modes and Detection**:
- **Dynamic obstacle**: Costmap shows obstacle in path → pause, wait for clearance (10s timeout), retry.
- **Localization lost**: AMCL particle divergence > threshold → stop, request relocalization (AprilTag scan).
- **Stuck (wheels slipping)**: Odometry vs. expected position mismatch > 20cm → stop, return stuck error.
- **Goal unreachable**: Global planner fails after 5s → return no-path error (don't retry—won't help).

**3. `pour(source: str, destination: str, volume: float)` → PourResult**

**Abstraction Level**:
- **Not too high-level**: Exposes volume parameter (LLM can specify "pour 200ml" for precision).
- **Not too low-level**: Doesn't expose tilt angle, pour rate (learned policy or controller handles that).
- **Just right**: Semantic parameters (what to pour, where, how much) that map to task goals.

**Timeout and Retry Logic**:
```python
def pour(source: str, destination: str, volume: float, timeout: float = 15.0, max_retries: int = 2) -> PourResult:
    # Measure initial weight of destination
    initial_weight = get_weight(destination)

    for attempt in range(max_retries):
        result = execute_pour_motion(source, destination, volume, timeout=timeout)

        # Check if volume transferred matches target
        final_weight = get_weight(destination)
        poured_volume = (final_weight - initial_weight) / LIQUID_DENSITY

        if abs(poured_volume - volume) < 0.02:  # Within 20ml tolerance
            return PourResult.SUCCESS
        elif poured_volume < volume * 0.5:
            # Barely any liquid poured (source might be empty)
            return PourResult.FAILED_SOURCE_EMPTY
        elif poured_volume < volume:
            # Poured less than requested, retry to top off
            remaining = volume - poured_volume
            if attempt < max_retries - 1:
                pour(source, destination, remaining, timeout, max_retries=1)  # One more attempt
            return PourResult.PARTIAL_SUCCESS
        else:
            # Poured too much (overshoot)
            return PourResult.PARTIAL_SUCCESS  # Can't un-pour

    return PourResult.FAILED_MAX_RETRIES
```

**Failure Modes and Detection**:
- **Source empty**: Weight sensor shows no mass change in destination → return source-empty error.
- **Spill detected**: Camera sees liquid outside destination → stop, return spill error.
- **Destination full**: Weight exceeds capacity before reaching target volume → stop, return overflow error.
- **Tipping failure**: IMU shows excessive tilt (>45°) but no pour → gripper slipped, return grasp-lost error.

**Primitive Lifecycle Diagram** (UML State Machine):

```
[idle] --execute()--> [validating_preconditions]
                              |
                       [precondition_check]
                              |
                     +--------+--------+
                     |                 |
               [preconditions_met]  [preconditions_failed]
                     |                 |
                [executing] -----> [failure_state] --> [idle]
                     |
              [monitoring_progress]
                     |
              +------+------+
              |             |
         [success]      [timeout/error]
              |             |
          [success_state]  [retry_logic]
              |             |
           [idle]       [executing] (if retries left)
                            |
                         [failure_state] (if max retries exceeded)
                            |
                          [idle]
```

**Composition Rules**:

- **Valid sequences**:
  - `navigate(location) → detect_object(type) → grasp(object_id)` (must detect before grasping)
  - `grasp(object) → navigate(destination) → place(location)` (pick-and-place)
  - `open_drawer(handle) → detect_object(type) → grasp(object) → close_drawer(handle)` (retrieve from drawer)

- **Invalid sequences**:
  - `grasp(object) → grasp(another_object)` (gripper already full—must `place` first)
  - `place(location)` without prior `grasp` (precondition violation: nothing in gripper)
  - `navigate(pose) → navigate(pose2)` immediately (redundant—just navigate to pose2 directly)

- **Idempotency considerations**:
  - `navigate(pose)` is idempotent (calling twice with same pose is safe).
  - `grasp(object)` is NOT idempotent (calling twice fails—gripper already closed).
  - `check_status(device)` is idempotent (reading state doesn't change it).

#### Grading Rubric (Total: 20 points)
- **Completeness** (5 pts): At least 12 primitives covering navigation, manipulation, perception, communication
- **Design quality** (5 pts): Primitives are atomic, composable, follow clear pre/postconditions
- **Specification rigor** (4 pts): Clear parameter types, duration estimates, pre/postconditions
- **Error handling** (4 pts): Thoughtful timeout, retry logic, failure mode detection for 3 primitives
- **Documentation** (2 pts): UML diagram, composition rules, clear explanations

#### Common Mistakes
- Too many low-level primitives (e.g., `move_joint_1`, `move_joint_2`—should use `grasp` abstraction)
- Missing preconditions (e.g., `place` doesn't check if gripper is holding an object)
- Unrealistic duration estimates (claiming `navigate` takes 1s across 10m warehouse)
- No retry logic (assumes all actions succeed on first attempt)
- Missing composition rules (no guidance on valid sequences)

---

### Exercise 5: VLA Latency Analysis

**Difficulty**: Recall | **Time**: 30 min

#### Complete Solution

**Latency Breakdown for "Robot, bring me water"**:

| **Stage** | **Best Case** | **Typical Case** | **Worst Case** |
|-----------|---------------|------------------|----------------|
| **Voice Activity Detection (VAD)** | 10ms (fast VAD, short command) | 30ms (standard VAD frame) | 50ms (delayed voice detection) |
| **Whisper Transcription** | 100ms (Whisper `tiny`, 2s audio, GPU) | 300ms (Whisper `base`, 3s audio, GPU) | 500ms (Whisper `small`, 5s audio, CPU) |
| **LLM Planning** | 500ms (Llama-2-7B local, cached KV) | 1500ms (GPT-4, cold start) | 3000ms (GPT-4, complex prompt, API delay) |
| **Action Execution** | 8s (water bottle on kitchen table, robot nearby) | 15s (navigate to kitchen 5m away, grasp, return) | 30s (navigate 10m, search multiple locations, retry failed grasp) |
| **Total End-to-End** | **8.61s** | **16.83s** | **33.55s** |

**Bottleneck Identification**:

- **Primary bottleneck**: **Action Execution** (8-30s, 80-90% of total latency).
  - Justification: Physical motion (navigation, manipulation) is orders of magnitude slower than computation.
  - Cannot be eliminated (physics constraints), but can be optimized (faster base speed, smoother motion planning).

- **Secondary bottleneck**: **LLM Planning** (0.5-3s, 5-18% of total latency).
  - Justification: Large language models (GPT-4) have high inference latency, especially on complex prompts.
  - Can be reduced with local models (Llama-2-7B) or caching.

- **Minor contributors**: VAD (10-50ms, <1%) and Whisper (100-500ms, 1-3%) are negligible compared to action execution.

**Timeline Diagram** (Typical Case):

```
[VAD: 30ms] --> [Whisper: 300ms] --> [LLM: 1500ms] --> [Action Execution: 15s]
0ms            30ms                 330ms              1830ms                   16830ms

Parallel opportunities (none in baseline—all sequential)
```

**Optimization Strategies** (Target: -40% latency reduction):

**1. Use Local LLM with KV Caching** (Reduces LLM latency by 60-70%)

**Implementation**:
- Deploy Llama-2-7B-Instruct locally (RTX 4090 or cloud GPU).
- Enable KV (key-value) caching: Stores prompt embeddings from previous requests, reuses them for similar prompts.
- Use prompt templates with fixed system message (cached) + variable user command.

**Expected Impact**:
- LLM latency: 1500ms → **500ms** (3× speedup from local + caching).
- Total latency: 16.83s → **15.83s** (-6% total, -66% LLM stage).

**Trade-offs**:
- Accuracy: Llama-2-7B is less capable than GPT-4 (may generate suboptimal plans for complex tasks).
- Cost: Requires GPU ($1500 RTX 4090) or cloud GPU rental ($1-2/hr), but saves API costs ($0.03/request × 1000 requests = $30 saved).
- Privacy: Data stays on-premise (good for sensitive applications).

**2. Stream Whisper Transcription** (Reduces perceived latency by 50-70%)

**Implementation**:
- Instead of buffering entire voice command, start transcription after 1s of audio.
- Use Whisper's streaming mode (process audio in overlapping 1s chunks).
- Pass partial transcriptions to LLM (trigger planning before speech ends).

**Expected Impact**:
- Perceived latency: User finishes speaking → robot starts moving.
  - Baseline: User speaks 3s → waits 1.8s (VAD + Whisper + LLM) → robot moves = **4.8s delay**.
  - Streaming: User speaks 3s → transcription starts at 1s → LLM starts at 2s → robot moves at 4.5s (overlaps with speech) = **1.5s delay** after speech ends.
- Total latency unchanged (16.83s), but user experience improved (robot appears more responsive).

**Trade-offs**:
- Accuracy: Partial transcriptions may be incorrect (e.g., mishears "water" as "daughter" at 1s, corrects at 3s).
- Complexity: Requires canceling/replanning if transcription changes mid-stream.

**3. Parallel Execution: LLM Planning During Navigation** (Reduces action execution by 10-20%)

**Implementation**:
- LLM generates plan asynchronously while robot navigates to probable first location (kitchen).
- For common commands ("bring me X"), start navigating to kitchen immediately (90% of objects are in kitchen).
- If LLM plan differs (e.g., "bring me my jacket" → bedroom), cancel kitchen navigation, reroute.

**Expected Impact**:
- Action execution latency: 15s → **13s** (save 2s of navigation by starting early).
- Total latency: 16.83s → **14.83s** (-12% total).

**Trade-offs**:
- Risk: If LLM plan is wrong, robot wastes energy navigating to wrong location (then re-navigates).
- Applicability: Only works for predictable tasks (80% of household commands are kitchen-related).

**Combined Optimizations**:

| **Optimization** | **Latency Reduction** | **Cumulative Total** | **% Reduction from Baseline** |
|------------------|-----------------------|----------------------|-------------------------------|
| Baseline | - | 16.83s | 0% |
| Local LLM + Caching | -1.0s (LLM stage) | 15.83s | -6% |
| Streaming Whisper | -0s (total), -3.3s (perceived) | 15.83s (12.5s perceived) | -26% (perceived) |
| Parallel Execution | -2.0s (action stage) | 13.83s | **-18%** (total), **-47%** (perceived) |

**Trade-off Matrix**:

| **Strategy** | **Latency** | **Accuracy** | **Cost** |
|--------------|-------------|--------------|----------|
| **Cloud GPT-4** | High (1.5-3s) | Best (90%+ plan quality) | High ($0.03/request) |
| **Local Llama-2-7B** | Medium (0.5-1s) | Good (75-85% plan quality) | Medium (GPU $1500 one-time) |
| **Streaming Whisper** | Low (perceived -3s) | Medium (partial transcriptions may be wrong) | Low (no additional cost) |
| **Parallel Execution** | Low (-2s action) | Medium (risk of wasted navigation) | None |

**Recommendation**:
- **Latency-critical** (home robots): Local Llama-2-7B + Streaming Whisper + Parallel Execution → **10.5s perceived latency** (47% reduction).
- **Accuracy-critical** (hospital robots): Cloud GPT-4 + Non-streaming Whisper → **16.8s** (safe, high accuracy).
- **Cost-critical** (consumer robots): Local Llama-2-7B + non-streaming → **15.8s** (no API costs).

#### Grading Rubric (Total: 15 points)
- **Latency calculations** (5 pts): Accurate estimates for best/typical/worst cases (within 20% of realistic values)
- **Bottleneck identification** (3 pts): Correctly identifies action execution as primary bottleneck, LLM as secondary
- **Optimization strategies** (4 pts): 3 creative, feasible strategies with quantitative impact (not vague)
- **Trade-off analysis** (2 pts): Balanced consideration of latency vs. accuracy vs. cost
- **Visualizations** (1 pt): Clear timeline diagram and trade-off matrix

#### Common Mistakes
- Claiming Whisper is the bottleneck (it's only 300ms vs. 15s action execution)
- Unrealistic optimization claims (e.g., "reduce latency by 90%"—can't speed up physical motion)
- Ignoring trade-offs (claiming local LLM is "always better"—ignores accuracy loss)
- Missing perceived vs. actual latency distinction (streaming improves perceived, not actual)

---

## 📗 Application Exercises

### Exercise 6: Implement Whisper ROS 2 Node with Noise Filtering

**Difficulty**: Application | **Time**: 2 hours

#### Solution Outline

**Extended Node Implementation** (`whisper_enhanced_node.py`):

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioStamped
from std_msgs.msg import String
from custom_msgs.msg import WhisperMetrics  # Custom message
import whisper
import numpy as np
from scipy.signal import butter, lfilter
import webrtcvad

class NoiseEstimator:
    """Estimates background noise level during silent periods."""
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.noise_samples = []
        self.noise_rms = 0.02  # Initial estimate (quiet environment)

    def update(self, audio_chunk, is_speech):
        if not is_speech:
            # Collect noise samples during silence
            rms = np.sqrt(np.mean(audio_chunk**2))
            self.noise_samples.append(rms)
            if len(self.noise_samples) > 50:
                self.noise_rms = np.median(self.noise_samples[-50:])  # Moving median

    def get_snr(self, audio_chunk):
        """Calculate Signal-to-Noise Ratio"""
        signal_rms = np.sqrt(np.mean(audio_chunk**2))
        return 20 * np.log10(signal_rms / (self.noise_rms + 1e-6))  # dB

class AdaptiveVAD:
    """Adjusts VAD aggressiveness based on background noise."""
    def __init__(self):
        self.vad = webrtcvad.Vad()
        self.aggressiveness = 1  # 0-3, higher = more aggressive

    def set_aggressiveness_from_snr(self, snr_db):
        if snr_db > 20:  # High SNR (quiet background)
            self.aggressiveness = 0
        elif snr_db > 10:
            self.aggressiveness = 1
        elif snr_db > 5:
            self.aggressiveness = 2
        else:  # Low SNR (noisy background)
            self.aggressiveness = 3
        self.vad.set_mode(self.aggressiveness)

    def is_speech(self, audio_chunk, sample_rate):
        return self.vad.is_speech(audio_chunk.tobytes(), sample_rate)

class WhisperEnhancedNode(Node):
    def __init__(self):
        super().__init__('whisper_enhanced_node')

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('min_confidence', 0.7)
        self.declare_parameter('noise_adapt_enabled', True)

        model_size = self.get_parameter('model_size').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Initialize components
        self.noise_estimator = NoiseEstimator()
        self.adaptive_vad = AdaptiveVAD()

        # Subscribers and Publishers
        self.audio_sub = self.create_subscription(
            AudioStamped, '/audio', self.audio_callback, 10)
        self.transcript_pub = self.create_publisher(String, '/transcript', 10)
        self.metrics_pub = self.create_publisher(WhisperMetrics, '/whisper_metrics', 10)

        self.audio_buffer = []
        self.is_speaking = False

    def bandpass_filter(self, audio, lowcut=300, highcut=3400, fs=16000, order=5):
        """Apply bandpass filter for speech frequency range."""
        nyquist = 0.5 * fs
        low = lowcut / nyquist
        high = highcut / nyquist
        b, a = butter(order, [low, high], btype='band')
        return lfilter(b, a, audio)

    def audio_callback(self, msg):
        audio_chunk = np.frombuffer(msg.audio.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Apply bandpass filter
        filtered_audio = self.bandpass_filter(audio_chunk)

        # Adaptive VAD
        snr = self.noise_estimator.get_snr(filtered_audio)
        self.adaptive_vad.set_aggressiveness_from_snr(snr)
        is_speech = self.adaptive_vad.is_speech(filtered_audio, 16000)

        # Update noise estimate
        self.noise_estimator.update(filtered_audio, is_speech)

        if is_speech:
            self.audio_buffer.append(filtered_audio)
            self.is_speaking = True
        elif self.is_speaking and len(self.audio_buffer) > 0:
            # End of speech, transcribe
            self.transcribe_and_publish()
            self.audio_buffer = []
            self.is_speaking = False

    def transcribe_and_publish(self):
        audio = np.concatenate(self.audio_buffer)

        # Whisper transcription
        start_time = self.get_clock().now()
        result = self.model.transcribe(audio, fp16=False)
        latency = (self.get_clock().now() - start_time).nanoseconds / 1e6  # ms

        # Confidence-based rejection
        confidence = np.exp(result['segments'][0]['avg_logprob']) if result['segments'] else 0.0

        # Dynamic confidence threshold based on SNR
        snr = self.noise_estimator.get_snr(audio)
        adaptive_threshold = self.min_confidence + (0.1 if snr < 10 else 0.0)

        if confidence >= adaptive_threshold:
            transcript_msg = String(data=result['text'])
            self.transcript_pub.publish(transcript_msg)

        # Publish metrics
        metrics = WhisperMetrics()
        metrics.transcription = result['text']
        metrics.confidence = confidence
        metrics.latency_ms = latency
        metrics.snr_db = snr
        metrics.vad_aggressiveness = self.adaptive_vad.aggressiveness
        metrics.accepted = (confidence >= adaptive_threshold)
        self.metrics_pub.publish(metrics)

def main(args=None):
    rclpy.init(args=args)
    node = WhisperEnhancedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Custom Message Definition** (`WhisperMetrics.msg`):

```
string transcription
float32 confidence
float32 latency_ms
float32 snr_db
int32 vad_aggressiveness
bool accepted
```

**Launch File** (`whisper_enhanced.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whisper_ros',
            executable='whisper_enhanced_node',
            name='whisper_enhanced',
            parameters=[{
                'model_size': 'base',
                'min_confidence': 0.7,
                'noise_adapt_enabled': True
            }]
        )
    ])
```

**Test Report** (Performance in 3 Environments):

| **Environment** | **Background Noise** | **WER (Baseline)** | **WER (Enhanced)** | **Avg Latency** | **Acceptance Rate** |
|-----------------|----------------------|--------------------|--------------------|-----------------|---------------------|
| **Quiet Office** | 35 dB | 5.2% | 4.8% | 280ms | 95% (confidence > 0.75) |
| **Active Kitchen** | 58 dB | 18.5% | 12.1% | 310ms | 78% (adaptive threshold = 0.8) |
| **Robot Workshop** | 72 dB | 35.8% | 22.3% | 350ms | 62% (adaptive threshold = 0.85) |

**Key Findings**:
- Bandpass filter reduces WER by 3-7% in noisy environments (removes low-frequency motor noise).
- Adaptive VAD improves speech detection in workshop (fewer false starts from background noise).
- Dynamic confidence thresholding prevents false accepts in high-noise (at cost of some rejected valid transcriptions).

**README.md Snippet**:

```markdown
## Usage

### Launch the Enhanced Whisper Node
ros2 launch whisper_ros whisper_enhanced.launch.py

### Tuning Parameters
- `model_size`: {tiny, base, small} - Smaller models are faster but less accurate
- `min_confidence`: 0.6-0.9 - Higher values reject more low-confidence transcriptions
- `noise_adapt_enabled`: true/false - Enable adaptive VAD and threshold

### Monitoring Performance
ros2 topic echo /whisper_metrics
```

#### Grading Rubric (Total: 20 points)
- **Functionality** (7 pts): All 4 enhancements (adaptive VAD, noise filtering, confidence rejection, metrics) implemented correctly
- **Code quality** (4 pts): Clean, well-commented, follows ROS 2 best practices (typing, parameter handling)
- **Performance** (4 pts): Demonstrable WER improvement in noisy environments (>5 percentage points)
- **Testing** (3 pts): Comprehensive test results in 3 environments with quantitative metrics
- **Documentation** (2 pts): Clear README with usage instructions, parameter tuning guide

#### Common Mistakes
- Applying bandpass filter incorrectly (e.g., cutoff frequencies wrong—should be 300-3400 Hz)
- VAD aggressiveness not actually adapting (hard-coded value instead of SNR-based)
- Confidence threshold is static (doesn't adjust for noise level)
- Missing test results (code only, no performance data)
- Launch file missing parameters (can't configure without editing code)

---

### Exercise 7: Write LLM Prompts for Complex Task Decomposition

**Difficulty**: Application | **Time**: 1.5 hours

#### Solution Outline

**Level 1 - Basic Decomposition**:

**System Prompt**:
```
You are a household service robot with the following action primitives:

- navigate(location: str) - Move to a room or landmark
- detect_object(object_type: str, store_as: str) - Find object and store its ID
- grasp(object_id: str) - Pick up object
- place(location: str) - Put down object
- vacuum(area: str) - Clean floor area
- organize(object_id: str, destination: str) - Move object to proper location

Output the plan as a JSON list with numbered steps. Each step should have: "action", "parameters", "reasoning".

Example:
[
  {"action": "navigate", "parameters": {"location": "living_room"}, "reasoning": "Go to area that needs cleaning"},
  {"action": "detect_object", "parameters": {"object_type": "toy", "store_as": "toy1"}, "reasoning": "Find first object to organize"}
]
```

**User Prompt**:
```
Task: "Clean the living room"
```

**Expected Output**:
```json
[
  {"action": "navigate", "parameters": {"location": "living_room"}, "reasoning": "Start by going to the room"},
  {"action": "detect_object", "parameters": {"object_type": "toy", "store_as": "toy1"}, "reasoning": "Identify clutter to organize"},
  {"action": "grasp", "parameters": {"object_id": "$toy1"}, "reasoning": "Pick up first toy"},
  {"action": "place", "parameters": {"location": "toy_box"}, "reasoning": "Put toy in designated storage"},
  {"action": "detect_object", "parameters": {"object_type": "dirty_dish", "store_as": "dish1"}, "reasoning": "Find next item to organize"},
  {"action": "grasp", "parameters": {"object_id": "$dish1"}, "reasoning": "Pick up dish"},
  {"action": "place", "parameters": {"location": "kitchen"}, "reasoning": "Return dish to kitchen"},
  {"action": "vacuum", "parameters": {"area": "living_room"}, "reasoning": "Clean the floor after organizing"}
]
```

**Variations Tested**:
1. "Tidy up the den" → Similar plan, substitutes "den" for "living_room"
2. "Clean the playroom and organize toys" → Focuses on toys, adds extra organization steps
3. "Straighten the lounge" → Different phrasing, same plan structure

**Level 2 - Conditional Reasoning**:

**System Prompt** (Extended):
```
You are a household service robot. You can use conditional logic:

Conditionals:
- if_detected(object_type: str, then: [actions], else: [actions])
- count_objects(object_type: str, store_as: str) - Stores count in variable

Output plan as JSON with conditional blocks.
```

**User Prompt**:
```
Task: "Set the table for dinner if we have guests"
```

**Expected Output**:
```json
[
  {"action": "count_objects", "parameters": {"object_type": "person", "store_as": "guest_count"}, "reasoning": "Detect how many people are present"},
  {"action": "if_detected", "parameters": {
    "condition": "$guest_count > 1",
    "then": [
      {"action": "navigate", "parameters": {"location": "kitchen"}},
      {"action": "grasp", "parameters": {"object_id": "plate"}},
      {"action": "place", "parameters": {"location": "table_position_1"}},
      {"action": "grasp", "parameters": {"object_id": "plate"}},
      {"action": "place", "parameters": {"location": "table_position_2"}},
      {"action": "grasp", "parameters": {"object_id": "fork"}},
      {"action": "place", "parameters": {"location": "table_position_1"}},
      {"action": "grasp", "parameters": {"object_id": "fork"}},
      {"action": "place", "parameters": {"location": "table_position_2"}}
    ],
    "else": [
      {"action": "say", "parameters": {"message": "No guests detected, table setting skipped"}}
    ]
  }, "reasoning": "Only set table if there are multiple people (host + guests)"}
]
```

**Level 3 - Multi-Step Planning with Recovery**:

**User Prompt**:
```
Task: "Prepare a sandwich and bring it to the user, even if ingredients are missing"
```

**Expected Output** (Simplified):
```json
[
  {"action": "navigate", "parameters": {"location": "kitchen"}},
  {"action": "detect_object", "parameters": {"object_type": "bread", "store_as": "bread"}},
  {"action": "if_detected", "parameters": {
    "condition": "$bread != null",
    "then": [
      {"action": "detect_object", "parameters": {"object_type": "cheese", "store_as": "cheese"}},
      {"action": "if_detected", "parameters": {
        "condition": "$cheese != null",
        "then": [
          {"action": "grasp", "parameters": {"object_id": "$bread"}},
          {"action": "place", "parameters": {"location": "cutting_board"}},
          {"action": "grasp", "parameters": {"object_id": "$cheese"}},
          {"action": "place", "parameters": {"location": "sandwich"}},
          {"action": "navigate", "parameters": {"location": "user"}},
          {"action": "place", "parameters": {"location": "user_table"}}
        ],
        "else": [
          {"action": "say", "parameters": {"message": "Cheese not found, making bread-only sandwich"}},
          {"action": "grasp", "parameters": {"object_id": "$bread"}},
          {"action": "navigate", "parameters": {"location": "user"}},
          {"action": "place", "parameters": {"location": "user_table"}}
        ]
      }}
    ],
    "else": [
      {"action": "say", "parameters": {"message": "Bread not found, cannot make sandwich. Alerting user."}},
      {"action": "navigate", "parameters": {"location": "user"}},
      {"action": "say", "parameters": {"message": "Apologies, no bread available for sandwich"}}
    ]
  }}
]
```

**Python Testing Script** (`test_prompts.py`):

```python
import openai
import json

def test_level_1():
    tasks = ["Clean the living room", "Tidy up the den", "Straighten the lounge"]
    for task in tasks:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT_L1},
                {"role": "user", "content": f"Task: {task}"}
            ],
            temperature=0.2
        )
        plan = json.loads(response.choices[0].message.content)
        print(f"Task: {task}")
        print(f"Plan: {json.dumps(plan, indent=2)}\n")

# Run tests for all 3 levels...
```

**Failure Analysis** (3 identified modes + fixes):

1. **Failure**: LLM invents actions not in primitive set (e.g., `assemble_sandwich()` instead of using `place`).
   - **Fix**: Explicitly list ALL available primitives in system prompt, add constraint: "ONLY use the listed primitives. Do not invent new actions."

2. **Failure**: Nested conditionals become too deep (5+ levels), unreadable.
   - **Fix**: Add constraint: "Limit conditional nesting to maximum 2 levels. For complex logic, break into sequential checks."

3. **Failure**: Error handling doesn't notify user (silent failures).
   - **Fix**: Add requirement: "For all `else` branches handling failures, include a `say()` action to notify the user what went wrong."

#### Grading Rubric (Total: 15 points)
- **Prompt design** (5 pts): All 3 prompts are well-structured, produce valid JSON action sequences
- **Complexity handling** (4 pts): Level 3 prompts successfully handle conditionals, error recovery
- **Testing rigor** (3 pts): 3 variations per level tested, results documented
- **Failure analysis** (2 pts): Insightful identification of 3+ failure modes with concrete fixes
- **Implementation** (1 pt): Working Python code producing valid outputs

#### Common Mistakes
- System prompt doesn't constrain output format (LLM returns prose instead of JSON)
- Missing `reasoning` field in outputs (reduces interpretability)
- Level 3 prompts don't actually handle missing ingredients (plan assumes everything is present)
- No refinement after testing (identified failures but didn't update prompts)
- Testing on only 1 variation per level (insufficient coverage)

---

### Exercise 8: Implement Action Executor with Variable Binding

**Difficulty**: Application | **Time**: 2.5 hours

#### Solution Outline (Key Components)

**VariableContext Class**:

```python
from typing import Any, Dict, Optional

class VariableContext:
    """Manages variable storage with scopes: global, task, action."""
    def __init__(self):
        self.global_vars: Dict[str, Any] = {}
        self.task_vars: Dict[str, Any] = {}
        self.action_vars: Dict[str, Any] = {}

    def set(self, key: str, value: Any, scope: str = 'task'):
        """Store variable in specified scope."""
        if scope == 'global':
            self.global_vars[key] = value
        elif scope == 'task':
            self.task_vars[key] = value
        elif scope == 'action':
            self.action_vars[key] = value
        else:
            raise ValueError(f"Invalid scope: {scope}")

    def get(self, key: str) -> Optional[Any]:
        """Retrieve variable, checking action -> task -> global scope."""
        if key in self.action_vars:
            return self.action_vars[key]
        elif key in self.task_vars:
            return self.task_vars[key]
        elif key in self.global_vars:
            return self.global_vars[key]
        return None

    def resolve(self, value: str) -> Any:
        """Resolve variable references like '$detected_objects[0]'."""
        if not isinstance(value, str) or not value.startswith('$'):
            return value  # Not a variable reference

        # Handle array indexing: $var[0]
        if '[' in value:
            var_name, index_str = value[1:].split('[')
            index = int(index_str.rstrip(']'))
            var_value = self.get(var_name)
            if var_value is None:
                raise ValueError(f"Variable {var_name} not found")
            if not isinstance(var_value, list):
                raise TypeError(f"Variable {var_name} is not indexable")
            return var_value[index]

        # Simple variable: $var
        var_name = value[1:]
        var_value = self.get(var_name)
        if var_value is None:
            raise ValueError(f"Variable {var_name} not defined")
        return var_value

    def clear_task_scope(self):
        """Clear task-level variables (call between tasks)."""
        self.task_vars = {}
        self.action_vars = {}

    def clear_action_scope(self):
        """Clear action-level variables (call between actions)."""
        self.action_vars = {}
```

**ActionExecutorEnhanced Class** (Simplified):

```python
import time
from enum import Enum
from typing import Any, Dict, Callable

class ActionResult(Enum):
    SUCCESS = "success"
    TIMEOUT = "timeout"
    FAILED = "failed"
    TYPE_ERROR = "type_error"
    DEPENDENCY_ERROR = "dependency_error"

class ActionExecutorEnhanced:
    def __init__(self):
        self.context = VariableContext()
        self.action_registry: Dict[str, Callable] = {}
        self.timeout_config: Dict[str, float] = {
            'default': 30.0,
            'navigate': 45.0,
            'grasp': 10.0,
            'detect_object': 5.0
        }
        self.max_retries = 3
        self.register_default_actions()

    def register_action(self, name: str, func: Callable, timeout: Optional[float] = None):
        """Register an action primitive."""
        self.action_registry[name] = func
        if timeout:
            self.timeout_config[name] = timeout

    def type_check(self, param_name: str, param_value: Any, expected_type: type) -> bool:
        """Validate parameter type."""
        if not isinstance(param_value, expected_type):
            raise TypeError(f"Parameter '{param_name}' expected {expected_type.__name__}, "
                            f"got {type(param_value).__name__}: {param_value}")
        return True

    def execute_action(self, action_name: str, params: Dict[str, Any]) -> ActionResult:
        """Execute a single action with timeout, retry, and variable resolution."""
        if action_name not in self.action_registry:
            raise ValueError(f"Unknown action: {action_name}")

        # Resolve variable references in parameters
        resolved_params = {}
        for key, value in params.items():
            try:
                resolved_params[key] = self.context.resolve(value)
            except ValueError as e:
                return ActionResult.DEPENDENCY_ERROR  # Variable not found

        # Type checking (simplified—would use action signatures in production)
        # ... (omitted for brevity)

        # Execute with timeout and retries
        timeout = self.timeout_config.get(action_name, self.timeout_config['default'])
        action_func = self.action_registry[action_name]

        for attempt in range(self.max_retries):
            try:
                start_time = time.time()
                result = action_func(**resolved_params)
                elapsed = time.time() - start_time

                if elapsed > timeout:
                    if attempt < self.max_retries - 1:
                        continue  # Retry
                    return ActionResult.TIMEOUT

                if result == ActionResult.SUCCESS:
                    return ActionResult.SUCCESS
                elif result in [ActionResult.FAILED, ActionResult.TIMEOUT]:
                    if attempt < self.max_retries - 1:
                        time.sleep(2 ** attempt)  # Exponential backoff
                        continue
                    return result
            except Exception as e:
                if attempt < self.max_retries - 1:
                    continue
                return ActionResult.FAILED

        return ActionResult.FAILED

    def execute_plan(self, plan: list) -> bool:
        """Execute a sequence of actions."""
        for action in plan:
            action_name = action['action']
            params = action['parameters']

            # Check for special 'store_as' parameter (variable binding)
            store_as = params.pop('store_as', None)

            result = self.execute_action(action_name, params)

            if result == ActionResult.SUCCESS and store_as:
                # Store action result in variable
                self.context.set(store_as, result.value, scope='task')
            elif result != ActionResult.SUCCESS:
                print(f"Action {action_name} failed with result: {result}")
                return False

            self.context.clear_action_scope()

        self.context.clear_task_scope()
        return True
```

**Unit Tests** (Simplified):

```python
import unittest

class TestVariableContext(unittest.TestCase):
    def test_variable_resolution(self):
        ctx = VariableContext()
        ctx.set('detected_object', 'cup_123', scope='task')
        self.assertEqual(ctx.resolve('$detected_object'), 'cup_123')

    def test_array_indexing(self):
        ctx = VariableContext()
        ctx.set('detected_objects', ['cup_123', 'bottle_456'], scope='task')
        self.assertEqual(ctx.resolve('$detected_objects[0]'), 'cup_123')

    def test_scope_priority(self):
        ctx = VariableContext()
        ctx.set('var', 'global_value', scope='global')
        ctx.set('var', 'task_value', scope='task')
        self.assertEqual(ctx.get('var'), 'task_value')  # Task scope wins

class TestActionExecutor(unittest.TestCase):
    def test_dependency_resolution(self):
        executor = ActionExecutorEnhanced()
        executor.context.set('cup_id', 'cup_123', scope='task')

        # Mock grasp action
        def mock_grasp(object_id):
            return ActionResult.SUCCESS if object_id == 'cup_123' else ActionResult.FAILED

        executor.register_action('grasp', mock_grasp)
        result = executor.execute_action('grasp', {'object_id': '$cup_id'})
        self.assertEqual(result, ActionResult.SUCCESS)
```

**Integration Test** (Complete Sequence):

```python
def test_end_to_end_execution():
    executor = ActionExecutorEnhanced()

    # Register mock actions
    executor.register_action('navigate', lambda location: ActionResult.SUCCESS)
    executor.register_action('detect_object', lambda object_type, store_as: 'cup_123')
    executor.register_action('grasp', lambda object_id: ActionResult.SUCCESS if object_id == 'cup_123' else ActionResult.FAILED)
    executor.register_action('place', lambda location: ActionResult.SUCCESS)

    plan = [
        {'action': 'navigate', 'parameters': {'location': 'kitchen'}},
        {'action': 'detect_object', 'parameters': {'object_type': 'cup', 'store_as': '$cup_id'}},
        {'action': 'grasp', 'parameters': {'object_id': '$cup_id'}},
        {'action': 'navigate', 'parameters': {'location': '$user_location'}},
        {'action': 'place', 'parameters': {'location': 'table'}}
    ]

    executor.context.set('user_location', 'living_room', scope='global')
    success = executor.execute_plan(plan)
    assert success, "Plan execution should succeed"
```

#### Grading Rubric (Total: 20 points)
- **Variable binding** (5 pts): Correct storage, retrieval, resolution (including array indexing)
- **Type safety** (4 pts): Robust type checking with informative error messages
- **Error handling** (4 pts): Proper timeout (configurable per action), exponential backoff retry
- **Testing** (4 pts): Comprehensive unit tests (variable resolution, type checking) + integration test
- **Code quality** (3 pts): Clean architecture, well-documented, follows Python best practices

#### Common Mistakes
- Variable resolution doesn't handle arrays (`$var[0]` fails)
- No scope hierarchy (global, task, action—should check action first)
- Timeout is global (doesn't vary per action type—`navigate` needs longer timeout than `grasp`)
- Retry logic doesn't use exponential backoff (retries immediately, overwhelming system)
- Missing integration test (only unit tests, no end-to-end validation)

---

*Due to length constraints, I'm providing detailed solutions for Exercises 1-8. Exercises 9-15 follow similar patterns with increasing complexity. Would you like me to continue with the remaining exercises, or is this sufficient for instructor reference?*

**Summary of Remaining Exercises (9-15) - Brief Outlines**:

**Exercise 9**: Integration test suite combining Whisper, LLM planner, and action executor. Measures end-to-end latency, success rate across 10 scenarios (navigation, manipulation, combined, error handling).

**Exercise 10**: Performance benchmarking with instrumentation, comparing local vs. cloud LLM, Whisper model sizes, measuring p95/p99 latencies, implementing optimizations (caching, streaming).

**Exercise 11**: System architecture design for warehouse robot VLA system. Comprehensive technical spec covering perception, language understanding, planning, execution, safety (3000-4000 words).

**Exercise 12**: FMEA (Failure Modes and Effects Analysis) for hospital service robot. Identifies 15+ failure modes, calculates RPN (severity × likelihood × detection), designs safety monitors.

**Exercise 13**: Rigorous comparison of GPT-4, Claude 3, Llama-2-70B, Mistral-7B for robotics planning. Evaluates plan quality, latency, cost across 30 test tasks with statistical analysis.

**Exercise 14**: Multi-robot coordination system design for 5 heterogeneous robots. Hierarchical planning, conflict resolution, distributed architecture, optimization algorithms.

**Exercise 15**: Capstone deployment on physical robot or high-fidelity simulator. Demonstrates voice→task execution with navigation, manipulation, error recovery. Includes demo video, evaluation report.

Each synthesis exercise requires 3-4 hours and integrates all module concepts.

