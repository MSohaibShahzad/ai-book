---
title: "Module 4: Exercises"
slug: /vision-language-action/exercises
sidebar_label: "Exercises"
sidebar_position: 8
toc: true
description: "15 hands-on exercises for Module 4: Recall, Application, and Synthesis problems covering VLA pipelines, Whisper integration, LLM planning, and capstone deployment."
---

# Module 4: Vision-Language-Action Exercises

This chapter provides 15 progressive exercises designed to reinforce and extend your understanding of Vision-Language-Action (VLA) systems for Physical AI. Exercises are categorized into three levels:

- **📘 Recall**: Foundational knowledge and concept comprehension
- **📗 Application**: Hands-on implementation and problem-solving
- **📕 Synthesis**: Advanced integration and system design

## Exercise Format

Each exercise includes:
- **Difficulty**: Recall, Application, or Synthesis
- **Time Estimate**: Expected completion time
- **Prerequisites**: Required knowledge/completed exercises
- **Prompt**: Problem statement
- **Deliverable**: Expected submission format
- **Assessment Criteria**: 3-5 evaluation points
- **Chapter References**: Related reading material

---

## 📘 Recall Exercises

### Exercise 1: VLA Architecture Comparison

**Difficulty**: Recall
**Time Estimate**: 30 minutes
**Prerequisites**: Chapter 1 (Introduction)

#### Prompt

Compare and contrast three VLA architecture paradigms discussed in the module:

1. **Pipeline Architecture**: Sequential processing (Vision → Language → Action)
2. **End-to-End Architecture**: Direct sensor-to-action learning (e.g., RT-1, RT-2)
3. **Hybrid Architecture**: LLM planning + learned policies (e.g., SayCan)

For each architecture:
- Describe the data flow and processing stages
- List 2 key advantages and 2 key disadvantages
- Provide one real-world robotics scenario where it excels
- Identify the primary failure modes

Create a comparison table and write a 300-word analysis of when to choose each approach.

#### Deliverable

- Markdown table comparing the three architectures across dimensions: latency, sample efficiency, interpretability, generalization, hardware requirements
- 300-word analytical essay with specific examples
- Annotated diagram showing information flow for each architecture

#### Assessment Criteria

1. **Completeness**: All three architectures are accurately described with correct technical details (25%)
2. **Comparative Analysis**: Clear articulation of trade-offs with concrete examples (25%)
3. **Scenario Matching**: Appropriate matching of architecture to use case with justification (25%)
4. **Technical Accuracy**: Correct understanding of latency, sample efficiency, and failure modes (15%)
5. **Clarity**: Well-organized presentation with effective use of diagrams (10%)

#### Chapter References

- Chapter 1: Introduction to VLA Systems (Sections 1.2, 1.3)
- Chapter 2: Vision Systems (Section 2.4 on integration)
- Chapter 3: LLM Planning (Section 3.1 on architecture patterns)

---

### Exercise 2: Whisper Accuracy Metrics

**Difficulty**: Recall
**Time Estimate**: 25 minutes
**Prerequisites**: Chapter 4 (Speech Recognition)

#### Prompt

OpenAI's Whisper model is evaluated using multiple accuracy metrics. Define and explain the following metrics in the context of robotics speech recognition:

1. **Word Error Rate (WER)**: Definition, calculation formula, interpretation
2. **Character Error Rate (CER)**: When is CER more useful than WER?
3. **Confidence Scores**: How does Whisper compute confidence? What threshold should you use?
4. **Real-Time Factor (RTF)**: Why does RTF < 1.0 matter for VLA systems?

Additionally, explain how background noise, domain-specific vocabulary, and speaker accent affect each metric. What preprocessing or fine-tuning strategies can mitigate these effects?

#### Deliverable

- Technical report (400-500 words) defining each metric with formulas
- Example calculations showing WER and CER for sample transcriptions
- Table showing Whisper model performance (tiny, base, small, medium, large) across metrics
- 2-3 mitigation strategies for improving accuracy in noisy robot environments

#### Assessment Criteria

1. **Metric Definitions**: Accurate definitions with correct mathematical formulas (30%)
2. **Calculations**: Correct example calculations demonstrating understanding (25%)
3. **Performance Analysis**: Accurate interpretation of Whisper model trade-offs (20%)
4. **Practical Application**: Realistic mitigation strategies grounded in robotics constraints (15%)
5. **Technical Writing**: Clear, concise explanations accessible to robotics engineers (10%)

#### Chapter References

- Chapter 4: Speech Recognition (Section 4.2 on Whisper architecture)
- Examples: Whisper ROS 2 Node (Example 1)

---

### Exercise 3: LLM Prompting Strategies for Task Decomposition

**Difficulty**: Recall
**Time Estimate**: 35 minutes
**Prerequisites**: Chapter 5 (LLM Planning)

#### Prompt

Analyze three prompting strategies for decomposing natural language commands into robotic action primitives:

1. **Zero-Shot Prompting**: Provide task description only, rely on model's pre-trained knowledge
2. **Few-Shot Prompting**: Include 3-5 examples of task→actions mappings
3. **Chain-of-Thought (CoT) Prompting**: Request step-by-step reasoning before action sequence

For each strategy:
- Write an example prompt for the task: "Make me a cup of coffee"
- Explain when this strategy is most effective
- Identify potential failure modes (e.g., hallucinated actions, incomplete plans)
- Discuss computational and latency trade-offs

Compare the three strategies in terms of: accuracy, robustness to novel tasks, prompt engineering effort, and inference latency.

#### Deliverable

- Three complete example prompts (200-300 words each)
- Comparison table across evaluation dimensions
- 400-word analysis of trade-offs with recommendations for different robot scenarios
- Code snippet showing how to implement each strategy with OpenAI API or Ollama

#### Assessment Criteria

1. **Prompt Quality**: Well-structured prompts that would produce valid action sequences (30%)
2. **Strategy Understanding**: Accurate description of when/why each strategy works (25%)
3. **Failure Mode Analysis**: Realistic identification of failure cases with examples (20%)
4. **Trade-off Analysis**: Thoughtful comparison balancing multiple constraints (15%)
5. **Practical Implementation**: Working code demonstrating each strategy (10%)

#### Chapter References

- Chapter 5: LLM Cognitive Planning (Sections 5.2, 5.3)
- Examples: LLM Planner Node (Example 2)
- Chapter 7: Prompt Engineering (Section 7.1)

---

### Exercise 4: Action Primitive Design Principles

**Difficulty**: Recall
**Time Estimate**: 40 minutes
**Prerequisites**: Chapter 6 (Action Execution)

#### Prompt

Action primitives are the atomic units of robot behavior in VLA systems. Design a set of action primitives for a mobile manipulator operating in a household environment.

Your action primitive library must:
- Include at least 12 distinct primitives covering navigation, manipulation, perception, and communication
- Define each primitive with: name, parameters (with types), preconditions, postconditions, and expected duration
- Follow best practices: atomicity, composability, error-reporting, idempotency (where applicable)

Examples:
- `navigate(target: Pose) → NavigationResult` - Move base to target pose
- `grasp(object_id: str, approach: GraspApproach) → GraspResult` - Pick up object

For three of your primitives, explain:
- Why this level of abstraction is appropriate (not too high-level, not too low-level)
- How you would implement timeout and retry logic
- What failure modes exist and how to detect them

#### Deliverable

- Table of 12+ action primitives with full specifications
- Detailed design rationale for 3 selected primitives (150 words each)
- UML or state machine diagram showing primitive lifecycle (idle → executing → success/failure)
- Discussion of how primitives compose (e.g., which sequences are valid)

#### Assessment Criteria

1. **Completeness**: At least 12 well-defined primitives covering key capabilities (25%)
2. **Design Quality**: Primitives follow atomicity, composability principles (25%)
3. **Specification Rigor**: Clear parameter types, pre/postconditions (20%)
4. **Error Handling**: Thoughtful approach to timeouts, retries, failure detection (20%)
5. **Documentation**: Clear diagrams and explanations (10%)

#### Chapter References

- Chapter 6: Action Execution (Sections 6.1, 6.2)
- Examples: Action Executor (Example 3)

---

### Exercise 5: VLA Latency Analysis

**Difficulty**: Recall
**Time Estimate**: 30 minutes
**Prerequisites**: Chapters 4, 5, 6

#### Prompt

A complete VLA pipeline consists of multiple stages, each contributing to end-to-end latency. Analyze the latency budget for a household service robot responding to voice commands.

Given the following components:
1. **Voice Activity Detection (VAD)**: 10-50ms (depends on frame size)
2. **Whisper Transcription**: 100-500ms (depends on model size, audio length)
3. **LLM Planning**: 500-3000ms (depends on model, prompt length, provider)
4. **Action Execution**: 2-30 seconds (depends on action type)

Tasks:
- Calculate best-case, typical, and worst-case end-to-end latency for: "Robot, bring me water"
- Identify the bottleneck stage(s)
- Propose 3 optimization strategies to reduce latency by at least 40%
- Discuss trade-offs: latency vs. accuracy vs. cost

Consider:
- Streaming vs. batch processing for Whisper
- Local LLM (Ollama/Llama) vs. cloud API (GPT-4)
- Caching frequently used plans
- Parallel execution where possible

#### Deliverable

- Latency breakdown table for best/typical/worst cases
- Annotated timeline diagram showing sequential and parallel operations
- 300-word optimization proposal with quantitative impact estimates
- Trade-off matrix comparing strategies

#### Assessment Criteria

1. **Latency Calculations**: Accurate estimates grounded in realistic performance data (30%)
2. **Bottleneck Identification**: Correct identification of limiting factors (20%)
3. **Optimization Strategies**: Creative, feasible proposals with clear impact (25%)
4. **Trade-off Analysis**: Balanced consideration of latency, accuracy, cost (15%)
5. **Presentation**: Clear visualizations and quantitative justification (10%)

#### Chapter References

- Chapter 8: System Integration (Section 8.3 on performance optimization)
- Examples: Complete VLA Pipeline (Example 4)

---

## 📗 Application Exercises

### Exercise 6: Implement Whisper ROS 2 Node with Noise Filtering

**Difficulty**: Application
**Time Estimate**: 2 hours
**Prerequisites**: Exercise 1, Python, ROS 2 basics

#### Prompt

Extend the Whisper ROS 2 node from Example 1 to handle noisy robot environments. Implement the following enhancements:

1. **Adaptive VAD**: Adjust VAD aggressiveness based on measured background noise levels
2. **Noise Filtering**: Apply bandpass filter (300-3400 Hz for speech) before transcription
3. **Confidence-Based Rejection**: Only publish transcriptions with confidence > dynamic threshold
4. **Performance Monitoring**: Publish metrics (WER estimate, latency, noise level) to `/whisper_metrics`

Your implementation should:
- Extend the base `WhisperROSNode` class from Example 1
- Add a `NoiseEstimator` class that estimates background noise using silent periods
- Implement `AdaptiveVAD` that adjusts sensitivity based on SNR (Signal-to-Noise Ratio)
- Create a custom message type for metrics publishing

Test your node in three environments:
- Quiet office (background noise < 40 dB)
- Active kitchen (background noise 50-60 dB)
- Robot workshop (background noise > 65 dB)

#### Deliverable

- Extended Python node implementation (`whisper_enhanced_node.py`, 400-500 lines)
- Custom message definition (`WhisperMetrics.msg`)
- Launch file with configurable parameters
- Test report showing performance in three environments (include transcription accuracy, latency)
- README with usage instructions and tuning guidance

#### Assessment Criteria

1. **Functionality**: All four enhancements correctly implemented and integrated (35%)
2. **Code Quality**: Clean, well-commented code following ROS 2 best practices (20%)
3. **Performance**: Demonstrable improvement in noisy environments (20%)
4. **Testing**: Comprehensive test results with quantitative metrics (15%)
5. **Documentation**: Clear usage instructions and parameter tuning guide (10%)

#### Chapter References

- Chapter 4: Speech Recognition (Section 4.3 on robustness)
- Examples: Whisper ROS 2 Node (Example 1)

---

### Exercise 7: Write LLM Prompts for Complex Task Decomposition

**Difficulty**: Application
**Time Estimate**: 1.5 hours
**Prerequisites**: Exercise 3, Chapter 5

#### Prompt

Design and test a comprehensive prompt engineering system for decomposing complex household tasks. Implement three increasingly sophisticated prompts:

**Level 1 - Basic Decomposition**: "Clean the living room"
- Should produce 5-8 action primitives
- Handle furniture detection, navigation, vacuuming

**Level 2 - Conditional Reasoning**: "Set the table for dinner if we have guests"
- Should include conditional logic (if/else)
- Object counting, conditional navigation/placement

**Level 3 - Multi-Step Planning with Recovery**: "Prepare a sandwich and bring it to the user, even if ingredients are missing"
- Should include error handling (missing items)
- Alternative plans, user notification

For each level:
- Write the system prompt defining available primitives, reasoning format, constraints
- Write the user prompt with the task description
- Test with at least 3 variations of the task
- Analyze failure modes and refine prompts

Use chain-of-thought prompting and require the LLM to output:
1. Analysis: Key objects, preconditions, goals
2. Plan: Numbered action sequence
3. Contingencies: Alternative plans for likely failures

#### Deliverable

- Three complete prompt templates (system + user prompts)
- Python script using OpenAI API to test prompts (`test_prompts.py`)
- Results table showing 3 variations per level with LLM outputs
- Failure analysis: at least 3 identified failure modes with proposed fixes
- Refined prompts incorporating fixes

#### Assessment Criteria

1. **Prompt Design**: Prompts elicit structured, valid action sequences (30%)
2. **Complexity Handling**: Level 3 prompts successfully handle conditionals and error cases (25%)
3. **Testing Rigor**: Comprehensive testing across variations (20%)
4. **Failure Analysis**: Insightful identification and resolution of failure modes (15%)
5. **Implementation**: Working code producing valid JSON outputs (10%)

#### Chapter References

- Chapter 5: LLM Cognitive Planning (Sections 5.2, 5.3, 5.4)
- Chapter 7: Prompt Engineering for Robotics
- Examples: LLM Planner Node (Example 2)

---

### Exercise 8: Implement Action Executor with Variable Binding

**Difficulty**: Application
**Time Estimate**: 2.5 hours
**Prerequisites**: Exercise 4, ROS 2, Python

#### Prompt

Implement a robust action executor that handles variable binding, such as `$detected_object`, `$user_location`, and supports complex parameter resolution.

Required features:

1. **Variable Binding Engine**:
   - Store variables in persistent context (key-value store)
   - Support scoped variables: global, task-level, action-level
   - Resolve nested references: `grasp(object_id: "$detected_objects[0]")`

2. **Type Checking**:
   - Validate parameter types before execution (string, int, float, Pose, etc.)
   - Provide informative error messages on type mismatches

3. **Dependency Resolution**:
   - Detect when action depends on output from prior action
   - Ensure prerequisites are satisfied before execution

4. **Timeout Management**:
   - Per-action timeout configuration (default 30s, configurable per action type)
   - Graceful handling of timeouts with status reporting

5. **Retry Logic**:
   - Exponential backoff for transient failures
   - Max retry limit (3 attempts default)
   - Different retry strategies per failure type

Test with action sequence:
```
navigate(location: "kitchen")
detect_object(object_type: "cup", store_as: "$cup_id")
grasp(object_id: "$cup_id")
navigate(location: "$user_location")
place(location: "table")
```

#### Deliverable

- `ActionExecutorEnhanced` class implementation (500-600 lines)
- `VariableContext` class for variable management
- Unit tests for variable resolution, type checking, dependency detection
- Integration test demonstrating end-to-end execution
- Documentation: variable syntax guide, timeout configuration examples

#### Assessment Criteria

1. **Variable Binding**: Correct implementation of storage, retrieval, resolution (25%)
2. **Type Safety**: Robust type checking with informative errors (20%)
3. **Error Handling**: Proper timeout and retry logic (20%)
4. **Testing**: Comprehensive unit and integration tests (20%)
5. **Code Quality**: Clean architecture, well-documented (15%)

#### Chapter References

- Chapter 6: Action Execution (Sections 6.2, 6.3, 6.4)
- Examples: Action Executor (Example 3)

---

### Exercise 9: Test Voice Commands for Mobile Manipulator

**Difficulty**: Application
**Time Estimate**: 2 hours
**Prerequisites**: Exercises 6, 7, 8

#### Prompt

Create a comprehensive test suite for voice-commanded mobile manipulation tasks. Your system should integrate Whisper (from Exercise 6), LLM planner (from Exercise 7), and action executor (from Exercise 8).

Implement 10 test scenarios covering:

**Navigation Tasks** (2 scenarios):
- "Go to the kitchen"
- "Navigate to the charging station and wait"

**Manipulation Tasks** (3 scenarios):
- "Pick up the red cup"
- "Open the drawer and get the spoon"
- "Place the book on the shelf"

**Combined Tasks** (3 scenarios):
- "Bring me water from the fridge"
- "Clear the table and put dishes in the sink"
- "Find my phone and bring it to me"

**Error Handling** (2 scenarios):
- "Get the blue vase" (object doesn't exist)
- "Go to the garage" (location not in map)

For each scenario:
- Provide voice input (WAV file or text simulation)
- Expected transcription
- Expected action plan (JSON)
- Expected execution outcome (success/failure with reason)
- Actual results from your integrated system

Measure:
- End-to-end latency (voice → task completion)
- Success rate (correctly completed tasks)
- Error recovery effectiveness

#### Deliverable

- Test suite implementation (`test_vla_integration.py`)
- 10 test case definitions with expected/actual results
- Test report with metrics table (latency, success rate per category)
- Video or ROS bag demonstrating 3 successful scenarios
- Failure analysis: why did tasks fail, how to improve

#### Assessment Criteria

1. **Test Coverage**: All 10 scenarios properly implemented (25%)
2. **Integration**: Successful integration of Whisper, LLM, executor (25%)
3. **Metrics Collection**: Accurate latency and success rate measurement (20%)
4. **Error Handling**: System gracefully handles error scenarios (15%)
5. **Documentation**: Clear test reports and analysis (15%)

#### Chapter References

- Chapter 8: System Integration (Section 8.2)
- Chapter 9: Testing and Validation
- Examples: Complete VLA Pipeline (Example 4)

---

### Exercise 10: Benchmark VLA Pipeline Latency

**Difficulty**: Application
**Time Estimate**: 2 hours
**Prerequisites**: Exercise 5, ROS 2, profiling tools

#### Prompt

Conduct a detailed performance benchmark of your VLA pipeline, identifying bottlenecks and optimization opportunities.

Tasks:

1. **Instrument Pipeline Stages**:
   - Add timing measurements to each stage (VAD, Whisper, LLM, action execution)
   - Publish timing data to `/vla_performance` topic
   - Log timestamps for: audio received, transcription complete, plan generated, action started, action completed

2. **Test Conditions**:
   - Local LLM (Ollama/Llama-2-7B) vs. Cloud API (GPT-4)
   - Whisper models: tiny, base, small, medium
   - Simple tasks (1-3 actions) vs. complex tasks (8-12 actions)
   - Cold start vs. warm cache

3. **Metrics**:
   - Mean, median, p95, p99 latency per stage
   - Total end-to-end latency distribution
   - Throughput (tasks per minute)
   - CPU/memory usage per component

4. **Optimization Experiment**:
   - Implement at least 2 optimizations (e.g., caching, streaming, parallel execution)
   - Measure impact on latency and accuracy

#### Deliverable

- Instrumented pipeline code with timing hooks
- Benchmarking script (`benchmark_vla.py`) running 50+ test cases
- Performance report with:
  - Latency breakdown charts (box plots, histograms)
  - Comparison tables (local vs. cloud LLM, Whisper model sizes)
  - Optimization results (before/after metrics)
- Recommendations for production deployment

#### Assessment Criteria

1. **Instrumentation**: Accurate timing measurement without disrupting operation (25%)
2. **Experimental Design**: Comprehensive test matrix covering key variables (25%)
3. **Analysis**: Insightful interpretation of results, correct bottleneck identification (25%)
4. **Optimization**: Meaningful performance improvement (>20% latency reduction or accuracy gain) (15%)
5. **Reporting**: Professional presentation with clear visualizations (10%)

#### Chapter References

- Chapter 8: System Integration (Section 8.3 on optimization)
- Chapter 10: Performance Analysis
- Examples: Complete VLA Pipeline (Example 4)

---

## 📕 Synthesis Exercises

### Exercise 11: Design End-to-End VLA System for Warehouse Robot

**Difficulty**: Synthesis
**Time Estimate**: 4 hours
**Prerequisites**: All Application exercises

#### Prompt

Design a complete VLA system for an autonomous warehouse robot that receives natural language commands from human operators (e.g., "Move 10 boxes from aisle B3 to staging area 2").

Your system must:

1. **Architecture**:
   - Define system components (perception, planning, execution, coordination)
   - Specify interfaces between components (ROS 2 topics, services, actions)
   - Design data flow and control flow

2. **Vision System**:
   - Object detection: boxes, pallets, forklifts, humans
   - Scene understanding: aisle structure, staging areas, navigation graph
   - Sensor suite: cameras, LiDAR, force-torque sensors

3. **Language Understanding**:
   - Parse quantity, object type, source, destination
   - Handle ambiguous references ("the blue boxes", "over there")
   - Support clarification questions

4. **Action Planning**:
   - Task decomposition: pick sequence, path planning, multi-trip handling
   - Constraint reasoning: load limits, collision avoidance, priority rules
   - Optimization: minimize travel time, energy consumption

5. **Execution and Monitoring**:
   - Real-time execution monitoring with progress updates
   - Error recovery: failed grasp, blocked path, wrong object
   - Human-in-the-loop for edge cases

6. **Safety and Compliance**:
   - Human detection and stopping
   - Geofencing (restricted areas)
   - Audit logging for accountability

Address:
- Scalability: support for 10+ concurrent commands
- Reliability: 99%+ task completion rate
- Latency: commands start executing within 5 seconds

#### Deliverable

- System architecture diagram (component, sequence, deployment views)
- Technical specification document (3000-4000 words) covering all 6 areas
- Pseudo-code or Python stubs for 3 critical components
- Risk analysis: top 5 failure modes with mitigation strategies
- Deployment plan: hardware requirements, software stack, testing strategy

#### Assessment Criteria

1. **Architecture Quality**: Well-structured, modular design with clear interfaces (25%)
2. **Technical Depth**: Detailed specifications grounded in real constraints (20%)
3. **Integration**: All components work together coherently (20%)
4. **Robustness**: Comprehensive error handling and safety measures (20%)
5. **Feasibility**: Realistic design implementable with current technology (15%)

#### Chapter References

- All chapters, especially Chapter 8 (Integration) and Chapter 11 (Case Studies)
- Examples: Complete VLA Pipeline (Example 4), Error Recovery (Example 5)

---

### Exercise 12: Analyze Failure Modes and Design Safety System

**Difficulty**: Synthesis
**Time Estimate**: 3 hours
**Prerequisites**: Exercise 11, safety engineering background

#### Prompt

Conduct a Failure Modes and Effects Analysis (FMEA) for a VLA-enabled service robot operating in a hospital environment (delivering medications, linens, meals).

Analysis scope:

1. **Identify Failure Modes** (at least 15):
   - Perception failures: misidentified object, missed obstacle, incorrect localization
   - Language failures: misunderstood command, ambiguous reference, hallucinated action
   - Planning failures: infeasible plan, violated constraint, resource deadlock
   - Execution failures: grasp failure, navigation collision, dropped payload

2. **For Each Failure Mode**:
   - Severity (1-10): impact on safety, mission, equipment
   - Likelihood (1-10): probability of occurrence
   - Detection (1-10): ability to detect before harm
   - Risk Priority Number (RPN = Severity × Likelihood × Detection)

3. **Design Safety System**:
   - Runtime monitors: sanity checks, anomaly detection
   - Fail-safe behaviors: emergency stop, human handoff, safe park
   - Redundancy: sensor fusion, multi-model voting, backup plans
   - Human oversight: teleoperation interface, approval workflows

4. **Safety Standards Compliance**:
   - ISO 13482 (Safety requirements for personal care robots)
   - Risk reduction measures per ISO 12100
   - Validation testing requirements

Prioritize mitigations for top 10 highest-RPN failure modes.

#### Deliverable

- FMEA table with 15+ failure modes (Excel or CSV)
- Safety system architecture diagram
- Technical specification for top 3 safety monitors (pseudo-code/Python)
- Compliance matrix mapping mitigations to ISO 13482 requirements
- Test plan for validating safety measures

#### Assessment Criteria

1. **FMEA Completeness**: Comprehensive identification of failure modes (25%)
2. **Risk Assessment**: Accurate severity, likelihood, detection ratings (20%)
3. **Safety System Design**: Effective mitigations addressing high-RPN modes (25%)
4. **Standards Compliance**: Proper mapping to ISO 13482/12100 (15%)
5. **Validation Plan**: Rigorous testing approach (15%)

#### Chapter References

- Chapter 12: Safety and Ethics
- Chapter 9: Testing and Validation
- Examples: Error Recovery (Example 5)

---

### Exercise 13: Compare GPT-4 vs. Open-Source LLMs for Robotics Planning

**Difficulty**: Synthesis
**Time Estimate**: 3.5 hours
**Prerequisites**: Exercise 7, access to multiple LLMs

#### Prompt

Conduct a rigorous comparison of LLM performance for robotic task planning. Evaluate:

**Models**:
- GPT-4 (proprietary, cloud API)
- Claude 3 (proprietary, cloud API)
- Llama-2-70B (open-source, local or cloud)
- Mistral-7B-Instruct (open-source, local)

**Evaluation Dimensions**:

1. **Plan Quality**:
   - Correctness: valid action sequences that achieve goals
   - Optimality: minimal number of actions
   - Safety: no dangerous action sequences
   - Robustness: handles edge cases, missing information

2. **Latency and Throughput**:
   - Time to first token
   - Total generation time
   - Tokens per second
   - Cost per 1000 requests

3. **Generalization**:
   - Few-shot learning: performance with 0, 1, 3, 5 examples
   - Novel tasks: performance on tasks not in training data
   - Ambiguity handling: clarifying questions vs. assumptions

4. **Practical Considerations**:
   - Deployment: cloud vs. edge, GPU requirements
   - Privacy: data sent to third party vs. on-premise
   - Cost: per-token pricing vs. one-time cost

**Test Suite**: 30 tasks across categories:
- Simple navigation (5 tasks)
- Manipulation (5 tasks)
- Multi-step combined (10 tasks)
- Ambiguous/underspecified (5 tasks)
- Safety-critical (5 tasks)

Use consistent prompts across models. Measure all dimensions quantitatively.

#### Deliverable

- Evaluation framework code (`evaluate_llms.py`) with 30 test tasks
- Results database (CSV/JSON) with all measurements
- Comparative analysis report (2000-2500 words):
  - Summary table with scores across dimensions
  - Statistical analysis (mean, std dev, significance tests)
  - Qualitative failure analysis with examples
  - Recommendations for different use cases
- Cost-benefit analysis spreadsheet

#### Assessment Criteria

1. **Experimental Rigor**: Fair comparison with controlled variables (25%)
2. **Comprehensive Evaluation**: All dimensions measured with appropriate metrics (25%)
3. **Statistical Analysis**: Proper use of statistics to assess significance (20%)
4. **Practical Insights**: Actionable recommendations grounded in data (20%)
5. **Reproducibility**: Clear methodology, shareable code/data (10%)

#### Chapter References

- Chapter 5: LLM Cognitive Planning (Section 5.5 on model selection)
- Chapter 10: Performance Analysis
- Examples: LLM Planner Node (Example 2)

---

### Exercise 14: Design Multi-Robot VLA Coordination System

**Difficulty**: Synthesis
**Time Estimate**: 4 hours
**Prerequisites**: Exercise 11, distributed systems knowledge

#### Prompt

Design a VLA system for coordinating a fleet of 5 heterogeneous robots (2 mobile manipulators, 2 AMRs, 1 quadruped) in a dynamic warehouse environment. The system receives high-level commands like "Inventory aisle C by 3pm" and must decompose, assign, and coordinate execution across robots.

System requirements:

1. **Hierarchical Planning**:
   - Top-level LLM planner: decomposes warehouse-wide tasks
   - Task allocation: assigns subtasks to robots based on capabilities, location, workload
   - Robot-level planners: each robot generates detailed action plans

2. **Coordination Mechanisms**:
   - Conflict resolution: path deconfliction, resource contention (chargers, elevators)
   - Dynamic reallocation: handle robot failures, priority changes
   - Synchronization: coordinated manipulation (two robots lifting heavy box)

3. **Communication Architecture**:
   - Centralized coordinator vs. distributed consensus
   - Message formats: task assignments, status updates, coordination requests
   - Failure handling: network partition, coordinator failure

4. **Optimization**:
   - Minimize total completion time
   - Balance workload across robots
   - Minimize energy consumption, travel distance

5. **Monitoring and Visualization**:
   - Real-time dashboard showing robot states, task progress
   - Alerts for failures, bottlenecks
   - Audit trail for accountability

Constraints:
- No single point of failure (coordinator can fail, system continues degraded)
- 10ms latency for collision avoidance coordination
- Support for adding/removing robots dynamically

#### Deliverable

- System architecture document (3500-4000 words):
  - Component diagram
  - Sequence diagrams for key workflows (task assignment, conflict resolution)
  - Data models for tasks, robots, world state
- Coordination protocol specification (pseudo-code or state machines)
- Optimization algorithm design (high-level, not full implementation)
- Failure scenario analysis: 5 scenarios with recovery strategies
- Simulation plan: how to validate design before deployment

#### Assessment Criteria

1. **Architectural Sophistication**: Elegant design handling complex coordination (25%)
2. **Scalability**: Design scales to 10+ robots without fundamental changes (20%)
3. **Robustness**: Graceful degradation under failures (20%)
4. **Optimization**: Effective resource allocation strategies (15%)
5. **Practical Viability**: Implementable with ROS 2 and existing tools (20%)

#### Chapter References

- Chapter 13: Multi-Robot Coordination
- Chapter 8: System Integration
- Examples: Complete VLA Pipeline (Example 4)

---

### Exercise 15: VLA Capstone - Deploy End-to-End System

**Difficulty**: Synthesis
**Time Estimate**: 6-8 hours
**Prerequisites**: All previous exercises, physical robot or high-fidelity simulator

#### Prompt

Deploy and evaluate a complete VLA system on a physical robot or high-fidelity simulator (Gazebo/Isaac Sim). Demonstrate end-to-end functionality from voice command to task execution.

**Scenario**: Personal service robot in apartment

**Required Capabilities**:
1. Voice command understanding (Whisper integration)
2. Natural language task planning (LLM integration)
3. Object detection and localization (YOLO + depth)
4. Navigation in cluttered environment (Nav2)
5. Mobile manipulation (MoveIt + gripper control)
6. Error recovery and replanning

**Demonstration Tasks** (complete at least 3):
1. "Bring me the water bottle from the kitchen table"
2. "Clean up the living room and put toys in the box"
3. "Find my keys and tell me where they are"
4. "Set the table for two people"
5. "Open the fridge and check if we have milk"

**System Requirements**:
- Runs on real hardware or validated simulator
- Full ROS 2 integration with all components
- Real-time execution (no manual intervention during task)
- Comprehensive logging for post-analysis
- Graceful failure handling (doesn't crash on errors)

**Evaluation**:
- Task success rate (target: 80%+ for simple tasks, 60%+ for complex)
- Average completion time per task type
- Failure mode analysis: why did tasks fail?
- Latency breakdown: perception, planning, execution
- Qualitative assessment: user experience, naturalness

**Documentation**:
- System setup guide (hardware, software, configuration)
- User manual for voice commands
- Troubleshooting guide
- Demo video (3-5 minutes) showing successful tasks

#### Deliverable

- Complete ROS 2 workspace with all source code
- Launch files for full system bring-up
- Configuration files (parameters, maps, semantic annotations)
- Evaluation report (2500-3000 words):
  - System description
  - Test results with metrics
  - Failure analysis
  - Lessons learned
  - Future improvements
- Demo video with narration
- GitHub repository with README, installation instructions

#### Assessment Criteria

1. **System Integration**: All components working together seamlessly (25%)
2. **Functionality**: Successfully demonstrates required capabilities (25%)
3. **Performance**: Meets latency, success rate targets (20%)
4. **Robustness**: Handles failures gracefully, provides useful feedback (15%)
5. **Documentation**: Comprehensive, enables others to reproduce (15%)

#### Chapter References

- All chapters
- All examples (Examples 1-5)

---

## Summary

These 15 exercises provide a structured progression through VLA concepts:

### 📘 Recall (Exercises 1-5)
Focus on understanding core concepts: architecture patterns, accuracy metrics, prompting strategies, action primitive design, and latency analysis.

### 📗 Application (Exercises 6-10)
Hands-on implementation: enhancing Whisper nodes, crafting LLM prompts, building robust executors, integration testing, and performance benchmarking.

### 📕 Synthesis (Exercises 11-15)
Advanced system design: warehouse robot VLA system, safety analysis, LLM comparison studies, multi-robot coordination, and capstone deployment.

**Progression Strategy**:
- Complete Recall exercises to build foundational knowledge
- Tackle Application exercises to develop implementation skills
- Attempt Synthesis exercises to integrate and extend concepts

**Time Commitment**:
- Recall: ~2.5 hours total
- Application: ~12 hours total
- Synthesis: ~20-24 hours total

**Learning Outcomes**:
By completing these exercises, you will:
- Understand VLA architecture trade-offs and design patterns
- Implement production-grade speech recognition and language planning
- Build robust action execution with error handling
- Evaluate and optimize VLA system performance
- Design and deploy complete Physical AI systems for real-world applications

Good luck with your VLA journey!
