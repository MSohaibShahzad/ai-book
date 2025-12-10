# Data Model: Physical-AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-robotics-textbook | **Date**: 2025-12-09

## Overview

This document defines the core entities, their attributes, relationships, and validation rules for the Physical-AI & Humanoid Robotics textbook content structure. These entities map directly to markdown files and Docusaurus configuration.

---

## Entity: Module

**Description**: A major learning unit covering one conceptual pillar (ROS 2, Digital Twin, Isaac, VLA). Contains chapters, learning objectives, outcomes, and exercises.

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `id` | string | Yes | Unique identifier (e.g., `01-foundations-ros2`) | Regex: `^\d{2}-[a-z0-9-]+$` |
| `title` | string | Yes | Full module title (e.g., "Foundations: ROS 2 Middleware") | Max 100 chars |
| `slug` | string | Yes | URL-friendly identifier (e.g., `foundations-ros2`) | Kebab-case, matches `id` suffix |
| `sidebar_label` | string | Yes | Concise navigation label (e.g., "Module 1: ROS 2") | Max 30 chars |
| `description` | string | Yes | 1-2 sentence summary | Max 200 chars |
| `sequence` | integer | Yes | Module order (1-4) | Range: 1-4 |
| `learning_outcomes` | array<string> | Yes | 3-5 measurable objectives | Min 3, max 5 items |
| `prerequisites` | array<string> | No | Prior knowledge required | Each item max 100 chars |
| `chapters` | array<Chapter> | Yes | Ordered list of chapters | Min 3, max 5 chapters |
| `references` | array<string> | No | Recommended reading | Each item max 200 chars |

### Relationships

- **Contains**: 3-5 Chapter entities
- **References**: 0-N Diagram entities (via chapters)
- **Includes**: 1 Examples entity (runnable code examples)
- **Includes**: 1 Exercises entity (3 exercises per chapter)
- **Includes**: 1 ExerciseAnswers entity (instructor solutions)

### Validation Rules

- `sequence` must be unique across all modules
- `id` must start with zero-padded sequence (e.g., `01-`, `02-`)
- Total chapters across all modules must be 12-16 (per specification constraint)
- Each module must have `index.md` file in `docs/{id}/` directory

### Example

```yaml
# docs/01-foundations-ros2/index.md frontmatter
---
title: "Foundations: ROS 2 Middleware"
slug: foundations-ros2
sidebar_label: "Module 1: ROS 2"
toc: true
description: "Understanding the middleware backbone of humanoid robots through ROS 2 concepts: nodes, topics, services, and actions."
---
```

---

## Entity: Chapter

**Description**: A subdivision within a module focusing on a specific concept (e.g., "Why ROS 2 Nodes Matter").

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `id` | string | Yes | Unique identifier within module (e.g., `01-what-is-ros2`) | Regex: `^\d{2}-[a-z0-9-]+\.md$` |
| `title` | string | Yes | Full chapter title | Max 100 chars |
| `slug` | string | Yes | URL-friendly identifier | Kebab-case |
| `sidebar_label` | string | Yes | Concise navigation label | Max 50 chars |
| `description` | string | Yes | 1-2 sentence summary | Max 200 chars |
| `sequence` | integer | Yes | Chapter order within module (1-5) | Range: 1-5 |
| `module_id` | string | Yes | Parent module identifier | Foreign key to Module.id |
| `learning_objectives` | array<string> | No | 2-3 objectives for this chapter | Max 3 items |
| `diagrams` | array<string> | No | Referenced diagram filenames | Each item references Diagram.filename |

### Relationships

- **Belongs to**: 1 Module entity
- **References**: 0-N Diagram entities
- **Associated with**: 3 Exercise entities (defined in module's exercises.md)

### Validation Rules

- `sequence` must be unique within parent module
- `id` must start with zero-padded sequence (e.g., `01-`, `02-`)
- Filename must match pattern: `{sequence}-{slug}.md`
- File must exist in `docs/{module_id}/` directory

### Example

```yaml
# docs/01-foundations-ros2/01-what-is-ros2.md frontmatter
---
title: "What is ROS 2? The Robot Operating System Explained"
slug: what-is-ros2
sidebar_label: "1.1 What is ROS 2?"
toc: true
description: "Explore the purpose of ROS 2 as the middleware nervous system for humanoid robots, enabling communication between sensors, controllers, and actuators."
---
```

---

## Entity: LearningOutcome

**Description**: A measurable objective that students should achieve after completing a module (e.g., "Explain the purpose of ROS 2 middleware").

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `id` | string | Yes | Unique identifier (e.g., `ros2-lo-01`) | Regex: `^[a-z0-9-]+-lo-\d{2}$` |
| `description` | string | Yes | Measurable learning objective | Max 200 chars, starts with action verb |
| `module_id` | string | Yes | Parent module identifier | Foreign key to Module.id |
| `bloom_level` | enum | Yes | Bloom's Taxonomy level | Values: remember, understand, apply, analyze, evaluate, create |
| `assessment_method` | string | No | How outcome is measured | Max 100 chars |

### Relationships

- **Belongs to**: 1 Module entity
- **Assessed by**: 0-N Exercise entities

### Validation Rules

- `description` must start with action verb (e.g., "Explain", "Design", "Compare")
- Each module should have 3-5 learning outcomes
- Learning outcomes should progress through Bloom's levels (remember → create)

### Example

```yaml
# In docs/01-foundations-ros2/index.md
learning_outcomes:
  - id: ros2-lo-01
    description: "Explain the purpose of ROS 2 as middleware for robot control systems"
    bloom_level: understand
    assessment_method: "Exercises 1.1.1, 1.2.1"
  - id: ros2-lo-02
    description: "Diagram node communication flows for sensor data processing"
    bloom_level: apply
    assessment_method: "Exercises 1.2.3, 1.3.2"
```

---

## Entity: Diagram

**Description**: Visual aids illustrating system architecture, data flows, or component relationships without implementation details.

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `filename` | string | Yes | Diagram file (e.g., `ros2-node-communication.svg`) | Regex: `^[a-z0-9-]+\.svg$` |
| `title` | string | Yes | Descriptive title | Max 100 chars |
| `alt_text` | string | Yes | Accessibility description | Max 300 chars |
| `category` | enum | Yes | Diagram type | Values: system-architecture, process-flow, conceptual |
| `referenced_by` | array<string> | No | Chapters using this diagram | Each item references Chapter.id |
| `source_file` | string | No | Original editable file (e.g., `.drawio` file) | Filename in diagrams/ folder |

### Relationships

- **Referenced by**: 0-N Chapter entities
- **Stored in**: `docs/diagrams/` directory

### Validation Rules

- All diagrams must be SVG format
- `alt_text` required for accessibility
- Diagrams must use grayscale (black, white, shades of gray)
- Pattern fills (dots, stripes) for differentiation
- File must exist in `docs/diagrams/` directory

### Example

```yaml
# Metadata for docs/diagrams/ros2-node-communication.svg
filename: ros2-node-communication.svg
title: "ROS 2 Node Communication Architecture"
alt_text: "Diagram showing three ROS 2 nodes (Sensor Publisher, Controller Subscriber, Actuator Action Server) connected via topics and action interfaces. Arrows indicate data flow from sensor to controller to actuator."
category: system-architecture
referenced_by:
  - 01-what-is-ros2
  - 02-nodes-topics-services
source_file: ros2-node-communication.drawio
```

---

## Entity: Exercise

**Description**: Assessment questions aligned with chapter learning outcomes, structured by Bloom's Taxonomy difficulty.

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `id` | string | Yes | Unique identifier (e.g., `ex-01-01-01`) | Regex: `^ex-\d{2}-\d{2}-\d{2}$` (module-chapter-exercise) |
| `chapter_id` | string | Yes | Parent chapter identifier | Foreign key to Chapter.id |
| `sequence` | integer | Yes | Exercise order within chapter (1-3) | Range: 1-3 |
| `tier` | enum | Yes | Difficulty tier | Values: recall, application, synthesis |
| `question` | string | Yes | Exercise prompt | Max 500 chars |
| `hints` | array<string> | No | Optional hints for students | Max 3 hints, each max 200 chars |
| `answer_reference` | string | Yes | Pointer to instructor solution | Format: `.answers/exercises-answers.md#ex-{id}` |

### Relationships

- **Belongs to**: 1 Chapter entity
- **Assesses**: 1-N LearningOutcome entities
- **Has solution in**: 1 ExerciseAnswers entity

### Validation Rules

- Each chapter must have exactly 3 exercises (tiers: recall, application, synthesis)
- `tier` must progress in difficulty (Tier 1 = recall, Tier 2 = application, Tier 3 = synthesis)
- `id` format: `ex-{module_seq}-{chapter_seq}-{exercise_seq}`
- Solution must exist in corresponding `.answers/exercises-answers.md` file

### Example

```yaml
# In docs/01-foundations-ros2/exercises.md
exercises:
  - id: ex-01-01-01
    chapter_id: 01-what-is-ros2
    sequence: 1
    tier: recall
    question: "Explain in your own words why ROS 2 is described as the 'nervous system' of a robot."
    answer_reference: ".answers/exercises-answers.md#ex-01-01-01"

  - id: ex-01-01-02
    chapter_id: 01-what-is-ros2
    sequence: 2
    tier: application
    question: "Design a simple node communication flow for a humanoid robot that must read IMU sensor data and adjust balance. Identify which nodes would publish and subscribe to which topics."
    hints:
      - "Consider one node for the IMU sensor and another for the balance controller"
      - "Think about the data direction: sensor → controller"
    answer_reference: ".answers/exercises-answers.md#ex-01-01-02"
```

---

## Entity: ExerciseAnswers

**Description**: Instructor solutions for exercises, stored separately from student-facing content.

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `exercise_id` | string | Yes | Reference to Exercise entity | Foreign key to Exercise.id |
| `solution` | string | Yes | Instructor solution | Max 1000 chars |
| `explanation` | string | No | Additional pedagogical notes | Max 500 chars |
| `rubric` | array<string> | No | Grading criteria | Max 5 items, each max 200 chars |

### Relationships

- **Corresponds to**: 1 Exercise entity
- **Stored in**: `.answers/exercises-answers.md` within module directory

### Validation Rules

- File must be in `.answers/` subdirectory (hidden from student sidebar)
- Each exercise in module must have corresponding answer
- Solutions must reference learning outcomes being assessed

### Example

```markdown
# In docs/01-foundations-ros2/.answers/exercises-answers.md

## Exercise 1.1.1 (ex-01-01-01) {#ex-01-01-01}

**Question**: Explain in your own words why ROS 2 is described as the 'nervous system' of a robot.

**Solution**:
ROS 2 is called the "nervous system" because it functions like the communication network in a biological organism. Just as the nervous system transmits signals between the brain, sensors (eyes, ears), and muscles, ROS 2 transmits data between a robot's perception systems (cameras, LiDAR), decision-making nodes (controllers, planners), and actuators (motors, servos). It enables distributed components to coordinate without tight coupling, similar to how neurons communicate via synapses.

**Explanation**:
This answer demonstrates understanding of the middleware abstraction (LO: ros2-lo-01). Look for students to connect ROS 2's communication mechanisms (topics, services) to biological analogies showing conceptual grasp beyond rote memorization.

**Rubric**:
- Mentions communication/coordination role (3 points)
- Connects to biological nervous system analogy (2 points)
- References sensors → controllers → actuators flow (2 points)
- Uses specific ROS 2 terms (nodes, topics) (3 points)
```

---

## Entity: CodeExample

**Description**: Runnable or conceptual code snippets demonstrating module concepts.

### Attributes

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `id` | string | Yes | Unique identifier (e.g., `ros2-ex-01`) | Regex: `^[a-z0-9-]+-ex-\d{2}$` |
| `module_id` | string | Yes | Parent module identifier | Foreign key to Module.id |
| `title` | string | Yes | Example title | Max 100 chars |
| `language` | enum | Yes | Programming language | Values: python, pseudocode, yaml |
| `code` | string | Yes | Code snippet | Max 2000 chars |
| `annotations` | array<string> | No | Inline comment explanations | Each item max 200 chars |
| `runnable` | boolean | Yes | Whether code can execute | Default: false |
| `environment_link` | string | No | Link to cloud environment (Colab, Binder) | Valid URL if runnable = true |

### Relationships

- **Belongs to**: 1 Module entity
- **Stored in**: `examples.md` within module directory

### Validation Rules

- If `runnable = true`, `environment_link` must be provided
- Code must include necessary imports and setup context
- Annotations must explain "why" not just "what"

### Example

```yaml
# In docs/01-foundations-ros2/examples.md
examples:
  - id: ros2-ex-01
    module_id: 01-foundations-ros2
    title: "Minimal ROS 2 Publisher Node (Conceptual)"
    language: python
    code: |
      import rclpy
      from rclpy.node import Node
      from std_msgs.msg import String

      class MinimalPublisher(Node):
          def __init__(self):
              super().__init__('minimal_publisher')  # Create node with name
              self.publisher_ = self.create_publisher(String, 'topic', 10)  # Create publisher
              self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5s

          def timer_callback(self):
              msg = String()
              msg.data = 'Hello, ROS 2!'
              self.publisher_.publish(msg)  # Send message to topic

      rclpy.init()
      node = MinimalPublisher()
      rclpy.spin(node)  # Keep node running
    runnable: false
    annotations:
      - "This demonstrates the fundamental publish-subscribe pattern in ROS 2"
      - "The timer callback simulates periodic sensor data publishing"
```

---

## Entity Relationship Diagram

```
Module (1) ──── contains ──── (3-5) Chapter
  │
  ├── has ──── (3-5) LearningOutcome
  │
  ├── includes ──── (1) Examples [contains N CodeExample]
  │
  └── includes ──── (1) Exercises [contains 3×N Exercise]
                          │
                          └── has_solution_in ──── (1) ExerciseAnswers

Chapter (1) ──── references ──── (0-N) Diagram

Exercise (1) ──── assesses ──── (1-N) LearningOutcome
```

---

## YAML Frontmatter Schema Summary

All markdown files must include frontmatter with these required fields:

```yaml
---
title: string              # Full descriptive title
slug: string               # URL-friendly identifier (kebab-case)
sidebar_label: string      # Concise navigation label
toc: true                  # Enable table of contents (always true)
description: string        # 1-2 sentence summary
---
```

Additional optional fields per entity type:

- **Module index.md**: `learning_outcomes`, `prerequisites`, `references`
- **Chapter .md**: `learning_objectives`, `diagrams`
- **Preface .md**: `author`, `version`, `date`

See `contracts/module-structure.schema.json` for complete JSON Schema validation.
