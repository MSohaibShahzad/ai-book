---
title: "Module 1: Foundations (ROS 2)"
slug: /foundations-ros2
sidebar_label: "Module 1 Overview"
toc: true
description: "Introduction to ROS 2 middleware fundamentals for humanoid robotics, covering nodes, topics, services, URDF modeling, and AI integration"
---

# Module 1: Foundations (ROS 2)

## Module Overview

Welcome to Module 1, where you'll learn the middleware layer that powers modern humanoid robots. **ROS 2 (Robot Operating System 2)** is not an operating system—it's a flexible framework for writing robot software that enables communication between sensors, actuators, and AI systems.

By the end of this module, you'll understand how to architect robot control systems using distributed nodes, publish sensor data to topics, call services for synchronous operations, and define robot geometries using URDF. You'll also learn how to bridge AI agents (like LLMs) to ROS 2 for cognitive planning.

### Learning Objectives

After completing this module, you will be able to:

1. **Explain** the role of middleware in robotics and why ROS 2 exists
2. **Create** ROS 2 nodes that communicate via topics, services, and actions
3. **Model** a humanoid robot using URDF with correct kinematics and collision geometry
4. **Integrate** AI agents with ROS 2 for high-level cognitive planning
5. **Debug** ROS 2 systems using command-line tools and introspection utilities

### Prerequisites

Before starting this module, ensure you have:

- ✅ Ubuntu 22.04 with ROS 2 Humble installed (see [Prerequisites](../preface/prerequisites.md))
- ✅ Basic Python programming skills (functions, classes, file I/O)
- ✅ Familiarity with terminal commands (`cd`, `ls`, running scripts)
- ✅ Basic understanding of coordinate systems and transformations

**Time commitment**: 15-20 hours (reading + exercises)

### Why ROS 2 Matters for Humanoid Robotics

**The challenge**: A humanoid robot has dozens of sensors (cameras, LiDAR, IMUs, force sensors) and actuators (joint motors, grippers) that must coordinate in real-time. How do you architect software so that:

- The vision system can detect objects and notify the planner?
- The planner can command the arm controller without tight coupling?
- Components can be developed independently and hot-swapped?
- The system scales from simulation to real hardware?

**The solution**: ROS 2 provides:

1. **Communication primitives**: Topics (async streaming), services (request-response), actions (long-running tasks)
2. **Hardware abstraction**: Same code runs in simulation (Gazebo) and on real robots
3. **Tooling ecosystem**: Visualization (RViz), diagnostics, logging, and introspection
4. **Interoperability**: Language-agnostic (Python, C++, others via DDS)

**Real-world example**: Boston Dynamics' Spot, Clearpath's mobile robots, and many humanoid research platforms use ROS/ROS 2 as their core middleware.

### Module Structure

This module consists of five chapters, designed to be completed sequentially:

#### Chapter 1: What is ROS 2?
**WHY before HOW**: Understand the problems ROS 2 solves before learning syntax.

- The middleware layer in robot architectures
- Nodes, packages, and workspaces
- DDS: The communication backbone
- ROS 2 vs. ROS 1 (and why we use ROS 2)

**Hands-on**: Install ROS 2 Humble and run your first node

#### Chapter 2: Nodes, Topics, and Services
**Core communication patterns** for building modular robot systems.

- Publishing sensor data to topics
- Subscribing to process incoming data
- Quality of Service (QoS) for reliability
- Synchronous services for request-response

**Hands-on**: Build a publisher-subscriber pair and a simple service

#### Chapter 3: Actions and Parameters
**Advanced communication** for long-running tasks and runtime configuration.

- Actions for tasks with feedback (e.g., "move to position")
- Parameter servers for dynamic reconfiguration
- Lifecycle nodes for state management

**Hands-on**: Create an action server and reconfigure a node via parameters

#### Chapter 4: URDF and Robot Models
**Defining robot geometry** for simulation and control.

- URDF structure: links, joints, collision/visual geometry
- XACRO for reducing repetition
- Publishing robot state to `/robot_description`
- Coordinate frames and `tf2` transforms

**Hands-on**: Model a simplified humanoid arm in URDF

#### Chapter 5: Bridging AI Agents to ROS 2
**Integrating LLMs** and AI planners with robotic systems.

- Voice-to-action pipelines
- LLM-to-ROS message translation
- Safety constraints and action validation
- Example: "Pick up the red block" → ROS 2 action

**Hands-on**: Connect a simple LLM prompt to a ROS 2 action

### Pedagogical Approach

Each chapter follows this structure:

1. **Motivation (WHY)**: The problem this technology solves
2. **Concepts (WHAT)**: Mental models and terminology
3. **Implementation (HOW)**: Step-by-step tutorials with code
4. **Examples**: Worked implementations you can run
5. **Exercises**: Recall, Application, Synthesis tiers

**Active learning tip**: Type code yourself rather than copy-pasting. Introduce intentional bugs and fix them—debugging builds competency faster than reading perfect examples.

### Key Concepts Preview

Before diving into chapters, familiarize yourself with these terms (detailed explanations in chapters):

**Node**: An independent process that performs computation (e.g., "camera_driver", "obstacle_detector")

**Topic**: A named bus for asynchronous message streaming (e.g., `/camera/image`, `/joint_states`)

**Service**: A synchronous request-response mechanism (e.g., `/reset_odometry`)

**Action**: A long-running task with feedback and cancellation (e.g., `/navigate_to_goal`)

**Package**: A collection of nodes, launch files, and configuration (analogous to a library)

**Workspace**: A directory containing one or more packages (your development environment)

**URDF**: XML format describing robot kinematics and geometry

**tf2**: Library for managing coordinate frame transformations over time

### Module Outcomes

By completing this module, you will have:

- ✅ A working ROS 2 development environment
- ✅ Hands-on experience with topics, services, and actions
- ✅ A URDF model of a humanoid arm
- ✅ A prototype LLM-to-ROS integration
- ✅ Debugging skills for distributed robotic systems

**Next modules build on this foundation**:
- **Module 2**: Simulating your URDF robots in Gazebo/Unity
- **Module 3**: Integrating NVIDIA Isaac for perception
- **Module 4**: Full voice-language-action pipelines

### Common Pitfalls to Avoid

As you work through this module, watch out for:

1. **Not sourcing setup.bash**: ROS 2 commands won't work until you run `source /opt/ros/humble/setup.bash`
2. **Forgetting to build**: After editing code, run `colcon build` before running nodes
3. **Mismatched message types**: Publishers and subscribers must use identical message types
4. **Ignoring QoS settings**: Default QoS doesn't work for all scenarios (especially with Gazebo)
5. **Skipping URDF validation**: Invalid URDF causes cryptic errors later; validate early with `check_urdf`

### Getting Help

**Stuck on a concept or exercise?**

1. **Check the Glossary**: [Appendix - Glossary](../appendices/glossary.md) defines all ROS 2 terminology
2. **Search ROS Answers**: https://answers.ros.org/ (99% of errors have been encountered before)
3. **Review official docs**: https://docs.ros.org/en/humble/
4. **Ask on Discord**: Link in textbook repository
5. **Office hours**: Schedule via textbook website

### Assessment and Practice

**Self-check questions** (test your understanding before moving on):

- Can you explain what a ROS 2 node is in your own words?
- Do you understand the difference between a topic and a service?
- Can you describe a scenario where you'd use an action instead of a topic?

If you can't confidently answer these, re-read the relevant chapter sections.

**Exercise distribution** (per chapter):
- 2-3 Recall exercises (📘): Basic comprehension
- 2-3 Application exercises (📗): Hands-on implementation
- 1-2 Synthesis exercises (📕): Open-ended challenges

**Recommended pace**: 1 chapter per week (3-4 hours each)

### Connection to Real Humanoid Robots

The concepts in this module are directly used in:

- **Boston Dynamics Atlas**: URDF modeling, joint state publishing
- **PAL Robotics TALOS**: Full-body control via ROS 2 actions
- **Agility Robotics Digit**: Perception pipelines using ROS 2 topics
- **Research platforms**: Most university humanoid labs use ROS 2

**Your takeaway**: The skills learned here transfer directly to real-world humanoid systems, whether in research or industry.

### Ready to Begin?

Start with **[Chapter 1: What is ROS 2?](./01-what-is-ros2.md)** to understand the middleware layer that powers modern humanoid robots.

**Remember**: The goal isn't to memorize APIs—it's to understand the architectural patterns that make complex robot systems tractable. Take time to internalize the *why* before rushing to the *how*.

---

## Module Contents

1. [Chapter 1: What is ROS 2?](./01-what-is-ros2.md)
2. [Chapter 2: Nodes, Topics, and Services](./02-nodes-topics-services.md)
3. [Chapter 3: Actions and Parameters](./03-actions-and-parameters.md)
4. [Chapter 4: URDF and Robot Models](./04-urdf-and-robot-models.md)
5. [Chapter 5: Bridging AI Agents to ROS](./05-bridging-ai-agents-to-ros.md)
6. [Examples](./examples.md)
7. [Exercises](./exercises.md)

**Estimated completion time**: 15-20 hours

---

**Let's build the foundation for intelligent humanoid robotics.**
