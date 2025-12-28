---
title: "Further Reading"
slug: further-reading
sidebar_label: "Further Reading"
toc: true
description: "Recommended books, courses, tutorials, and resources for deepening Physical-AI and robotics knowledge"
---

# Further Reading

This page provides curated recommendations for deepening your knowledge beyond the textbook, organized by topic and difficulty level.

---

## General Robotics

### Textbooks

**Beginner**
- **Introduction to Autonomous Robots** by Correll, Hayes, Heckman, Roncone (2022)
  - Free online: https://github.com/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots
  - Covers fundamentals with accessible explanations and practical examples

**Intermediate**
- **Modern Robotics: Mechanics, Planning, and Control** by Lynch & Park (2017)
  - Companion videos and code: http://modernrobotics.org
  - Rigorous treatment of kinematics, dynamics, and control with worked examples

**Advanced**
- **Springer Handbook of Robotics** edited by Siciliano & Khatib (2016)
  - Comprehensive 2,000+ page reference covering all robotics subfields
  - Graduate-level depth; excellent for targeted deep dives

### Online Courses

1. **Underactuated Robotics** by Russ Tedrake (MIT OCW)
   - https://underactuated.mit.edu/
   - Advanced control theory for legged robots; great for humanoid dynamics

2. **Robotics Specialization** by University of Pennsylvania (Coursera)
   - https://www.coursera.org/specializations/robotics
   - 5-course series covering kinematics, planning, perception, estimation, and capstone

3. **Self-Driving Cars Specialization** by University of Toronto (Coursera)
   - https://www.coursera.org/specializations/self-driving-cars
   - Perception, localization, and planning for autonomous vehicles (applicable to mobile robots)

---

## ROS 2 and Middleware

### Books

- **A Concise Introduction to Robot Programming with ROS2** by Cañas, Matellán, Martínez-Gómez (2024)
  - Focused on ROS 2 Humble; hands-on tutorials with Nav2 and MoveIt2

- **ROS 2 Robot Programming** by Pyo, Cho, Jung (2023)
  - Covers TurtleBot 3, Gazebo integration, and real-world deployment

### Tutorials and Documentation

1. **Official ROS 2 Tutorials**
   - https://docs.ros.org/en/humble/Tutorials.html
   - Authoritative step-by-step guides from beginner to advanced topics

2. **The Construct (ROS Tutorials)**
   - https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/
   - Interactive online courses with cloud-based labs (some free, some paid)

3. **Articulated Robotics YouTube Channel**
   - https://www.youtube.com/c/ArticulatedRobotics
   - Excellent video tutorials on URDF, Gazebo, and ROS 2 best practices

---

## Simulation and Digital Twins

### Gazebo

1. **Gazebo Tutorials**
   - https://gazebosim.org/docs/fortress/tutorials
   - Official documentation with building custom worlds, sensor plugins, and physics tuning

2. **OSRF Simulation Webinars**
   - https://www.youtube.com/@OpenRobotics
   - Webinar recordings on advanced Gazebo features and case studies

### Unity for Robotics

1. **Unity Robotics Hub Documentation**
   - https://github.com/Unity-Technologies/Unity-Robotics-Hub
   - Comprehensive guides on ROS-Unity integration, URDF importers, and sensor simulation

2. **Unity Learn: Robotics Projects**
   - https://learn.unity.com/search?k=%5B%22q%3Arobotics%22%5D
   - Free interactive tutorials for building robot simulations in Unity

### Isaac Sim and Omniverse

1. **Isaac Sim Tutorials**
   - https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_index.html
   - Official NVIDIA tutorials covering photorealistic rendering, synthetic data, and Isaac ROS integration

2. **NVIDIA Deep Learning Institute: Isaac Sim**
   - https://www.nvidia.com/en-us/training/
   - Instructor-led courses on GPU-accelerated simulation (some paid)

3. **Omniverse Developer Blog**
   - https://developer.nvidia.com/blog/tag/omniverse/
   - Technical deep dives on USD pipelines, rendering optimization, and robotics workflows

---

## Computer Vision and Perception

### Books

- **Computer Vision: Algorithms and Applications** by Szeliski (2022)
  - Free draft: http://szeliski.org/Book/
  - Comprehensive textbook covering classical and deep learning-based vision

- **Multiple View Geometry in Computer Vision** by Hartley & Zisserman (2004)
  - The definitive reference for 3D reconstruction and SLAM

### Courses

1. **CS231n: Convolutional Neural Networks for Visual Recognition** (Stanford)
   - http://cs231n.stanford.edu/
   - Classic deep learning course with lecture notes and assignments

2. **First Principles of Computer Vision** (YouTube)
   - https://www.youtube.com/@firstprinciplesofcomputerv3258
   - In-depth video lectures on camera models, stereo vision, and structure from motion

### Tools and Frameworks

1. **OpenCV Documentation**
   - https://docs.opencv.org/4.x/
   - Essential library for image processing and classical computer vision

2. **PyTorch Tutorials**
   - https://pytorch.org/tutorials/
   - Official tutorials for training neural networks, including perception models

---

## SLAM and Localization

### Books

- **Probabilistic Robotics** by Thrun, Burgard, Fox (2005)
  - The foundational text for SLAM, particle filters, and Kalman filters
  - Accompanying code: http://www.probabilistic-robotics.org/

### Papers (Classic)

1. **Durrant-Whyte, H., & Bailey, T. (2006).** "Simultaneous localization and mapping: Part I." *IEEE Robotics & Automation Magazine*, 13(2), 99-110.
   - Two-part survey paper providing a comprehensive SLAM introduction

2. **Cadena, C., et al. (2016).** "Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age." *IEEE Transactions on Robotics*, 32(6), 1309-1332.
   - Modern SLAM survey covering visual SLAM and deep learning approaches

### Software

1. **RTAB-Map Tutorials**
   - https://github.com/introlab/rtabmap_ros
   - Real-time SLAM with ROS 2 integration

2. **ORB-SLAM3**
   - https://github.com/UZ-SLAMLab/ORB_SLAM3
   - State-of-the-art visual-inertial SLAM (monocular, stereo, RGB-D)

---

## Motion Planning and Control

### Books

- **Planning Algorithms** by LaValle (2006)
  - Free online: http://planning.cs.uiuc.edu/
  - Comprehensive treatment of path planning, RRT, PRM, and optimal control

- **Feedback Control of Dynamic Systems** by Franklin, Powell, Emami-Naeini (2014)
  - Classic control theory textbook; essential for understanding PID and state-space methods

### Courses

1. **Robotic Manipulation** by Russ Tedrake (MIT)
   - https://manipulation.mit.edu/
   - Grasp planning, trajectory optimization, and contact-rich manipulation

2. **Robot Dynamics and Control** by Matthew Mason (CMU)
   - https://www.cs.cmu.edu/~me/
   - Lagrangian mechanics and modern control for robotic systems

### Software

1. **MoveIt2 Tutorials**
   - https://moveit.picknik.ai/humble/index.html
   - Motion planning framework for ROS 2 with inverse kinematics and collision checking

2. **OMPL (Open Motion Planning Library)**
   - https://ompl.kavrakilab.org/
   - Sampling-based planning algorithms (used internally by MoveIt2)

---

## Machine Learning and AI

### Deep Learning

**Books**
- **Deep Learning** by Goodfellow, Bengio, Courville (2016)
  - Free online: https://www.deeplearningbook.org/
  - Comprehensive mathematical foundation for neural networks

- **Dive into Deep Learning** by Zhang, Lipton, Li, Smola (2023)
  - Free online: https://d2l.ai/
  - Interactive Jupyter notebooks with PyTorch/TensorFlow implementations

**Courses**
1. **Deep Learning Specialization** by Andrew Ng (Coursera)
   - https://www.coursera.org/specializations/deep-learning
   - 5-course series covering fundamentals, CNNs, RNNs, and practical ML engineering

2. **Practical Deep Learning for Coders** by fast.ai
   - https://course.fast.ai/
   - Top-down approach: build models first, understand theory later

### Reinforcement Learning

**Books**
- **Reinforcement Learning: An Introduction** by Sutton & Barto (2018)
  - Free online: http://incompleteideas.net/book/the-book.html
  - The definitive RL textbook with clear explanations and pseudocode

**Courses**
1. **CS 285: Deep Reinforcement Learning** (UC Berkeley)
   - https://rail.eecs.berkeley.edu/deeprlcourse/
   - Cutting-edge RL for robotics; lecture videos and assignments available

### LLMs and Transformers

1. **Hugging Face NLP Course**
   - https://huggingface.co/learn/nlp-course/chapter1/1
   - Free course on transformers, fine-tuning, and deployment

2. **LLM Bootcamp** (Full Stack Deep Learning)
   - https://fullstackdeeplearning.com/llm-bootcamp/
   - Practical guide to building LLM applications

3. **Prompt Engineering Guide**
   - https://www.promptingguide.ai/
   - Best practices for prompting LLMs for reasoning and task execution

---

## Ethics, Safety, and Law

### Books

- **Weapons of Math Destruction** by Cathy O'Neil (2016)
  - Accessible introduction to algorithmic bias and societal impact

- **Robot Ethics 2.0** edited by Lin, Jenkins, Abney (2017)
  - Scholarly essays on moral, legal, and social implications of robotics

- **Race After Technology** by Ruha Benjamin (2019)
  - Explores how technology can reinforce inequality; highly relevant to embodied AI

### Papers

1. **Winfield, A. F. T., et al. (2021).** "Machine ethics: The design and governance of ethical AI and autonomous systems." *Proceedings of the IEEE*, 109(5), 509-517.
   - Framework for ethical robotics design

2. **Gebru, T., et al. (2018).** "Datasheets for datasets." *arXiv preprint arXiv:1803.09010*.
   - Transparency standards for ML datasets (applicable to synthetic robotics data)

### Guidelines and Standards

1. **IEEE Ethics in Action**
   - https://ethicsinaction.ieee.org/
   - Case studies and tools for ethical decision-making in engineering

2. **Partnership on AI Resources**
   - https://partnershiponai.org/resources/
   - Reports on fairness, transparency, and accountability in AI systems

---

## Humanoid Robotics (Specialized)

### Legged Locomotion

1. **Biped Locomotion Control** by Vukobratović & Borovac (2004)
   - Classic paper on Zero Moment Point (ZMP) control for humanoids

2. **Kajita, S., et al. (2014).** *Introduction to Humanoid Robotics*. Springer.
   - Detailed treatment of bipedal walking, balance, and gait planning

### Manipulation and Grasping

1. **Modern Robotics: Mechanics, Planning, and Control** (Chapter 12)
   - http://modernrobotics.org
   - Grasp analysis and manipulation planning

2. **Kragic, D., & Christensen, H. I. (2002).** "Survey on visual servoing for manipulation." *Computational Vision and Active Perception Laboratory*, Vol. 15.
   - Vision-based manipulation techniques

### Full-Body Control

1. **Sentis, L. (2007).** *Synthesis and Control of Whole-Body Behaviors in Humanoid Systems*. PhD Thesis, Stanford.
   - Whole-body operational space control

2. **Atkeson, C. G., et al. (2015).** "What happened at the DARPA Robotics Challenge?" *Robotics Research*, pp. 667-684. Springer.
   - Lessons learned from real-world humanoid challenges

---

## Robotics Communities and Conferences

### Online Communities

1. **r/robotics (Reddit)**
   - https://www.reddit.com/r/robotics/
   - Active community for discussions, project sharing, and career advice

2. **ROS Discourse**
   - https://discourse.ros.org/
   - Official ROS community forum for technical discussions

3. **Robotics Stack Exchange**
   - https://robotics.stackexchange.com/
   - Q&A for robotics-specific technical questions

### Conferences (Top-Tier)

1. **ICRA (IEEE International Conference on Robotics and Automation)**
   - Premier robotics conference; browse past proceedings on IEEE Xplore

2. **IROS (IEEE/RSJ International Conference on Intelligent Robots and Systems)**
   - Focuses on intelligent systems and applications

3. **CoRL (Conference on Robot Learning)**
   - Cutting-edge research on learning-based robotics

4. **HRI (ACM/IEEE International Conference on Human-Robot Interaction)**
   - Social robotics and human-centered design

---

## YouTube Channels and Podcasts

### YouTube

1. **Boston Dynamics**
   - https://www.youtube.com/@BostonDynamics
   - Impressive humanoid and quadruped robot demonstrations

2. **Lex Fridman Podcast (Robotics Episodes)**
   - https://www.youtube.com/@lexfridman
   - In-depth interviews with robotics researchers (e.g., Marc Raibert, Pieter Abbeel)

3. **Two Minute Papers**
   - https://www.youtube.com/@TwoMinutePapers
   - Summaries of recent AI/robotics research papers

4. **Robotics Today**
   - https://www.youtube.com/@RoboticsToday
   - Interviews with robotics startups and researchers

### Podcasts

1. **The Robot Brains Podcast**
   - https://www.therobotbrains.ai/
   - Interviews with leading robotics and AI researchers

2. **Sense Think Act**
   - https://www.senseactthink.com/
   - Robotics news and technical discussions

---

## Hands-On Projects and Competitions

### DIY Robotics

1. **Hackster.io Robotics Projects**
   - https://www.hackster.io/projects/tags/robotics
   - Community-contributed tutorials for Arduino, Raspberry Pi, and ROS-based robots

2. **Instructables Robotics**
   - https://www.instructables.com/circuits/robots/
   - Step-by-step guides for building robots from scratch

### Competitions

1. **RoboCup**
   - https://www.robocup.org/
   - International robot soccer competition with humanoid leagues

2. **FIRST Robotics**
   - https://www.firstinspires.org/
   - High school robotics competition (great for mentoring/volunteering)

3. **AWS DeepRacer**
   - https://aws.amazon.com/deepracer/
   - Autonomous racing league using reinforcement learning

4. **Kaggle Robotics Competitions**
   - https://www.kaggle.com/competitions
   - Occasionally hosts perception/manipulation challenges with real datasets

---

## Open-Source Robot Platforms

1. **TurtleBot 3**
   - https://emanual.robotis.com/docs/en/platform/turtlebot3/
   - Low-cost educational mobile robot; excellent for learning ROS 2

2. **Poppy Project**
   - https://www.poppy-project.org/
   - 3D-printed humanoid and torso robots for research and education

3. **OpenDog**
   - https://github.com/XRobots/openDog
   - Open-source quadruped robot (inspired by Boston Dynamics Spot)

4. **InMoov**
   - http://inmoov.fr/
   - 3D-printed life-size humanoid robot

---

## Advanced Topics

### Soft Robotics

- **Soft Robotics** journal by Mary Ann Liebert, Inc.
  - https://www.liebertpub.com/loi/soro
  - Cutting-edge research on compliant actuators and biomimetic designs

### Swarm Robotics

- **Swarm Intelligence: From Natural to Artificial Systems** by Bonabeau, Dorigo, Theraulaz (1999)
  - Foundational text on collective robot behavior

### Human-Robot Collaboration

- **HRI Handbook** edited by Bartneck, Belpaeme, Eyssel (2020)
  - Comprehensive reference on designing robots for human interaction

---

## Staying Current

### Preprint Servers

1. **arXiv.org (cs.RO - Robotics)**
   - https://arxiv.org/list/cs.RO/recent
   - Latest research papers before peer review

2. **OpenReview.net**
   - https://openreview.net/
   - Preprints with public reviews (used by CoRL, ICLR, NeurIPS)

### Blogs and News

1. **IEEE Spectrum Robotics**
   - https://spectrum.ieee.org/topic/robotics/
   - Industry news and technical analysis

2. **The Robot Report**
   - https://www.therobotreport.com/
   - Business and technology news in robotics

3. **Towards Data Science (Robotics Tag)**
   - https://towardsdatascience.com/tagged/robotics
   - Technical blog posts by practitioners

---

## Textbook Companion Repository

All code examples, solution templates, and supplementary materials are available at:

**GitHub**: https://github.com/physical-ai-textbook/examples

**Contents**:
- Full source code for all chapter examples
- ROS 2 packages for exercises
- Dockerfiles for reproducible environments
- Sample datasets for perception tasks
- Instructor resources (upon request)

---

## Feedback and Contributions

Found a great resource not listed here? Suggest it via:
- **GitHub Issues**: https://github.com/physical-ai-textbook/book/issues
- **Email**: `suggestions@physical-ai-textbook.dev`

We review and update this list quarterly.

---

**Remember**: The best way to learn robotics is by building. Use these resources to deepen understanding, but prioritize hands-on experimentation over passive consumption.

---

## See Also

- **[References](./references.md)**: Primary sources cited in the textbook
- **[Glossary](./glossary.md)**: Definitions of key terms
