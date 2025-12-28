---
title: "References"
slug: references
sidebar_label: "References"
toc: true
description: "Cited works, research papers, and primary sources used throughout the Physical-AI textbook"
---

# References

This page lists all primary sources, research papers, official documentation, and foundational works cited throughout the textbook, organized by module and topic.

---

## Foundational Works

### Robotics Textbooks

1. **Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009).** *Robotics: Modelling, Planning and Control*. Springer.
   - Comprehensive reference for kinematics, dynamics, and control theory

2. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.
   - Foundational text for SLAM, localization, and Bayesian filtering

3. **LaValle, S. M. (2006).** *Planning Algorithms*. Cambridge University Press.
   - Motion planning, configuration spaces, and search algorithms

### AI and Machine Learning

4. **Goodfellow, I., Bengio, Y., & Courville, A. (2016).** *Deep Learning*. MIT Press.
   - Standard reference for neural networks and deep learning fundamentals

5. **Russell, S., & Norvig, P. (2020).** *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.
   - Comprehensive AI textbook covering search, planning, and learning

---

## Module 1: Foundations (ROS 2)

### ROS 2 Official Documentation

6. **Open Robotics. (2024).** *ROS 2 Documentation - Humble Release*.
   - https://docs.ros.org/en/humble/
   - Primary reference for ROS 2 concepts, APIs, and tutorials

7. **Quigley, M., Gerkey, B., & Smart, W. D. (2015).** *Programming Robots with ROS*. O'Reilly Media.
   - Foundational ROS 1 book; concepts transfer to ROS 2

### DDS and Communication

8. **Object Management Group. (2015).** *Data Distribution Service (DDS) Specification, Version 1.4*.
   - https://www.omg.org/spec/DDS/
   - Underlying middleware standard for ROS 2 communication

### URDF and Robot Modeling

9. **Open Robotics. (2024).** *URDF Specification*.
   - http://wiki.ros.org/urdf
   - XML format for robot kinematic and visual descriptions

10. **Foote, T. (2013).** "tf: The transform library." *IEEE International Conference on Technologies for Practical Robot Applications (TePRA)*.
    - Coordinate frame management in ROS

---

## Module 2: Digital Twin Simulation

### Gazebo Simulation

11. **Open Robotics. (2024).** *Gazebo Sim Documentation - Fortress Release*.
    - https://gazebosim.org/docs/fortress
    - Official documentation for Gazebo physics and sensor simulation

12. **Koenig, N., & Howard, A. (2004).** "Design and use paradigms for Gazebo, an open-source multi-robot simulator." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Vol. 3, pp. 2149-2154.
    - Original Gazebo architecture paper

### Unity for Robotics

13. **Unity Technologies. (2024).** *Unity Robotics Hub*.
    - https://github.com/Unity-Technologies/Unity-Robotics-Hub
    - ROS integration, URDF importers, and sensor simulation in Unity

14. **Juliani, A., et al. (2018).** "Unity: A general platform for intelligent agents." *arXiv preprint arXiv:1809.02627*.
    - Unity ML-Agents framework (related to robotics simulation)

### Sensor Simulation

15. **Gschwandtner, M., et al. (2011).** "BlenSor: Blender sensor simulation toolbox." *Advances in Visual Computing*, pp. 199-208. Springer.
    - Sensor noise modeling in simulation environments

---

## Module 3: AI-Driven Perception (NVIDIA Isaac)

### Isaac Sim

16. **NVIDIA. (2024).** *Isaac Sim Documentation*.
    - https://docs.omniverse.nvidia.com/isaacsim/latest/
    - Official documentation for GPU-accelerated robotics simulation

17. **NVIDIA. (2023).** "Bringing Robotics to Reality with Isaac Sim." *NVIDIA Technical Blog*.
    - https://developer.nvidia.com/blog/
    - Photorealistic rendering, synthetic data generation, domain randomization

### Synthetic Data and Domain Randomization

18. **Tobin, J., et al. (2017).** "Domain randomization for transferring deep neural networks from simulation to the real world." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 23-30.
    - Foundational work on sim-to-real transfer

19. **Tremblay, J., et al. (2018).** "Training deep networks with synthetic data: Bridging the reality gap by domain randomization." *CVPR Workshops*, Vol. 10, p. 1.
    - NVIDIA's approach to synthetic data generation

### Isaac ROS (Perception)

20. **NVIDIA. (2024).** *Isaac ROS Documentation*.
    - https://nvidia-isaac-ros.github.io/
    - GPU-accelerated perception pipelines for ROS 2

21. **Redmon, J., & Farhadi, A. (2018).** "YOLOv3: An incremental improvement." *arXiv preprint arXiv:1804.02767*.
    - Real-time object detection used in perception pipelines

### Visual SLAM

22. **Mur-Artal, R., Montiel, J. M. M., & Tardos, J. D. (2015).** "ORB-SLAM: A versatile and accurate monocular SLAM system." *IEEE Transactions on Robotics*, 31(5), 1147-1163.
    - Visual SLAM algorithm used in Nav2 integration

23. **Labbé, M., & Michaud, F. (2019).** "RTAB-Map as an open-source LiDAR and visual simultaneous localization and mapping library for large-scale and long-term online operation." *Journal of Field Robotics*, 36(2), 416-446.
    - RTAB-Map algorithm for 3D SLAM

---

## Module 4: Vision-Language-Action Pipelines

### Speech Recognition (Whisper)

24. **Radford, A., et al. (2022).** "Robust speech recognition via large-scale weak supervision." *arXiv preprint arXiv:2212.04356*.
    - OpenAI Whisper model for speech-to-text

### Large Language Models

25. **Brown, T., et al. (2020).** "Language models are few-shot learners." *Advances in Neural Information Processing Systems (NeurIPS)*, 33, 1877-1901.
    - GPT-3 and large-scale language models

26. **Achiam, J., et al. (2023).** "GPT-4 Technical Report." *arXiv preprint arXiv:2303.08774*.
    - GPT-4 capabilities and architecture

27. **Touvron, H., et al. (2023).** "LLaMA: Open and efficient foundation language models." *arXiv preprint arXiv:2302.13971*.
    - Open-source LLM alternative

### Vision-Language Models

28. **Radford, A., et al. (2021).** "Learning transferable visual models from natural language supervision." *International Conference on Machine Learning (ICML)*, pp. 8748-8763.
    - CLIP model for vision-language alignment

29. **Alayrac, J.-B., et al. (2022).** "Flamingo: A visual language model for few-shot learning." *Advances in Neural Information Processing Systems (NeurIPS)*, 35, 23716-23736.
    - Vision-language models for robotics applications

### VLA (Vision-Language-Action Models)

30. **Brohan, A., et al. (2022).** "RT-1: Robotics transformer for real-world control at scale." *arXiv preprint arXiv:2212.06817*.
    - Google's Robotics Transformer for action generation

31. **Brohan, A., et al. (2023).** "RT-2: Vision-language-action models transfer web knowledge to robotic control." *arXiv preprint arXiv:2307.15818*.
    - Vision-language-action integration for humanoids

32. **Driess, D., et al. (2023).** "PaLM-E: An embodied multimodal language model." *arXiv preprint arXiv:2303.03378*.
    - Google's embodied language model combining vision and robotics

### LLM Agents and Cognitive Planning

33. **Huang, W., et al. (2022).** "Language models as zero-shot planners: Extracting actionable knowledge for embodied agents." *International Conference on Machine Learning (ICML)*, pp. 9118-9147.
    - Using LLMs for task planning in robotics

34. **Ahn, M., et al. (2022).** "Do as I can, not as I say: Grounding language in robotic affordances." *arXiv preprint arXiv:2204.01691*.
    - SayCan: Grounding language in robot actions

---

## Ethics and Safety

35. **IEEE. (2019).** *Ethically Aligned Design: A Vision for Prioritizing Human Well-being with Autonomous and Intelligent Systems*, First Edition.
    - https://standards.ieee.org/content/ieee-standards/en/industry-connections/ec/autonomous-systems.html
    - Framework for ethical AI and robotics

36. **European Commission. (2019).** *Ethics Guidelines for Trustworthy AI*.
    - https://ec.europa.eu/digital-single-market/en/news/ethics-guidelines-trustworthy-ai
    - EU regulatory perspective on AI safety and ethics

37. **Winfield, A. F. T., & Jirotka, M. (2017).** "The case for an ethical black box." *Proceedings of the Conference Towards Autonomous Robotic Systems*, pp. 262-273. Springer.
    - Logging and accountability in autonomous systems

38. **Calo, R., Froomkin, A. M., & Kerr, I. (2016).** *Robot Law*. Edward Elgar Publishing.
    - Legal implications of robotics and autonomous systems

---

## Software and Tools

### General Tools

39. **Git. (2024).** *Git Documentation*.
    - https://git-scm.com/doc
    - Version control system used throughout the textbook

40. **Docker. (2024).** *Docker Documentation*.
    - https://docs.docker.com/
    - Containerization for reproducible environments

### Python Libraries

41. **Harris, C. R., et al. (2020).** "Array programming with NumPy." *Nature*, 585(7825), 357-362.
    - NumPy library for numerical computation

42. **Paszke, A., et al. (2019).** "PyTorch: An imperative style, high-performance deep learning library." *Advances in Neural Information Processing Systems (NeurIPS)*, 32.
    - PyTorch framework for deep learning

43. **Wolf, T., et al. (2020).** "Transformers: State-of-the-art natural language processing." *Proceedings of the 2020 Conference on Empirical Methods in Natural Language Processing: System Demonstrations*, pp. 38-45.
    - Hugging Face Transformers library for LLMs

### Visualization and Rendering

44. **Hunter, J. D. (2007).** "Matplotlib: A 2D graphics environment." *Computing in Science & Engineering*, 9(3), 90-95.
    - Python plotting library

45. **Unity Technologies. (2024).** *Unity User Manual 2022.3 LTS*.
    - https://docs.unity3d.com/2022.3/Documentation/Manual/
    - Unity game engine for high-fidelity rendering

---

## Robotics Platforms and Case Studies

### Humanoid Robots

46. **Boston Dynamics. (2024).** *Atlas Humanoid Robot*.
    - https://www.bostondynamics.com/atlas
    - Industry-leading bipedal humanoid platform

47. **PAL Robotics. (2024).** *TALOS Humanoid Robot*.
    - http://pal-robotics.com/robots/talos/
    - Open-source humanoid research platform

48. **Agility Robotics. (2024).** *Digit Bipedal Robot*.
    - https://www.agilityrobotics.com/digit
    - Commercial humanoid for warehouse applications

### Mobile Manipulation

49. **ROBOTIS. (2024).** *TurtleBot 3 Documentation*.
    - https://emanual.robotis.com/docs/en/platform/turtlebot3/
    - Low-cost educational mobile robot platform

50. **Clearpath Robotics. (2024).** *Jackal UGV*.
    - https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/
    - Rugged outdoor mobile robot platform

---

## Standards and Specifications

51. **ISO 10218-1:2011.** *Robots and robotic devices — Safety requirements for industrial robots — Part 1: Robots*.
    - International safety standard for industrial robots

52. **ISO 13482:2014.** *Robots and robotic devices — Safety requirements for personal care robots*.
    - Safety standard for service and assistive robots

53. **ISO/IEC 27001:2013.** *Information technology — Security techniques — Information security management systems*.
    - Data security standards relevant to robot data collection

---

## Online Resources and Communities

54. **ROS Discourse.** *Official ROS 2 Community Forum*.
    - https://discourse.ros.org/
    - Community support and discussions

55. **ROS Answers.** *Q&A for ROS Users*.
    - https://answers.ros.org/
    - Technical troubleshooting and solutions

56. **NVIDIA Developer Forums.** *Isaac Sim and Isaac ROS*.
    - https://forums.developer.nvidia.com/
    - Community support for NVIDIA robotics tools

57. **Stack Overflow.** *Programming Q&A*.
    - https://stackoverflow.com/questions/tagged/ros2
    - General programming and ROS 2 troubleshooting

---

## Datasets

58. **COCO Dataset. (2014).** *Common Objects in Context*.
    - https://cocodataset.org/
    - Large-scale object detection and segmentation dataset

59. **ImageNet. (2009).** *ImageNet Large Scale Visual Recognition Challenge (ILSVRC)*.
    - https://www.image-net.org/
    - Image classification benchmark dataset

60. **Waymo Open Dataset. (2019).** *Waymo Open Dataset: An autonomous driving dataset*.
    - https://waymo.com/open/
    - Sensor data for perception and SLAM research

---

## Updates and Errata

This reference list is current as of December 2025. For updates, corrections, or additional references:

- **GitHub**: https://github.com/physical-ai-textbook/book
- **Errata**: Email `errata@physical-ai-textbook.dev`

---

**Citation Format**: This textbook uses a hybrid of IEEE and APA citation styles. When citing this textbook, use:

```
[Author(s)]. (2025). Physical-AI & Humanoid Robotics: A University Textbook.
Retrieved from https://physical-ai-textbook.dev
```

---

## See Also

- **[Glossary](./glossary.md)**: Definitions of key terms
- **[Further Reading](./further-reading.md)**: Recommended books, courses, and tutorials beyond primary references
