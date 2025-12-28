/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main textbook sidebar - will be populated as modules are added
  textbookSidebar: [
    {
      type: 'category',
      label: 'Home',
      items: [
        'preface/index',
        'preface/how-to-use-this-book',
        'preface/prerequisites',
        'preface/ethics-and-safety',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Foundations (ROS 2)',
      items: [
        'foundations-ros2/index',
        'foundations-ros2/what-is-ros2',
        'foundations-ros2/nodes-topics-services',
        'foundations-ros2/actions-and-parameters',
        'foundations-ros2/urdf-and-robot-models',
        'foundations-ros2/bridging-ai-agents-to-ros',
        'foundations-ros2/examples',
        'foundations-ros2/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Simulation',
      items: [
        'digital-twin/index',
        'digital-twin/why-simulation-matters',
        'digital-twin/digital-twins-concept',
        'digital-twin/gazebo-physics-simulation',
        'digital-twin/unity-high-fidelity-rendering',
        'digital-twin/sensor-simulation',
        'digital-twin/examples',
        'digital-twin/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: GPU-Accelerated Perception',
      items: [
        'ai-robot-brain/index',
        'ai-robot-brain/gpu-accelerated-simulation',
        'ai-robot-brain/isaac-sim-photorealism',
        'ai-robot-brain/synthetic-data-generation',
        'ai-robot-brain/isaac-ros-perception',
        'ai-robot-brain/vslam-nav2-integration',
        'ai-robot-brain/examples',
        'ai-robot-brain/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'vision-language-action/index',
        'vision-language-action/voice-to-action-pipelines',
        'vision-language-action/whisper-speech-recognition',
        'vision-language-action/llms-for-cognitive-planning',
        'vision-language-action/llm-to-ros2-action-generation',
        'vision-language-action/capstone-humanoid-autonomy',
        'vision-language-action/examples',
        'vision-language-action/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/references',
        'appendices/further-reading',
      ],
    },
  ],
};

export default sidebars;
