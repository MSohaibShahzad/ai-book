import React from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

export default function Home() {

  const modules = [
    {
      number: '01',
      title: 'Foundations (ROS 2)',
      topics: [
        'ROS 2 middleware architecture',
        'Nodes, topics, and services',
        'URDF robot modeling',
        'AI-ROS 2 integration'
      ],
      link: '/docs/foundations-ros2'
    },
    {
      number: '02',
      title: 'Digital Twin & Simulation',
      topics: [
        'Physics-based simulation (Gazebo)',
        'High-fidelity rendering (Unity)',
        'Sim-to-real transfer',
        'Synthetic data generation'
      ],
      link: '/docs/digital-twin'
    },
    {
      number: '03',
      title: 'GPU-Accelerated Perception',
      topics: [
        'NVIDIA Isaac Sim',
        'Isaac ROS perception nodes',
        'Real-time vision processing',
        'Visual SLAM & object detection'
      ],
      link: '/docs/ai-robot-brain'
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      topics: [
        'Whisper speech recognition',
        'LLM cognitive planning',
        'Multimodal integration',
        'Voice-controlled autonomy'
      ],
      link: '/docs/vision-language-action'
    }
  ];

  return (
    <Layout
      title={`Home`}
      description="Master Physical AI & Humanoid Robotics - A comprehensive university textbook">

      {/* Hero Section */}
      <section className="hero-section">
        <div className="hero-content">
          <h1 className="hero-title">
            Master Physical AI &<br/>Humanoid Robotics
          </h1>
          <p className="hero-subtitle">
            A comprehensive university textbook bridging classical robotics, modern AI,
            and real-world humanoid systems. From ROS 2 foundations to vision-language-action models.
          </p>

          <div className="hero-buttons">
            <Link
              className="cyber-button cyber-button-primary"
              to="/docs/preface">
              <span>📖 Start Reading</span>
            </Link>
            <a
              className="cyber-button cyber-button-secondary"
              href="https://github.com/MSohaibShahzad"
              target="_blank"
              rel="noopener noreferrer">
              <span>⭐ GitHub Profile</span>
            </a>
          </div>

          <div className="scroll-indicator">
            ↓ Explore Modules Below ↓
          </div>
        </div>
      </section>

      {/* Modules Section */}
      <section className="modules-section">
        <div className="modules-container">
          <h2 className="section-title">Learning Modules</h2>

          <div className="modules-grid">
            {modules.map((module, idx) => (
              <div key={idx} className="module-card">
                <div className="module-number">{module.number}</div>
                <h3 className="module-title">{module.title}</h3>
                <ul className="module-topics">
                  {module.topics.map((topic, topicIdx) => (
                    <li key={topicIdx}>{topic}</li>
                  ))}
                </ul>
                <Link className="module-link" to={module.link}>
                  Explore Module →
                </Link>
              </div>
            ))}
          </div>
        </div>
      </section>
    </Layout>
  );
}
