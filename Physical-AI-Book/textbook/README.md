# Physical-AI & Humanoid Robotics Textbook

A comprehensive university-level textbook covering Physical-AI and Humanoid Robotics, targeting undergraduate to early graduate students.

## Overview

This textbook provides a complete learning path through modern humanoid robotics, covering:

- **Module 1**: Foundations of ROS 2 Middleware
- **Module 2**: Digital Twin Simulation (Gazebo & Unity)
- **Module 3**: AI-Driven Perception (NVIDIA Isaac)
- **Module 4**: Vision-Language-Action Pipelines

## Features

- **WHY-before-HOW Pedagogy**: Conceptual understanding before implementation details
- **Self-Contained Modules**: Each module independently testable and deliverable
- **Structured Learning**: Clear learning outcomes, exercises, and instructor solutions
- **Accessible Design**: Grayscale diagrams, academic tone, mobile-friendly

## Getting Started

### Prerequisites

- Node.js 18+ installed
- Basic programming knowledge (Python or C++)
- Familiarity with command-line interfaces

### Installation

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build
```

### Development Server

The development server will start at `http://localhost:3000` with hot-reloading enabled.

## Project Structure

```
textbook/
├── docs/                    # All textbook content
│   ├── 00-preface/          # Introduction and prerequisites
│   ├── 01-foundations-ros2/ # Module 1: ROS 2
│   ├── 02-digital-twin/     # Module 2: Simulation
│   ├── 03-ai-robot-brain/   # Module 3: Isaac
│   ├── 04-vision-language-action/ # Module 4: VLA
│   ├── 99-appendices/       # Glossary, references
│   └── diagrams/            # Shared SVG diagrams
├── src/                     # Custom React components
├── static/                  # Static assets (images, fonts)
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Navigation configuration
└── package.json             # Dependencies
```

## Validation

### Validate Frontmatter

```bash
npm run validate:frontmatter
```

### Check Links

```bash
npm run validate:links
```

### Lint Markdown

```bash
npm run validate:markdown
```

### Run All Validations

```bash
npm run validate
```

## Content Guidelines

### File Naming

- Use kebab-case: `what-is-ros2.md`, `digital-twin-architecture.svg`
- Prefix chapters with sequence: `01-what-is-ros2.md`

### YAML Frontmatter

All markdown files must include:

```yaml
---
title: "Chapter Title"
slug: chapter-slug
sidebar_label: "1.1 Short Label"
toc: true
description: "1-2 sentence summary"
---
```

### Diagrams

- Format: SVG only
- Style: Grayscale (black, white, shades of gray)
- Accessibility: Include alt text in markdown

### Exercises

- 3 exercises per chapter
- Structure: Recall → Application → Synthesis
- Instructor solutions in `.answers/` subdirectories

## Contributing

1. Follow the constitution principles in `.specify/memory/constitution.md`
2. Validate all content before committing
3. Maintain academic tone and WHY-before-HOW approach
4. Ensure diagrams are accessible (grayscale + alt text)

## License

This textbook is licensed under [CC BY-NC-SA 4.0 International License](LICENSE).

**Educational and non-commercial use permitted. Commercial use requires separate permission.**

## Authors

[Add author names and affiliations]

## Citation

[Add citation format for academic use]

## Support

For issues or questions:
- GitHub Issues: [repository-url/issues]
- Email: [contact-email]

---

Built with [Docusaurus 3](https://docusaurus.io/)
