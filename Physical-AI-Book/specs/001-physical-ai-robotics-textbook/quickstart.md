# Quickstart: Physical-AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-robotics-textbook | **Date**: 2025-12-09

## Overview

This guide provides instructions for setting up the Docusaurus project, contributing content, validating markdown files, and building the textbook for web or PDF export.

---

## Prerequisites

- **Node.js**: Version 18+ (LTS recommended)
- **npm or yarn**: Package manager for Node.js dependencies
- **Git**: Version control (repository already initialized)
- **Text Editor**: VS Code, Vim, or any markdown-compatible editor
- **Optional**: Python 3.9+ (for validating code examples)

---

## Initial Setup

### 1. Install Docusaurus

From the repository root (`textbook/`), initialize a new Docusaurus site:

```bash
cd textbook/
npx create-docusaurus@latest . classic --typescript
```

This creates the base Docusaurus structure with:
- `docs/` — Markdown content directory
- `src/` — Custom React components and styles
- `static/` — Static assets (images, fonts)
- `docusaurus.config.js` — Site configuration
- `sidebars.js` — Navigation configuration

### 2. Install Dependencies

```bash
npm install
```

This installs:
- Docusaurus 3.x
- React 18+
- remark/rehype plugins (Markdown processing)
- Syntax highlighting (Prism.js)

### 3. Configure Site Metadata

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  title: 'Physical-AI & Humanoid Robotics',
  tagline: 'A University Textbook on Modern Humanoid Robotics',
  url: 'https://physical-ai-textbook.dev',
  baseUrl: '/',
  organizationName: 'your-org',
  projectName: 'physical-ai-textbook',

  themeConfig: {
    navbar: {
      title: 'Physical-AI Textbook',
      items: [
        {
          type: 'doc',
          docId: '00-preface/index',
          position: 'left',
          label: 'Preface',
        },
        {
          type: 'doc',
          docId: '01-foundations-ros2/index',
          position: 'left',
          label: 'Module 1: ROS 2',
        },
        // Add other modules...
      ],
    },
    footer: {
      copyright: `© ${new Date().getFullYear()} Licensed under CC BY-NC-SA 4.0`,
    },
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/physical-ai-textbook/edit/main/',
        },
      },
    ],
  ],
};
```

### 4. Configure Sidebar

Edit `sidebars.js` to define module navigation:

```javascript
module.exports = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Preface',
      items: [
        '00-preface/index',
        '00-preface/how-to-use-this-book',
        '00-preface/prerequisites',
        '00-preface/ethics-and-safety',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Foundations (ROS 2)',
      items: [
        '01-foundations-ros2/index',
        '01-foundations-ros2/01-what-is-ros2',
        '01-foundations-ros2/02-nodes-topics-services',
        '01-foundations-ros2/03-actions-and-parameters',
        '01-foundations-ros2/04-urdf-and-robot-models',
        '01-foundations-ros2/05-bridging-ai-agents-to-ros',
        '01-foundations-ros2/examples',
        '01-foundations-ros2/exercises',
      ],
    },
    // Add other modules...
    {
      type: 'category',
      label: 'Appendices',
      items: [
        '99-appendices/glossary',
        '99-appendices/references',
        '99-appendices/further-reading',
      ],
    },
  ],
};
```

**Note**: `.answers/` directories are excluded from the sidebar to keep instructor solutions hidden from students.

---

## Content Creation Workflow

### 1. Create Module Directory Structure

For each module (e.g., Module 1: ROS 2):

```bash
mkdir -p docs/01-foundations-ros2/.answers
```

### 2. Create Module Index

Create `docs/01-foundations-ros2/index.md`:

```markdown
---
title: "Foundations: ROS 2 Middleware"
slug: foundations-ros2
sidebar_label: "Module 1: ROS 2"
toc: true
description: "Understanding the middleware backbone of humanoid robots through ROS 2 concepts: nodes, topics, services, and actions."
---

# Module 1: Foundations of ROS 2

## Overview

This module introduces ROS 2 as the middleware "nervous system" of humanoid robots...

## Learning Outcomes

By the end of this module, you will be able to:

1. Explain the purpose of ROS 2 as middleware for robot control systems
2. Diagram node communication flows for sensor data processing
3. Design a URDF model representing a humanoid robot's structure
4. Identify when to use topics vs. services vs. actions in ROS 2 architectures
5. Understand how AI agents bridge to ROS 2 controllers

## Prerequisites

- Basic programming knowledge (Python or C++)
- Familiarity with command-line interfaces

## Chapters

1. [What is ROS 2?](./01-what-is-ros2.md)
2. [Nodes, Topics, and Services](./02-nodes-topics-services.md)
3. [Actions and Parameters](./03-actions-and-parameters.md)
4. [URDF and Robot Models](./04-urdf-and-robot-models.md)
5. [Bridging AI Agents to ROS](./05-bridging-ai-agents-to-ros.md)

## References

- ROS 2 Documentation: https://docs.ros.org/
- *Robot Operating System: The Complete Reference (Volume 6)*
```

### 3. Create Chapter Files

Create `docs/01-foundations-ros2/01-what-is-ros2.md`:

```markdown
---
title: "What is ROS 2? The Robot Operating System Explained"
slug: what-is-ros2
sidebar_label: "1.1 What is ROS 2?"
toc: true
description: "Explore the purpose of ROS 2 as the middleware nervous system for humanoid robots, enabling communication between sensors, controllers, and actuators."
---

# 1.1 What is ROS 2?

## Introduction

The Robot Operating System (ROS 2) is not an operating system in the traditional sense...

## Why Middleware Matters

Imagine a humanoid robot with dozens of sensors...

## The Publish-Subscribe Pattern

![ROS 2 Node Communication](../diagrams/ros2-node-communication.svg)

*Figure 1.1: ROS 2 nodes communicating via topics*

## Summary

In this chapter, we explored...
```

### 4. Create Exercises

Create `docs/01-foundations-ros2/exercises.md`:

```markdown
---
title: "Module 1 Exercises"
slug: exercises
sidebar_label: "Exercises"
toc: true
description: "Practice problems for ROS 2 foundations module."
---

# Module 1: Exercises

## Chapter 1.1: What is ROS 2?

### Exercise 1.1.1 (Recall)

Explain in your own words why ROS 2 is described as the "nervous system" of a robot.

### Exercise 1.1.2 (Application)

Design a simple node communication flow for a humanoid robot that must read IMU sensor data and adjust balance. Identify which nodes would publish and subscribe to which topics.

**Hints**:
- Consider one node for the IMU sensor and another for the balance controller
- Think about the data direction: sensor → controller

### Exercise 1.1.3 (Synthesis)

Compare the publish-subscribe pattern (topics) to the request-response pattern (services) in ROS 2. When would you use each for a humanoid robot's perception system?
```

### 5. Create Instructor Solutions (Hidden)

Create `docs/01-foundations-ros2/.answers/exercises-answers.md`:

```markdown
# Module 1: Exercise Solutions (Instructor Only)

## Exercise 1.1.1 (ex-01-01-01) {#ex-01-01-01}

**Question**: Explain in your own words why ROS 2 is described as the "nervous system" of a robot.

**Solution**:
ROS 2 is called the "nervous system" because it functions like the communication network in a biological organism...

**Rubric**:
- Mentions communication/coordination role (3 points)
- Connects to biological nervous system analogy (2 points)
- References sensors → controllers → actuators flow (2 points)
- Uses specific ROS 2 terms (nodes, topics) (3 points)

## Exercise 1.1.2 (ex-01-01-02) {#ex-01-01-02}

**Question**: Design a simple node communication flow...

**Solution**:
[Diagram showing IMU Publisher Node → `/imu/data` topic → Balance Controller Subscriber Node]

Node 1: `imu_publisher`
- Publishes to topic: `/imu/data`
- Message type: `sensor_msgs/Imu`

Node 2: `balance_controller`
- Subscribes to topic: `/imu/data`
- Processes orientation and adjusts motor commands

**Explanation**:
This demonstrates understanding of the publish-subscribe pattern (LO: ros2-lo-02)...
```

### 6. Create Diagrams

Use draw.io or Inkscape to create SVG diagrams in `docs/diagrams/`:

```bash
mkdir -p docs/diagrams
```

Place diagrams like `ros2-node-communication.svg` in this directory with proper alt text in markdown:

```markdown
![ROS 2 Node Communication](../diagrams/ros2-node-communication.svg)

*Figure 1.1: Diagram showing three ROS 2 nodes (Sensor Publisher, Controller Subscriber, Actuator Action Server) connected via topics and action interfaces.*
```

---

## Validation

### 1. Validate YAML Frontmatter

Use a JSON Schema validator to check frontmatter:

```bash
npm install -g ajv-cli
ajv validate -s specs/001-physical-ai-robotics-textbook/contracts/module-structure.schema.json -d "docs/01-foundations-ros2/index.md"
```

### 2. Lint Markdown

```bash
npm install -g markdownlint-cli
markdownlint docs/**/*.md
```

### 3. Check Links

```bash
npm install -g markdown-link-check
find docs -name "*.md" -exec markdown-link-check {} \;
```

---

## Development Workflow

### 1. Start Development Server

```bash
npm start
```

This launches a local server at `http://localhost:3000` with hot-reloading for live editing.

### 2. Build Static Site

```bash
npm run build
```

This generates static HTML/CSS/JS in the `build/` directory for deployment.

### 3. Serve Built Site Locally

```bash
npm run serve
```

Test the production build at `http://localhost:3000`.

---

## Export to PDF/ePub

### Option 1: Docusaurus PDF Plugin

Install the PDF plugin:

```bash
npm install --save @docusaurus/plugin-ideal-image
npm install --save docusaurus-prince-pdf
```

Configure in `docusaurus.config.js`:

```javascript
plugins: [
  [
    'docusaurus-prince-pdf',
    {
      princeArgs: ['--javascript'],
      output: 'physical-ai-textbook.pdf',
    },
  ],
],
```

Generate PDF:

```bash
npm run build
npm run serve
# Visit http://localhost:3000/pdf to download
```

### Option 2: Pandoc Conversion

Convert markdown to PDF via Pandoc:

```bash
# Install Pandoc
sudo apt install pandoc texlive

# Concatenate all chapters
cat docs/01-foundations-ros2/*.md > module1.md

# Convert to PDF
pandoc module1.md -o module1.pdf --toc --number-sections
```

---

## Contributing Guidelines

### File Naming Conventions

- **All files**: Use kebab-case (e.g., `what-is-ros2.md`, `ros2-node-communication.svg`)
- **Chapters**: Prefix with sequence (e.g., `01-what-is-ros2.md`)
- **Modules**: Prefix directories (e.g., `01-foundations-ros2/`)

### YAML Frontmatter Requirements

Every markdown file MUST include:

```yaml
---
title: string              # Full descriptive title
slug: string               # URL-friendly identifier (kebab-case)
sidebar_label: string      # Concise navigation label
toc: true                  # Always true
description: string        # 1-2 sentence summary
---
```

Validate against `contracts/module-structure.schema.json`.

### Code Examples

- Use Python 3.9+ syntax for ROS 2 examples
- Include necessary imports and context
- Annotate with conceptual comments ("why" not just "what")
- Provide optional cloud environment links (Colab, Binder) for runnable examples

### Diagrams

- **Format**: SVG only
- **Style**: Grayscale (black, white, shades of gray)
- **Accessibility**: Include alt text in markdown
- **Source files**: Keep `.drawio` or `.svg` source in `docs/diagrams/`

### Exercises

- **Structure**: 3 exercises per chapter (Recall, Application, Synthesis)
- **Numbering**: `ex-{module}-{chapter}-{exercise}` (e.g., `ex-01-01-01`)
- **Solutions**: Place in `.answers/exercises-answers.md` (hidden from sidebar)

---

## Troubleshooting

### Issue: Module not appearing in sidebar

**Solution**: Verify `sidebars.js` includes the module category and chapter paths match file locations exactly.

### Issue: Diagrams not rendering

**Solution**: Check that:
1. SVG files are in `docs/diagrams/`
2. Markdown uses relative paths (e.g., `../diagrams/file.svg`)
3. File permissions allow reading

### Issue: Frontmatter validation errors

**Solution**: Run JSON Schema validator:

```bash
ajv validate -s contracts/module-structure.schema.json -d "docs/path/to/file.md"
```

Check for:
- Missing required fields (`title`, `slug`, `sidebar_label`, `toc`, `description`)
- Invalid field types (e.g., `toc` must be boolean `true`)
- Exceeding max lengths

---

## Next Steps

1. **Phase 2**: Generate `tasks.md` using `/sp.tasks` command
2. **Implementation**: Create all module directories and initial markdown files
3. **Content Writing**: Fill chapters with conceptual explanations and diagrams
4. **Review**: Validate all frontmatter, links, and exercises
5. **Build**: Generate static site and test navigation

---

## References

- **Docusaurus Documentation**: https://docusaurus.io/docs
- **Markdown Guide**: https://www.markdownguide.org/
- **JSON Schema**: https://json-schema.org/
- **Draw.io**: https://www.diagrams.net/
- **Pandoc**: https://pandoc.org/

---

**Last Updated**: 2025-12-09
