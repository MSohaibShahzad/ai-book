---
id: 0002
title: Physical-AI Textbook Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: sohaib
command: /sp.plan
labels: ["textbook", "robotics", "physical-ai", "architecture", "docusaurus", "planning"]
links:
  spec: specs/001-physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-robotics-textbook/plan.md
  - specs/001-physical-ai-robotics-textbook/research.md
  - specs/001-physical-ai-robotics-textbook/data-model.md
  - specs/001-physical-ai-robotics-textbook/quickstart.md
  - specs/001-physical-ai-robotics-textbook/contracts/module-structure.schema.json
  - CLAUDE.md
tests:
  - Constitution Check validation - PASS (all gates)
---

## Prompt

Title: Physical-AI & Humanoid Robotics — Plan

Documentation Framework:
  - Use Docusaurus for the final textbook.
  - All content written in Markdown with YAML frontmatter.
  - Navigation handled through an auto-generated sidebar (sidebars.js).

Book Structure:
  - Textbook root folder: physical-ai-humanoid-robotics/textbook/
  - Four modules as defined in /sp.specify:
      Module 1: foundations-ros2
      Module 2: digital-twin
      Module 3: ai-robot-brain
      Module 4: vision-language-action
  - Each module contains:
      index.md
      chapter files (one per chapter)
      examples.md
      exercises.md
      .answers/exercises-answers.md
  - A shared diagrams/ folder for SVG placeholders.

Content Style:
  - Academic tone suitable for undergraduate to early graduate students.
  - Clear structure using headings, numbered lists, diagrams, and conceptual explanations.
  - No hardware instructions; conceptual demonstrations only.

Technologies Referenced:
  - ROS 2 with rclpy for conceptual middleware examples.
  - Gazebo for physics simulation concepts.
  - Unity for high-fidelity visualization concepts.
  - NVIDIA Isaac Sim & Isaac ROS for perception and synthetic data concepts.
  - Whisper and LLM-based reasoning for Vision-Language-Action examples.

Naming Conventions:
  - All filenames in kebab-case.
  - All markdown pages include YAML frontmatter (title, slug, sidebar_label, toc: true, description).

Diagram Handling:
  - Diagrams stored as SVG placeholders in diagrams/ folder.
  - Files only referenced, not rendered.

## Response snapshot

Created comprehensive implementation plan for Physical-AI & Humanoid Robotics textbook using Docusaurus framework:

**Technical Architecture**:
- **Framework**: Docusaurus 3.x with React 18+ for static site generation
- **Content Format**: Markdown with YAML frontmatter validation via JSON Schema
- **Structure**: 4 modules (00-preface, 01-foundations-ros2, 02-digital-twin, 03-ai-robot-brain, 04-vision-language-action, 99-appendices)
- **Diagrams**: SVG format with grayscale design for accessibility
- **Code Examples**: Python 3.9+ pseudocode with optional cloud lab links

**Phase 0 (Research) - Completed**:
6 architectural decisions documented in research.md:
1. Docusaurus 3.x for multi-module textbook structure
2. Annotated Python pseudocode + cloud labs (balances WHY-before-HOW with hands-on)
3. SVG diagrams (grayscale, high-contrast, alt text for accessibility)
4. 3-tier Bloom's Taxonomy exercises (Recall, Application, Synthesis)
5. Sequential module flow with prerequisite flexibility
6. Principles-based, version-agnostic content for future-proofing

**Phase 1 (Design) - Completed**:
1. **data-model.md**: Defined 7 core entities:
   - Module (4 total, each with 3-5 chapters, learning outcomes, exercises)
   - Chapter (12-16 total across all modules)
   - LearningOutcome (3-5 per module, Bloom's Taxonomy aligned)
   - Diagram (SVG system architecture, process flows, conceptual diagrams)
   - Exercise (3 per chapter: Tier 1 Recall, Tier 2 Application, Tier 3 Synthesis)
   - ExerciseAnswers (instructor solutions in .answers/ directories)
   - CodeExample (runnable/conceptual Python examples)

2. **contracts/module-structure.schema.json**: JSON Schema for validating YAML frontmatter across module index, chapter, and preface files

3. **quickstart.md**: Step-by-step setup guide covering:
   - Docusaurus installation and configuration
   - Content creation workflow (modules, chapters, exercises, diagrams)
   - Validation tools (frontmatter, markdown linting, link checking)
   - Development workflow (local server, build, PDF export)
   - Contributing guidelines (file naming, YAML requirements, exercise structure)

**Constitution Check**: ✅ ALL GATES PASSED
- Structured Learning Path (Principle I)
- Clear Academic Communication (Principle II)
- Strict File Conventions (Principle III)
- Hands-On Learning (Principle IV)
- Assessment & Validation (Principle V)
- Attribution & Licensing (Principle VI)

**Project Structure**:
```
textbook/
├── docs/
│   ├── 00-preface/ (4 files: index, how-to-use, prerequisites, ethics)
│   ├── 01-foundations-ros2/ (9 files: index, 5 chapters, examples, exercises, .answers/)
│   ├── 02-digital-twin/ (9 files)
│   ├── 03-ai-robot-brain/ (9 files)
│   ├── 04-vision-language-action/ (9 files)
│   ├── 99-appendices/ (3 files: glossary, references, further-reading)
│   └── diagrams/ (5 SVG files: ROS 2, digital twin, Isaac, VLA, capstone)
├── src/ (Docusaurus customizations)
├── static/ (images, fonts)
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

## Outcome

- ✅ Impact: Complete architectural plan with research decisions, data model, contracts, and quickstart guide ready for implementation
- 🧪 Tests: Constitution Check validation - PASS (9/9 principles satisfied)
- 📁 Files: plan.md (205 lines), research.md (215 lines), data-model.md (450 lines), quickstart.md (390 lines), module-structure.schema.json (JSON Schema)
- 🔁 Next prompts: `/sp.tasks` to generate testable implementation tasks, then `/sp.implement` for execution
- 🧠 Reflection: Plan successfully balances Docusaurus technical requirements with pedagogical constitution principles. Research phase resolved 6 key architectural decisions (framework, code strategy, diagrams, exercises, sequencing, future-proofing). Data model defines clear entity relationships enabling systematic content generation. Ready for task breakdown and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan completed without clarifications or gate violations
- Graders run and results (PASS/FAIL): Constitution Check - PASS (9/9 gates); YAML Schema validation - defined
- Prompt variant (if applicable): Standard /sp.plan workflow with Docusaurus-specific adaptations
- Next experiment (smallest change to try): Proceed to /sp.tasks for generating chapter-by-chapter implementation tasks with acceptance criteria
