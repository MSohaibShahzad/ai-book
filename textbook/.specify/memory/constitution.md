# Physical-AI & Humanoid Robotics Constitution

<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial constitution creation)
Modified principles: N/A (initial creation)
Added sections:
  - Core Pedagogical Principles (6 principles)
  - Content Standards
  - Safety & Ethics Requirements
  - Governance
Removed sections: N/A
Templates requiring updates:
  ✅ plan-template.md: Compatible (Constitution Check section ready)
  ✅ spec-template.md: Compatible (Requirements section ready)
  ✅ tasks-template.md: Compatible (Task organization ready)
Follow-up TODOs: None
-->

## Core Pedagogical Principles

### I. Structured Learning Path
Each module MUST follow a consistent pedagogical structure:
- Overview section stating module purpose and scope
- Explicit learning outcomes (3-5 measurable objectives)
- 3-5 chapters covering the topic progressively
- Runnable code examples demonstrating concepts (where applicable)
- 3 exercises per chapter with solutions in separate instructor files
- Recommended references and further reading

**Rationale**: Consistency enables students to focus on content rather than navigation. Explicit learning outcomes ensure measurable progress.

### II. Clear Academic Communication
Content MUST maintain:
- Academic tone: clear, concise, formal
- Example-driven explanations with concrete demonstrations
- Short summaries at key transition points
- Numbered lists for sequential processes
- Neutral, objective language for technical material
- Avoidance of colloquialisms and informal expressions

**Rationale**: Professional academic writing ensures credibility and accessibility for undergraduate to early graduate audiences and practicing engineers.

### III. Strict File Conventions
All files MUST adhere to:
- Filenames in kebab-case format (e.g., `inverse-kinematics.md`, `neural-motor-control.md`)
- YAML frontmatter in all markdown files containing:
  - `title`: Full descriptive title
  - `slug`: URL-friendly identifier
  - `sidebar_label`: Concise navigation label
  - `toc: true`: Enable table of contents
  - `description`: Brief summary (1-2 sentences)

**Rationale**: Consistent naming and metadata enable automated processing, navigation generation, and maintainability.

### IV. Hands-On Learning (NON-NEGOTIABLE)
Practical application MUST be embedded:
- Runnable code examples for implementable concepts
- Examples MUST be tested and functional
- Code snippets MUST include necessary context (imports, setup)
- Exercises MUST be solvable with module content alone
- Instructor solution files MUST be separate from student-facing content

**Rationale**: Active learning through implementation significantly improves retention and understanding in technical subjects.

### V. Assessment & Validation
Every chapter MUST include:
- 3 exercises ranging from recall to application/synthesis
- Exercises clearly numbered and progressive in difficulty
- Instructor solutions in dedicated files (not inline)
- Exercises aligned with chapter learning outcomes

**Rationale**: Structured assessment enables self-paced learning validation and provides teaching resources for instructors.

### VI. Attribution & Licensing
All content MUST comply with:
- CC BY-NC-SA 4.0 International License
- Educational and non-commercial use explicitly permitted
- Commercial use requires separate permission
- Proper attribution to original author(s)
- References and citations for external sources

**Rationale**: Open educational resources maximize impact while protecting author rights and enabling controlled distribution.

## Content Standards

### Scope & Boundaries
- **Target Audience**: Undergraduate to early graduate students; engineers transitioning to robotics
- **Language**: English (US academic standard)
- **Prerequisites**: Documented at module level where applicable
- **Depth**: Balance theoretical foundations with practical application

### Quality Requirements
- **Technical Accuracy**: All formulas, algorithms, and code MUST be verified
- **Consistency**: Terminology and notation MUST be consistent within and across modules
- **Completeness**: Each module MUST be self-contained with necessary background
- **Currency**: Content MUST reflect current industry practices and recent research where applicable

### Deliverable Format
- Markdown files with proper frontmatter (YAML)
- Code examples in fenced code blocks with language specification
- Mathematical notation using LaTeX/MathJax syntax
- Images and diagrams in standard formats (PNG, SVG, JPEG) with alt text
- File organization following modular structure (modules → chapters → sections)

## Safety & Ethics Requirements

### Ethical Considerations (MANDATORY)
- Ethics & Safety section MUST appear in:
  - Book preface (overview of ethical framework)
  - Module 4 or dedicated ethics module (detailed treatment)
- Safety considerations MUST be highlighted where applicable

### Content Restrictions
- **Prohibited**: Step-by-step instructions for harmful or weaponizable applications
- **Prohibited**: Detailed guidance enabling unauthorized surveillance or privacy violations
- **Permitted**: High-level hardware concepts and architectural discussions
- **Permitted**: Academic discussion of dual-use technologies with ethical context
- **Required**: Clear safety warnings for physical hardware interactions

**Rationale**: Educational content must promote responsible development while avoiding direct enablement of harmful applications.

## Governance

### Amendment Process
This constitution governs all content generation and modification. Amendments require:
1. Documented justification for the change
2. Impact analysis on existing content and templates
3. Version increment following semantic versioning (MAJOR.MINOR.PATCH)
4. Update of dependent templates and documentation

### Version Control Policy
- **MAJOR**: Fundamental changes to pedagogical approach or content structure
- **MINOR**: New principles added or existing principles materially expanded
- **PATCH**: Clarifications, wording improvements, formatting fixes

### Compliance Verification
- All generated content MUST be validated against these principles
- Generation reports MUST summarize compliance with file conventions and structure
- Non-compliance MUST be documented with justification or corrected

### Change Management
- Template files (spec, plan, tasks) MUST remain compatible with constitution principles
- Command files MUST reference constitution for validation gates
- Constitution changes MUST trigger review of dependent templates

### Output & Repository Policy
- **NEVER** push generated content to git automatically
- All files MUST be generated locally for review
- Generation commands MUST produce a summary report including:
  - File counts by type
  - Skipped items with reasons
  - Validation results

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
