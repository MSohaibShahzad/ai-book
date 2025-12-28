# Physical-AI & Humanoid Robotics Constitution

<!--
Sync Impact Report:
Version: 1.3.0 → 1.4.0
Modified principles: None
Added sections:
  - Personalized Chapter Content Principles (3 principles for background-based personalization)
Removed sections: None
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

## RAG Chatbot Principles

### VII. Content Grounding (MANDATORY)
The chatbot MUST:
- Answer questions based exclusively on textbook content
- Use Retrieval-Augmented Generation (RAG) with verified textbook chunks
- When user highlights text, prioritize that selection as primary context
- State "Not covered in this textbook" when information is unavailable
- Never fabricate or extrapolate beyond documented content

**Rationale**: Educational integrity requires factual accuracy grounded in authoritative course materials.

### VIII. Simple Reference Attribution
The chatbot MUST:
- Include simple references in responses (module name and/or chapter name)
- Format references as: "Source: Module X, Chapter Y" or "Source: [Chapter Title]"
- Avoid complex citation formats (no page numbers, line numbers, or academic citations)
- Enable students to locate source material easily

**Rationale**: Students need to verify information and explore topics deeper without citation complexity.

### IX. Safety & Ethics Compliance
The chatbot MUST:
- Refuse requests for harmful robotics applications
- Refuse detailed instructions for weaponization or unauthorized surveillance
- Redirect harmful queries to the Ethics & Safety section of the textbook
- Maintain the same ethical boundaries as the textbook content

**Rationale**: Educational AI assistants must uphold the same safety standards as course materials.

### X. Teacher-Like Interaction Style
The chatbot MUST:
- Use clear, helpful, supportive tone
- Break down complex topics into digestible explanations
- Provide examples when helpful (drawn from textbook)
- Encourage exploration of related textbook sections
- Avoid condescension or overly casual language

**Rationale**: Educational assistants should model effective teaching communication.

### XI. Highlight-Based Context Priority
When user highlights textbook content, the chatbot MUST:
- Treat highlighted text as primary context for the query
- Retrieve additional relevant chunks only if needed for complete answer
- Reference the highlighted section explicitly in response
- Support queries like "explain this" or "what does this mean"

**Rationale**: Context-aware assistance improves comprehension of difficult passages.

### XII. Markdown Response Formatting
The chatbot MUST format responses with:
- Simple Markdown (headings, lists, code blocks, bold/italic)
- Code blocks with appropriate language tags (python, cpp, bash, etc.)
- Mathematical notation using LaTeX/MathJax where applicable
- No HTML or complex formatting
- Consistent structure: answer → explanation → reference

**Rationale**: Clean formatting improves readability in the Docusaurus chat widget.

### XIII. Scope & Feature Boundaries
The chatbot implementation MUST:
- Integrate as a chat widget within Docusaurus
- Support free-text questions about textbook content
- Support highlight-based contextual queries
- Use backend RAG system to retrieve textbook chunks
- Maintain conversation history within session (no persistent user profiles)

**Rationale**: Clear scope boundaries ensure focused development and privacy compliance.

## Authentication & User Management Principles

### XIV. Documentation-Driven Authentication (MANDATORY)
Authentication implementation MUST:
- Use Better-Auth as the authentication library
- Consult official Better-Auth documentation via Context7 MCP server before writing code
- Follow Better-Auth best practices and recommended patterns
- Verify implementation approaches against current documentation
- Never rely on outdated examples or assumed API usage

**Rationale**: Authentication security requires accurate, up-to-date implementation following library maintainer guidance. Documentation-first approach prevents security vulnerabilities from outdated patterns.

### XV. User Background Collection
During signup, the system MUST:
- Collect user software background (programming languages, frameworks, experience level)
- Collect user hardware background (robotics platforms, sensors, actuators experience)
- Store background as structured user metadata (not in main credentials table)
- Make background fields optional (users can skip or update later)
- Use background data to personalize learning recommendations

**Rationale**: Understanding user background enables adaptive content recommendations and improves learning experience for diverse skill levels.

### XVI. Secure Credential Management
Authentication system MUST:
- Never expose API keys, secrets, or credentials on the frontend
- Store sensitive configuration in backend environment variables only
- Use secure session management (httpOnly cookies, CSRF protection)
- Implement password hashing with industry-standard algorithms
- Follow Better-Auth security defaults without weakening settings

**Rationale**: Educational platforms handle student data and must maintain strict security standards to protect user privacy and prevent unauthorized access.

### XVII. Simple & Minimal Authentication
Authentication implementation MUST:
- Provide email/password signup and signin as primary method
- Keep authentication UI simple and accessible
- Avoid complex multi-step registration flows
- Provide clear error messages for authentication failures
- Support password reset functionality

**Rationale**: Educational platforms should minimize barriers to entry while maintaining security. Simple authentication reduces friction for students focused on learning.

## Localization & Translation Principles

### XVIII. Meaning-Preserving Translation (MANDATORY)
Translation to Urdu MUST:
- Preserve the original meaning and intent of all content
- Maintain technical accuracy across languages
- Keep mathematical notation and formulas unchanged
- Preserve code examples exactly as written (no translation of code)
- Maintain the same pedagogical structure and learning outcomes

**Rationale**: Educational content must retain its accuracy and effectiveness when translated. Technical precision cannot be compromised for linguistic fluency.

### XIX. Technical Term Handling
When translating to Urdu, the system MUST:
- Keep domain-specific technical terms in English (e.g., "kinematics", "actuator", "neural network")
- Use commonly accepted Urdu terms where they exist and are widely understood
- Provide English technical terms in parentheses after Urdu equivalents when helpful
- Maintain consistency in technical terminology across all translated chapters
- Never create new Urdu translations for established English technical terms

**Rationale**: Technical education relies on standardized terminology. Mixing languages appropriately aids comprehension for students familiar with English technical literature.

### XX. Simple & Readable Urdu
Translated content MUST:
- Use simple, clear, and modern Urdu language
- Avoid overly formal or archaic Urdu expressions
- Write in a style accessible to undergraduate students
- Maintain short sentences and clear paragraph structure
- Use active voice where the English original uses active voice

**Rationale**: Translation should enhance accessibility, not create additional linguistic barriers. Simple language ensures the focus remains on learning technical content.

### XXI. Authenticated Translation Access
Urdu translation feature MUST:
- Only be available to logged-in users
- Be triggered by a button at the start of each chapter
- Generate translations on-demand (not pre-generated for all content)
- Track translation usage per user session
- Display both English original and Urdu translation options

**Rationale**: Authenticated access enables usage tracking and protects against abuse. On-demand translation reduces infrastructure costs.

### XXII. Original Content Preservation (NON-NEGOTIABLE)
The translation system MUST:
- NEVER modify or replace the original English textbook content
- Store translations separately from source markdown files
- Keep English content as the authoritative version
- Enable users to toggle between English and Urdu views
- Maintain English content in version control as the single source of truth

**Rationale**: The English textbook is the authoritative version. Translations are supplementary aids and must not interfere with the original content or its maintenance.

## Personalized Chapter Content Principles

### XXIII. Background-Based Personalization (MANDATORY)
Personalized chapter content MUST:
- Adapt explanations based on user's software and hardware background levels
- Use the user's stored background metadata (softwareBackground, hardwareBackground, interestArea)
- Adjust technical depth and terminology to match user's experience level
- Provide more foundational explanations for beginners, advanced insights for experts
- Maintain the same learning outcomes as the original chapter

**Rationale**: Adaptive learning improves comprehension and engagement by meeting students at their current knowledge level.

### XXIV. Original Content Preservation for Personalization (NON-NEGOTIABLE)
The personalization system MUST:
- NEVER modify or replace the original textbook markdown files
- Generate personalized content on-demand only when triggered by user
- Display personalized content as an overlay or alternative view, not as replacement
- Enable users to toggle between original and personalized versions
- Keep original English content as the authoritative version in all cases

**Rationale**: Original content must remain unchanged to ensure consistency, maintainability, and serve as the canonical reference for all users.

### XXV. Authenticated Personalization Access
Personalized content feature MUST:
- Only be available to logged-in users with complete background profiles
- Be triggered by a "Personalize for Me" button at the start of each chapter
- Generate personalized content on-demand using AI (not pre-generated)
- Preserve all technical accuracy, code examples, and formulas from original
- Use educational tone appropriate to user's level but remain professional
- Never hallucinate or add information not present in original chapter

**Rationale**: Personalization requires user context and should be resource-efficient through on-demand generation. Accuracy must be preserved to maintain educational integrity.

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

**Version**: 1.4.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-23
