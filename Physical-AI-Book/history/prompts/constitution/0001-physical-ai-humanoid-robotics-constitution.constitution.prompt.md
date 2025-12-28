---
id: 0001
title: Physical AI Humanoid Robotics Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: sohaib
command: /sp.constitution
labels: ["constitution", "setup", "pedagogy", "safety", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

Title: Physical-AI & Humanoid Robotics — Constitution
Purpose: Establish the project's guiding principles. These principles will govern content style, technical constraints, file/filename rules, and ethical/safety considerations for generation.
Deliverables:
  - A JSON/YAML style constitution saved to .specify/sp.constitution (or inserted into project metadata).
Constitution Text:
  - Project ID: physical-ai-humanoid-robotics
  - Project Title: "Physical-AI & Humanoid Robotics"
  - Author: "Sohaib"
  - Language: "English"
  - Audience: "Undergraduate to early graduate students; engineers learning applied robotics"
  - Tone & Style: "Academic, clear, concise, example-driven. Use short summaries, numbered lists, and explicit learning outcomes. Prefer neutral, formal language for technical material."
  - Pedagogy: "Each module must include: overview, learning outcomes, 3–5 chapters, runnable examples where applicable, 3 exercises per chapter with answers in separate instructor files, and recommended references."
  - File Rules: "Filenames must be kebab-case. All markdown files must include YAML frontmatter with: title, slug, sidebar_label, toc: true, and description (short)."
  - Safety & Ethics: "Include an Ethics & Safety note in Module 4 and in the book preface. Do not provide step-by-step instructions for harmful or weaponizable hardware. High level hardware conceptual content allowed."
  - Licensing: "CC BY-NC-SA (allow reuse for educational, non-commercial purposes)."
  - Output Policy: "Never push to git. Only generate files locally. Produce a final generation report summarizing counts and any skipped items."

## Response snapshot

Constitution created at .specify/memory/constitution.md with the following structure:

**Version**: 1.0.0 (Initial creation)

**Core Pedagogical Principles** (6 principles):
1. Structured Learning Path - Consistent module structure with learning outcomes, chapters, examples, exercises
2. Clear Academic Communication - Academic tone, example-driven, neutral language
3. Strict File Conventions - Kebab-case filenames, YAML frontmatter requirements
4. Hands-On Learning (NON-NEGOTIABLE) - Runnable examples, tested code, separate instructor solutions
5. Assessment & Validation - 3 exercises per chapter, progressive difficulty, aligned with outcomes
6. Attribution & Licensing - CC BY-NC-SA 4.0, proper attribution

**Content Standards**:
- Scope: undergraduate to graduate students, engineers
- Quality: technical accuracy, consistency, completeness, currency
- Format: Markdown with frontmatter, code blocks, LaTeX math, standard images

**Safety & Ethics Requirements**:
- Mandatory ethics sections in preface and Module 4
- Prohibited: harmful/weaponizable step-by-step instructions
- Permitted: high-level concepts with ethical context
- Required: safety warnings for hardware

**Governance**:
- Amendment process with justification and impact analysis
- Semantic versioning (MAJOR.MINOR.PATCH)
- Compliance verification for all generated content
- No automatic git pushes, local generation with reports

## Outcome

- ✅ Impact: Foundation established for textbook content generation; all future content must comply with pedagogical, safety, and formatting principles
- 🧪 Tests: N/A (constitution document, no executable tests)
- 📁 Files: Created .specify/memory/constitution.md (156 lines)
- 🔁 Next prompts: Ready for /sp.specify to create feature specifications for book modules
- 🧠 Reflection: Constitution successfully establishes clear boundaries for educational content while balancing academic rigor with safety considerations

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (initial constitution)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test constitution compliance during first module specification
