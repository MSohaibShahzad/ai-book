---
id: 0002
title: RAG Chatbot Constitution Principles
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: msohaibshahzad
command: /sp.constitution
labels: ["constitution", "rag-chatbot", "principles", "ai-assistant"]
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

Title: RAG Chatbot for Physical-AI Textbook

Purpose:
  Define rules for a chatbot that answers questions based on the textbook content and user-selected text only.

Principles:
  - Answers must be grounded in the book's content.
  - When user highlights text, use that text as primary context.
  - Show simple references (module / chapter names).
  - No hallucination; say "Not in the textbook" if unsure.
  - No harmful robotics instructions.
  - Tone: clear, helpful, teacher-like.
  - Format answers in simple Markdown.

Scope:
  - Chat widget inside the Docusaurus textbook.
  - Backend RAG: retrieves chunks from the book.
  - Supports free questions + highlight-based questions.

## Response snapshot

Updated constitution from v1.0.0 to v1.1.0 (MINOR version bump).

Added new section: "RAG Chatbot Principles" with 7 principles (VII-XIII):
- VII. Content Grounding (MANDATORY): Answers based exclusively on textbook content via RAG
- VIII. Simple Reference Attribution: Module/chapter references without complex citations
- IX. Safety & Ethics Compliance: Refuse harmful robotics applications
- X. Teacher-Like Interaction Style: Clear, helpful, supportive educational tone
- XI. Highlight-Based Context Priority: Prioritize user-selected text as primary context
- XII. Markdown Response Formatting: Simple markdown with code blocks and math notation
- XIII. Scope & Feature Boundaries: Docusaurus widget, free-text + highlight queries, session history only

All principles include explicit rationales. Templates remain compatible.

## Outcome

- ✅ Impact: Constitution v1.1.0 adds comprehensive chatbot behavior principles
- 🧪 Tests: None required (governance document)
- 📁 Files: .specify/memory/constitution.md (updated)
- 🔁 Next prompts: Create feature spec for RAG chatbot implementation (/sp.specify)
- 🧠 Reflection: MINOR bump appropriate - new section added without modifying existing principles

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (constitution update)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
