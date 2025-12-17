---
id: 0003
title: Better-Auth Signup Signin Principles
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-auth-signup-signin
branch: 003-auth-signup-signin
user: sohaib
command: /sp.constitution
labels: ["constitution", "authentication", "better-auth", "user-management", "security"]
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

Title: Signup & Signin using Better-Auth

Purpose:
  Define rules for implementing signup and signin using Better-Auth with accurate, documentation-driven code.

Principles:
  - Authentication must be implemented using Better-Auth.
  - The agent should consult official Better-Auth documentation via the Context7 MCP server before writing code.
  - Collect user background during signup.
  - Store background as user metadata.
  - Do not expose credentials or secrets on the frontend.
  - Keep authentication simple, secure, and minimal.

Scope:
  - Signup and Signin inside the textbook website.
  - Software and hardware background collection at signup.

## Response snapshot

Updated constitution from v1.1.0 to v1.2.0 (MINOR version bump).

Added new section: "Authentication & User Management Principles" with 4 principles (XIV-XVII):
- XIV. Documentation-Driven Authentication (MANDATORY): Use Better-Auth, consult official docs via Context7 MCP, follow best practices, verify against current documentation
- XV. User Background Collection: Collect software/hardware background during signup, store as structured metadata (optional fields), enable personalized learning recommendations
- XVI. Secure Credential Management: Never expose secrets on frontend, use backend env vars, secure sessions (httpOnly cookies, CSRF), industry-standard password hashing, follow Better-Auth security defaults
- XVII. Simple & Minimal Authentication: Email/password signup/signin, simple accessible UI, avoid complex flows, clear error messages, password reset support

All principles include explicit rationales focusing on security, user experience, and educational platform requirements. Templates remain compatible (plan, spec, tasks).

## Outcome

- ✅ Impact: Constitution v1.2.0 adds comprehensive authentication principles for Better-Auth integration
- 🧪 Tests: None required (governance document)
- 📁 Files: .specify/memory/constitution.md (updated)
- 🔁 Next prompts: Create feature spec for authentication implementation (/sp.specify)
- 🧠 Reflection: MINOR bump appropriate - new section added without modifying existing principles. Documentation-first approach ensures secure implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (constitution update)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
