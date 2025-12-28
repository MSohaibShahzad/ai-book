---
id: 0002
title: Auth Signup Signin Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-auth-signup-signin
branch: 003-auth-signup-signin
user: sohaib
command: /sp.plan
labels: ["plan", "architecture", "better-auth", "authentication", "user-metadata", "session-management"]
links:
  spec: specs/003-auth-signup-signin/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-auth-signup-signin/plan.md
  - specs/003-auth-signup-signin/research.md
  - specs/003-auth-signup-signin/data-model.md
  - specs/003-auth-signup-signin/quickstart.md
  - specs/003-auth-signup-signin/contracts/auth-api.openapi.yaml
  - CLAUDE.md
tests:
  - none
---

## Prompt

Title: Signup & Signin — Plan

Architecture:
  - Frontend: Signup and Signin UI embedded in the textbook site.
  - Auth Provider: Better-Auth.
  - Documentation Access: Context7 MCP server used by the agent to read official Better-Auth documentation.
  - Backend and frontend code written according to documented Better-Auth practices.

Data:
  - User metadata includes software background, hardware background, and interests.

Integration:
  - Frontend integrates Better-Auth according to documentation.
  - Authenticated user profile data can be shared with the RAG backend for personalization.

Deployment:
  - Auth works on the live deployed textbook.
  - Secrets managed via environment variables.

## Response snapshot

Created comprehensive implementation plan for authentication feature using Better-Auth with custom user metadata.

**Phase 0: Research (Completed)**:
- Consulted Better-Auth documentation via Context7 MCP (/better-auth/better-auth, 2274 code snippets)
- Selected Better-Auth v1.3+ for framework-agnostic TypeScript authentication
- Researched custom schema extension via `additionalFields` API
- Defined session management strategy (7-day expiration, httpOnly cookies)
- Documented chatbot integration pattern (session middleware → user metadata → LLM context)

**Phase 1: Design (Completed)**:
- Created data-model.md with User/Session entity schemas
  - User table: 10 fields including softwareBackground, hardwareBackground, interestArea
  - Session table: 8 fields with userId foreign key, CASCADE DELETE
  - Indexes on email (unique), token (unique), userId, expiresAt
- Generated OpenAPI 3.1 specification (auth-api.openapi.yaml)
  - 6 endpoints: signup, signin, signout, session, profile GET/PATCH
  - Full request/response schemas with examples
  - Error response definitions (400, 401, 409, 500)
- Created quickstart.md with step-by-step developer setup
  - Installation commands for Better-Auth + Drizzle ORM
  - Code examples for backend config and frontend components
  - Testing instructions and troubleshooting guide

**Technical Decisions**:
- Web application architecture (backend: FastAPI + Better-Auth TypeScript, frontend: React/Docusaurus)
- PostgreSQL with extended user schema via Better-Auth additionalFields
- Session management: 7-day expiration, httpOnly/secure cookies, CSRF protection
- Three custom metadata fields (enum types) for personalization
- Chatbot integration via session middleware in FastAPI

**Constitution Compliance**:
- ✅ XIV. Documentation-Driven Authentication: Better-Auth docs consulted via Context7
- ✅ XV. User Background Collection: 3 custom fields with sensible defaults
- ✅ XVI. Secure Credential Management: httpOnly cookies, env var secrets, bcrypt hashing
- ✅ XVII. Simple & Minimal Authentication: Email/password only, 5-field signup form

**Artifacts Generated**:
1. plan.md: Complete implementation plan with technical context, phase breakdown, risk analysis
2. research.md: Better-Auth architecture research, schema extension strategy, security configuration
3. data-model.md: Database schema definitions, TypeScript types, validation rules
4. contracts/auth-api.openapi.yaml: OpenAPI 3.1 API specification
5. quickstart.md: Developer setup guide with code examples

**Architecture Highlights**:
- Backend: Better-Auth TypeScript server with PostgreSQL adapter, mounted at /api/auth/*
- Frontend: Better-Auth React client, signup/signin forms, session hooks
- Database: Extended user table with 3 custom fields, session table with 7-day expiration
- Integration: FastAPI session middleware passes user metadata to RAG chatbot

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation via /sp.tasks
- 🧪 Tests: Testing strategy defined (pytest backend, Jest frontend, Playwright E2E)
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/auth-api.openapi.yaml, CLAUDE.md updated
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation task breakdown
- 🧠 Reflection: Planning complete with all Phase 0 (research) and Phase 1 (design) artifacts generated. Constitution gates passed. Better-Auth selected after Context7 documentation review (2274 snippets, 84.8 benchmark score). Custom schema extension via additionalFields API enables personalization without migration complexity. Suggested ADR for Better-Auth TypeScript/Python integration decision.

## Evaluation notes (flywheel)

- Failure modes observed: update-agent-context.sh template error (non-blocking, manually updated CLAUDE.md)
- Graders run and results (PASS/FAIL): Constitution Check PASSED (all 4 auth principles met)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
