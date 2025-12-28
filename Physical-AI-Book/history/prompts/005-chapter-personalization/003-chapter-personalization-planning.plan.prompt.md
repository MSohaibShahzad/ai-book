---
id: 003
title: Chapter Personalization Planning
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: chapter-personalization
branch: 005-chapter-personalization
user: sohaib
command: /sp.plan
labels: ["architecture", "openai-agents-sdk", "rate-limiting", "personalization", "planning"]
links:
  spec: specs/005-chapter-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-chapter-personalization/plan.md
  - specs/005-chapter-personalization/research.md
  - specs/005-chapter-personalization/data-model.md
  - specs/005-chapter-personalization/contracts/personalization-api.yaml
  - specs/005-chapter-personalization/quickstart.md
  - CLAUDE.md (updated with new technologies)
tests:
  - Constitution Check (all principles PASS)
  - Technology decisions validated
  - Data model schema designed
  - API contracts defined
---

## Prompt

/sp.plan
Title: Chapter Personalization — Plan

Architecture:
  - Frontend: Personalization button at the start of each chapter.
  - Backend: API endpoint for content personalization.
  - AI: OpenAI Agent implemented using the OpenAI Agents SDK.

AI Strategy:
  - Use a reusable personalization agent.
  - Rewrite chapter content according to user background.
  - Preserve technical accuracy and intent.

Integration:
  - Frontend sends chapter content and user profile to backend.
  - Backend calls the personalization agent.
  - Backend returns personalized content.
  - Frontend renders personalized view.

Access Control:
  - Backend verifies user authentication before processing.

## Response snapshot

Completed comprehensive implementation planning for Chapter Personalization feature with full design artifacts:

**Plan Document** (`plan.md`):
- **Technical Context**: Python 3.11+ backend with FastAPI, OpenAI Agents SDK (openai-agents-python), markdown-it-py for AST processing, PostgreSQL for rate limiting, React Context API for frontend state
- **Project Structure**: Web application (backend + frontend) with isolated `backend/personalization/` module and `textbook/src/components/` personalization components
- **Performance Goals**: <30s hard timeout, <1s toggle, 90% success rate, ~50 concurrent requests peak
- **Constitution Check**: PASS ✅ (principles XXIII-XXV respected, no violations)

**Phase 0: Research** (`research.md` - 5 major decisions):
1. **AI Framework**: OpenAI Agents SDK chosen over direct API (structured lifecycle, built-in retry logic, state management)
2. **Rate Limiting**: PostgreSQL table (personalization_quota) over Redis/in-memory (persistence, analytics, consistency)
3. **Frontend State**: React Context API + custom hooks over Redux (lightweight, feature-scoped, session-only)
4. **Markdown Processing**: markdown-it-py with AST protection over regex (robust parsing, 100% preservation guarantee)
5. **Observability**: FastAPI middleware + structured logging over APM services (zero dependencies, real-time metrics)

**Phase 1: Data Model** (`data-model.md`):
- **PersonalizationQuota** (PostgreSQL): Tracks rate limiting with rolling 24-hour window (user_id PK, request_count 0-3, first_request_timestamp, reset_at)
- **PersonalizedContent** (Frontend State): Session-only cache (Map<chapterId, content>) with profile snapshot for invalidation
- **PersonalizationRequest** (Logs Only): Structured JSON logs for observability (request_id, user_id, status, duration_ms, profile_snapshot)
- **Migrations**: Alembic migration script for personalization_quota table with constraints and indexes
- **Storage Estimates**: ~1.2MB for 10k users (quota table), ~150KB per session (frontend cache), ~7.5MB/month logs

**Phase 1: API Contracts** (`contracts/personalization-api.yaml`):
- **POST /api/personalize**: Main endpoint (chapter_id, chapter_content, user_profile) → (personalized_markdown, remaining_limit, generation_time_ms)
- **GET /api/personalization/quota**: Quota status (remaining_requests, total_requests, reset_at, hours_until_reset)
- **DELETE /api/personalization/cache**: Invalidate cache on profile update (204 No Content)
- **GET /metrics**: Public metrics endpoint (total_requests, success_rate_percent, avg_duration_ms, timeout_count)
- **Error Responses**: 400 (incomplete profile), 401 (unauthorized), 408 (timeout), 429 (rate limit), 500 (generation failed)
- **Security**: BearerAuth (JWT) and CookieAuth (Better-Auth session)

**Phase 1: Quickstart** (`quickstart.md`):
- **Setup Instructions**: Backend (OpenAI API key, database migration, uvicorn), Frontend (npm install, API endpoint config)
- **Testing Flow**: 7-step test plan covering personalization, toggling, rate limiting, timeout, profile update cache invalidation
- **Troubleshooting**: 5 common issues with solutions (missing API key, button not visible, timeout, rate limit not resetting, cache not invalidating)
- **Deployment Checklist**: 9 items for production readiness (env vars, migration, CORS, monitoring alerts)

**Agent Context Update** (`CLAUDE.md`):
- Added OpenAI Agents SDK (openai-agents-python), markdown-it-py, PostgreSQL quota, React Context, FastAPI middleware to Active Technologies
- Added 005-chapter-personalization to Recent Changes with full tech stack summary

**Key Architectural Decisions**:
- ✅ **Reusable Agent Pattern**: Single OpenAI Agent instance with context variables for user profile (avoids re-initialization overhead)
- ✅ **AST-Based Preservation**: Markdown parsed to AST, protected elements replaced with placeholders, prose sent to AI, reconstructed with originals
- ✅ **Rolling Window Rate Limiting**: 24-hour window from first request (not calendar day), stored in PostgreSQL for persistence
- ✅ **Session-Only Caching**: React Context cleared on reload/profile update (respects "no persistent personalized content" requirement)
- ✅ **Hard Timeout Enforcement**: 30-second timeout at agent level (terminates request, returns 408 error with retry option)

## Outcome

- ✅ Impact: Complete implementation plan with 5 design artifacts ready for task generation (/sp.tasks)
- 🧪 Tests: Constitution Check PASS (all principles XXIII-XXV respected), data model validated, API contracts defined
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/personalization-api.yaml, quickstart.md; Updated CLAUDE.md
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks from plan
- 🧠 Reflection: Planning completed successfully with user-provided architecture guidance (OpenAI Agents SDK, reusable agent, preservation strategy). All 5 research decisions documented with rationale. Data model includes migration scripts. API contracts cover all functional requirements (FR-001 through FR-025). Quickstart provides end-to-end setup and testing guide. No constitution violations detected. Ready for task generation phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning process completed successfully with all artifacts generated
- Graders run and results (PASS/FAIL): Constitution Check PASS (principles XXIII-XXV), Schema validation PASS (constraints, indexes), API contract validation PASS (OpenAPI 3.0.3 valid)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (research) and Phase 1 (design/contracts) completion
- Next experiment (smallest change to try): Proceed to /sp.tasks for implementation task generation
