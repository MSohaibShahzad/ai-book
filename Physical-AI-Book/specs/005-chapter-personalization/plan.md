# Implementation Plan: Chapter Personalization

**Branch**: `005-chapter-personalization` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-chapter-personalization/spec.md` + user architecture guidance

## Summary

Implement an AI-powered chapter personalization system that adapts textbook content to individual user backgrounds. Users can click a "Personalize for Me" button on any chapter to receive content tailored to their software/hardware experience level and interests. The system uses OpenAI Agents SDK with a reusable personalization agent, enforces rate limiting (3 requests/day), implements 30-second hard timeout, and caches personalized content during the session. Frontend displays personalized content with toggle functionality to compare with original, while backend orchestrates AI generation and tracks usage metrics.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.3+ (React 18+ / Docusaurus)
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: React Context API (state management), Docusaurus SDK (chapter access)
- Backend: FastAPI 0.104+, OpenAI Agents SDK (openai-agents-python), markdown-it-py (markdown processing), PostgreSQL adapter (rate limiting storage)

**Storage**:
- PostgreSQL (Neon - existing): Personalization quota tracking (user_id, request_count, first_request_timestamp, reset_at)
- In-memory/session: Cached personalized content (not persisted)

**Testing**:
- Frontend: Jest (unit tests for components), React Testing Library (integration tests)
- Backend: pytest (API endpoint tests, agent tests, rate limiting tests)

**Target Platform**:
- Frontend: Web (Docusaurus static site with client-side React)
- Backend: Linux server (Vercel serverless functions or similar)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Personalization generation: < 30 seconds (hard timeout)
- Toggle between views: < 1 second
- Rate limit check: < 100ms
- 90% success rate for personalization requests

**Constraints**:
- 30-second hard timeout for AI generation
- 3 personalization requests per user per day
- Session-only caching (no persistent storage of personalized content)
- Original content must NEVER be modified
- Preserve 100% accuracy of code, formulas, diagrams

**Scale/Scope**:
- Expected users: Thousands of students
- Expected chapters: ~50-100 textbook chapters
- Daily personalization requests: Estimated 100-500 (based on user base × 3 requests/day limit)
- Concurrent requests: Peak of ~50 concurrent personalizations

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Applicable Principles

**✅ XXIII. Background-Based Personalization (MANDATORY)**
- Plan includes user profile metadata (softwareBackground, hardwareBackground, interestArea) as input to AI agent
- Adaptation logic defined in OpenAI agent instructions
- Learning outcomes preservation enforced in agent prompts

**✅ XXIV. Original Content Preservation for Personalization (NON-NEGOTIABLE)**
- Personalized content generated on-demand, never written to markdown files
- Session-only caching (no database persistence)
- Toggle functionality allows users to view original at any time
- Original content remains authoritative source

**✅ XXV. Authenticated Personalization Access**
- "Personalize for Me" button only visible to authenticated users with complete profiles
- Backend validates JWT/session before processing requests
- Rate limiting enforced per user (3/day)
- AI generation on-demand (not pre-generated)

**✅ XIV. Documentation-Driven Authentication (MANDATORY)**
- Reuses existing Better-Auth system (feature 003)
- No new authentication patterns introduced

**✅ XVI. Secure Credential Management**
- OpenAI API keys stored in backend environment variables only
- Never exposed to frontend
- Session cookies used for authentication (existing pattern)

### Constitution Gates: PASS ✅

No violations detected. All personalization principles (XXIII-XXV) are respected. Original content preservation is enforced. Authentication reuses existing secure patterns.

## Project Structure

### Documentation (this feature)

```text
specs/005-chapter-personalization/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology choices)
├── data-model.md        # Phase 1 output (entities and schema)
├── quickstart.md        # Phase 1 output (setup guide)
├── contracts/           # Phase 1 output (API contracts)
│   └── personalization-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created yet)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend detected)

backend/
├── personalization/
│   ├── __init__.py
│   ├── agent.py              # OpenAI personalization agent
│   ├── api.py                # FastAPI endpoints for personalization
│   ├── rate_limiter.py       # Rate limiting logic
│   ├── metrics.py            # Observability tracking
│   └── models.py             # Personalization quota model
├── tests/
│   ├── test_personalization_agent.py
│   ├── test_personalization_api.py
│   └── test_rate_limiter.py
└── migrations/
    └── 005_add_personalization_quota.sql

textbook/ (Docusaurus frontend)
├── src/
│   ├── components/
│   │   ├── PersonalizeButton.tsx      # "Personalize for Me" button component
│   │   ├── PersonalizedChapter.tsx    # Personalized content renderer
│   │   ├── ViewToggle.tsx             # Original/Personalized toggle
│   │   └── RateLimitDisplay.tsx       # Shows remaining limit
│   ├── contexts/
│   │   └── PersonalizationContext.tsx # State management for personalization
│   ├── hooks/
│   │   ├── usePersonalization.ts      # Hook for personalization API calls
│   │   └── useRateLimit.ts            # Hook for rate limit tracking
│   └── services/
│       └── personalizationService.ts  # API client for backend
└── tests/
    ├── PersonalizeButton.test.tsx
    ├── PersonalizedChapter.test.tsx
    └── usePersonalization.test.ts
```

**Structure Decision**: Web application structure selected because feature requires both frontend UI (Personalize button, toggle, personalized content display) and backend API (AI agent orchestration, rate limiting, metrics tracking). Reuses existing `backend/` and `textbook/` directories from previous features (002-rag-chatbot, 003-auth, 004-urdu-translation). Backend personalization logic isolated in `backend/personalization/` module. Frontend components in `textbook/src/components/` following established Docusaurus patterns.

## Complexity Tracking

> No constitution violations detected - this section intentionally left blank.

## Phase 0: Research & Technology Decisions

### Decision 1: OpenAI Agents SDK vs Direct API

**Chosen**: OpenAI Agents SDK (openai-agents-python)

**Rationale**:
- Provides structured agent lifecycle management with state handling
- Built-in retry logic and error handling
- Easier to define reusable personalization agent with consistent behavior
- Supports system prompts, tools, and structured outputs
- Better abstraction than raw OpenAI API for complex multi-step personalization

**Alternatives Considered**:
- **Direct OpenAI API calls**: More control but requires manual retry logic, state management, and error handling. Would increase code complexity.
- **LangChain**: More heavyweight framework with unnecessary abstractions for this focused use case.

**Implementation Notes**:
- Agent defined with system prompt containing personalization instructions
- User profile metadata passed as context variables
- Chapter markdown passed as user message
- Agent configured with 30-second timeout
- Reusable across all personalization requests

### Decision 2: Rate Limiting Storage Strategy

**Chosen**: PostgreSQL table (personalization_quota)

**Rationale**:
- Persistent across server restarts (unlike in-memory)
- Already using PostgreSQL for user data (consistency)
- Simple schema with 4 fields: user_id, request_count, first_request_timestamp, reset_at
- Supports rolling 24-hour window calculation
- Can be queried for analytics and monitoring

**Alternatives Considered**:
- **Redis**: Would require new infrastructure dependency. Overkill for simple counter with infrequent resets.
- **In-memory (Python dict)**: Lost on server restart, no persistence across deployments.
- **User table column**: Less flexible for tracking reset timestamps and historical data.

**Implementation Notes**:
- Row per user created on first personalization request
- `request_count` incremented on each request
- `reset_at` calculated as `first_request_timestamp + 24 hours`
- Background job or lazy reset when `now() > reset_at`
- Index on user_id for fast lookups

### Decision 3: Frontend State Management

**Chosen**: React Context API + custom hooks

**Rationale**:
- Lightweight solution for personalization state (no Redux needed)
- Scoped to personalization feature (doesn't pollute global state)
- Custom hooks (`usePersonalization`, `useRateLimit`) provide clean API
- Session-only state matches requirement (no persistence needed)
- Integrates well with existing Docusaurus React components

**Alternatives Considered**:
- **Redux**: Too heavyweight for session-only state with no complex cross-component sharing.
- **Component state only**: Would require prop drilling for rate limit display across multiple components.
- **LocalStorage**: Violates requirement that personalized content should not persist across sessions.

**Implementation Notes**:
- `PersonalizationContext` stores: personalizedContent (Map<chapterId, markdown>), rateLimitRemaining, currentView ("original" | "personalized")
- Context cleared on session end or profile update
- Hooks abstract API calls and state updates

### Decision 4: Markdown Processing for Preservation

**Chosen**: markdown-it-py with AST-based selective rewriting

**Rationale**:
- Same library used in 004-urdu-translation (consistency)
- Parses markdown to AST, allowing selective rewriting of prose while preserving code blocks, LaTeX, frontmatter
- Can identify and protect special elements (code fences, math blocks, diagrams)
- Battle-tested for technical content preservation

**Alternatives Considered**:
- **Regex-based replacement**: Fragile, error-prone with nested markdown structures
- **LLM-only approach**: Risk of AI hallucinating changes to code/formulas despite instructions
- **CommonMark-py**: Less feature-rich, no plugin support for LaTeX/frontmatter

**Implementation Notes**:
- Parse original markdown to AST
- Extract protected elements (code blocks, math, YAML frontmatter, images)
- Replace with placeholders (e.g., `{{CODE_BLOCK_1}}`)
- Send prose-only markdown to AI agent
- Reconstruct markdown by replacing placeholders with original protected elements
- Validate output structure matches input structure

### Decision 5: Observability Strategy

**Chosen**: FastAPI middleware + structured logging + metrics endpoint

**Rationale**:
- FastAPI middleware automatically captures all personalization requests
- Structured logging (JSON format) enables easy parsing for monitoring
- Metrics endpoint exposes Prometheus-compatible metrics for dashboards
- No external dependency (e.g., Datadog, New Relic) needed initially

**Alternatives Considered**:
- **Application Performance Monitoring (APM) service**: Adds cost and complexity for MVP. Can add later if needed.
- **Manual logging in each endpoint**: Error-prone, requires discipline to maintain.
- **No metrics**: Violates FR-024, FR-025 requirements.

**Implementation Notes**:
- Middleware tracks: request_count, success_count, failure_count, timeout_count, avg_generation_time
- Log format: `{"event": "personalization_request", "user_id": "...", "chapter_id": "...", "duration_ms": 1500, "status": "success"}`
- Metrics endpoint: `/metrics` returns JSON with aggregated stats
- Integrate with existing monitoring dashboard (if available)

## Phase 1: Data Model & Contracts

*(Detailed data models and API contracts will be generated in subsequent files: data-model.md, contracts/personalization-api.yaml)*

### Entity Summary

1. **PersonalizationQuota** (PostgreSQL)
   - Tracks rate limiting per user
   - Fields: user_id, request_count, first_request_timestamp, reset_at

2. **PersonalizationRequest** (In-memory/log only)
   - Represents single personalization operation
   - Fields: request_id, user_id, chapter_id, profile_snapshot, status, duration_ms, error_message

3. **PersonalizedContent** (Frontend session state only)
   - Cached personalized markdown
   - Fields: chapter_id, personalized_markdown, original_markdown, generated_at, profile_snapshot

### API Contracts Summary

**POST /api/personalize**
- Request: chapter_id, chapter_content (markdown), user_profile (softwareBackground, hardwareBackground, interestArea)
- Response: personalized_markdown, remaining_limit
- Errors: 429 (rate limit exceeded), 408 (timeout), 500 (generation failed)

**GET /api/personalization/quota**
- Response: remaining_requests, reset_at (ISO timestamp)

**DELETE /api/personalization/cache** (profile update trigger)
- Invalidates all cached personalized content for current user
- Response: 204 No Content

## Quickstart Summary

*(Detailed setup guide will be in quickstart.md)*

1. **Backend Setup**:
   - Add `OPENAI_API_KEY` to backend environment variables
   - Run migration: `005_add_personalization_quota.sql`
   - Install: `pip install openai-agents-python markdown-it-py`
   - Start backend: `uvicorn main:app --reload`

2. **Frontend Setup**:
   - Install: `npm install` (existing dependencies sufficient)
   - Configure API endpoint in `personalizationService.ts`
   - Start Docusaurus: `npm run start`

3. **Testing Personalization**:
   - Sign up and complete user profile
   - Navigate to any chapter
   - Click "Personalize for Me" button
   - View personalized content and toggle to original

## Next Steps

1. **Complete Phase 0**: Generate detailed `research.md` (this section serves as summary)
2. **Complete Phase 1**: Generate `data-model.md`, `contracts/personalization-api.yaml`, `quickstart.md`
3. **Update Agent Context**: Run update script to add OpenAI Agents SDK to tech stack
4. **Proceed to Phase 2**: Run `/sp.tasks` to generate implementation tasks
