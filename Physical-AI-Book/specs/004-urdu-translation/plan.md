# Implementation Plan: Urdu Translation for Textbook Chapters

**Branch**: `004-urdu-translation` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-urdu-translation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Add on-demand Urdu translation functionality for textbook chapters, accessible only to authenticated users. Users click a "Translate to Urdu" button at the start of each chapter, which sends the chapter markdown content to a FastAPI backend endpoint. The backend uses an OpenAI Agent (implemented with OpenAI Agents SDK) to translate English content to Urdu while preserving technical terms, code blocks, and mathematical formulas. The frontend renders the translated Urdu content without modifying the original English markdown files.

**User Input Architecture**:
- Frontend: "Translate to Urdu" button at the start of each chapter
- Backend: FastAPI endpoint to handle translation requests
- AI: OpenAI Agent implemented using the OpenAI Agents SDK for Urdu translation
- AI Strategy: Agent translates English to Urdu, preserves meaning/structure, keeps technical terms in English
- Integration: Frontend → Backend → OpenAI Agent → Backend → Frontend (rendered Urdu content)
- Access Control: Translation available only to logged-in users; backend verifies session
- Configuration: OpenAI API key via environment variables

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.3+ (frontend), React 18+ (Docusaurus)
**Primary Dependencies**:
  - Backend: FastAPI 0.104+, OpenAI Python SDK 1.3+, OpenAI Agents SDK (latest), Pydantic 2.5+
  - Frontend: React 18+, TypeScript 5.3+, Docusaurus 3.x
**Storage**:
  - Optional caching: Redis or in-memory cache (to reduce duplicate translation costs)
  - Analytics logging: Existing PostgreSQL database (track translation requests per user)
**Testing**:
  - Backend: pytest 7.4+ (unit tests for translation endpoint, integration tests for OpenAI Agent)
  - Frontend: Jest (React component tests for translation button and toggle)
**Target Platform**:
  - Backend: Linux server (existing FastAPI deployment)
  - Frontend: Modern browsers with UTF-8 Urdu font support
**Project Type**: Web application (frontend + backend)
**Performance Goals**:
  - Translation completes within 30 seconds for chapters up to 5,000 words (SC-002)
  - Backend endpoint responds within 1 second for cached translations
**Constraints**:
  - Must NOT modify original English markdown files (constitution principle XXII)
  - Translation only for authenticated users (constitution principle XXI)
  - Technical terms MUST remain in English (constitution principle XIX)
  - OpenAI API rate limits (approx 10,000 tokens per minute for GPT-4)
  - Cost constraint: minimize redundant translations via optional caching
**Scale/Scope**:
  - Approximately 50-100 chapters in textbook
  - Estimated 1,000-5,000 words per chapter
  - Support 100+ concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pedagogical Principles
- ✅ **Principle I (Structured Learning Path)**: Not applicable - translation preserves existing structure
- ✅ **Principle II (Clear Academic Communication)**: Translation maintains academic tone per FR-008
- ✅ **Principle III (Strict File Conventions)**: Original files unchanged per FR-007 (principle XXII)
- ✅ **Principle IV (Hands-On Learning)**: Code examples preserved exactly per FR-005
- ✅ **Principle V (Assessment & Validation)**: Not applicable - exercises remain in original content
- ✅ **Principle VI (Attribution & Licensing)**: Not applicable - no content creation

### RAG Chatbot Principles
- ✅ **Principle VII-XIII**: Not applicable - translation feature separate from chatbot

### Authentication Principles
- ✅ **Principle XIV (Documentation-Driven Auth)**: Reuses existing Better-Auth implementation
- ✅ **Principle XV (User Background Collection)**: Not applicable - no new user data collected
- ✅ **Principle XVI (Secure Credential Management)**: OpenAI API key in backend .env only per FR-014
- ✅ **Principle XVII (Simple & Minimal Auth)**: Leverages existing email/password auth

### Localization & Translation Principles (XVIII-XXII) - CRITICAL
- ✅ **Principle XVIII (Meaning-Preserving Translation)**: FR-004, FR-005 ensure technical accuracy
- ✅ **Principle XIX (Technical Term Handling)**: FR-004 explicitly preserves English technical terms
- ✅ **Principle XX (Simple & Readable Urdu)**: FR-008 mandates undergraduate-appropriate Urdu
- ✅ **Principle XXI (Authenticated Translation Access)**: FR-002, FR-014, SC-006 enforce auth requirement
- ✅ **Principle XXII (Original Content Preservation - NON-NEGOTIABLE)**: FR-007, SC-004 guarantee no file modification

**GATE STATUS**: ✅ **PASSED** - All applicable principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/004-urdu-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── translation-api.yaml  # OpenAPI schema for translation endpoint
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   └── routes/
│   │       └── translation.py        # New: /translate endpoint
│   ├── services/
│   │   ├── openai_agent_service.py   # New: OpenAI Agents SDK integration
│   │   └── translation_service.py    # New: Translation orchestration logic
│   ├── middleware/
│   │   └── jwt_auth.py               # Existing: Auth middleware (reuse)
│   ├── models/
│   │   └── translation.py            # New: Pydantic models for translation requests
│   └── config.py                     # Updated: Add OPENAI_API_KEY config
├── tests/
│   ├── unit/
│   │   ├── test_translation_service.py
│   │   └── test_openai_agent_service.py
│   └── integration/
│       └── test_translation_api.py
└── .env.example                      # Updated: Add OPENAI_API_KEY placeholder

textbook/
├── src/
│   ├── components/
│   │   └── TranslationButton/
│   │       ├── index.tsx             # New: Translation button component
│   │       ├── styles.module.css     # New: Button styling
│   │       └── __tests__/
│   │           └── TranslationButton.test.tsx
│   ├── hooks/
│   │   └── useTranslation.ts         # New: React hook for translation state
│   └── services/
│       └── translationApi.ts         # New: API client for /translate endpoint
└── docusaurus.config.js              # Updated: May need theme customization

history/
└── prompts/
    └── urdu-translation/
        └── [PHR files for this feature]
```

**Structure Decision**: Web application structure (Option 2). Extends existing `backend/` FastAPI server with new translation endpoint and services. Adds new React component to existing `textbook/` Docusaurus frontend. No new projects or repositories required.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations - table not needed.

---

## Phase 0: Outline & Research

**Status**: Ready to execute

**Unknowns to Research**:
1. OpenAI Agents SDK best practices for translation tasks
2. Optimal prompt engineering for preserving technical terms in English while translating explanatory text
3. Caching strategy for translated content (Redis vs in-memory vs database)
4. Markdown parsing and preservation during translation (handling code blocks, LaTeX, tables)
5. Error handling patterns for OpenAI API failures and rate limits
6. Frontend state management for language toggle (React state vs URL params)

**Research Tasks**:

| Unknown | Research Focus | Output |
|---------|---------------|--------|
| OpenAI Agents SDK for translation | Best practices for creating translation agents; authentication patterns; streaming vs batch translation | Decision on agent configuration and implementation approach |
| Technical term preservation | Prompt engineering strategies; few-shot examples; instruction clarity for bilingual translation | System prompt template for translation agent |
| Caching strategy | Cost-benefit analysis of Redis vs PostgreSQL vs in-memory; cache invalidation; TTL strategies | Caching architecture decision |
| Markdown preservation | Libraries for markdown parsing (e.g., `markdown-it`, `remark`); preservation of code fences and LaTeX | Markdown handling approach |
| Error handling | OpenAI API retry patterns; exponential backoff; fallback strategies; user-facing error messages | Error handling implementation guide |
| Frontend state management | React Context API vs component state; persisting toggle state; URL-based language selection | Frontend architecture decision |

**Deliverable**: `research.md` with decisions, rationale, and alternatives for each unknown.

---

## Phase 1: Design & Contracts

**Status**: Pending Phase 0 completion

### 1. Data Model (`data-model.md`)

**Entities** (from spec):
- Translation Request (user_id, chapter_slug, timestamp, status)
- Translated Chapter Content (chapter_slug, urdu_markdown, original_language, translation_timestamp)
- Chapter Metadata (slug, title, module, original_path)

**Additional Design Entities**:
- Translation Cache Entry (if caching implemented)
- Analytics Log Entry (user_id, chapter_slug, action, timestamp)

### 2. API Contracts (`contracts/translation-api.yaml`)

**Endpoints** (derived from functional requirements):

1. `POST /api/translate`
   - Request: `{ chapter_slug: string, markdown_content: string }`
   - Response: `{ translated_content: string, status: "success" | "error", message?: string }`
   - Authentication: Required (JWT token in header)
   - Errors: 401 (unauthenticated), 400 (invalid input), 429 (rate limit), 500 (OpenAI failure)

2. `POST /api/translate/log` (analytics tracking per FR-013)
   - Request: `{ chapter_slug: string, action: "translate_requested" | "toggle_language" }`
   - Response: `{ logged: boolean }`
   - Authentication: Required

**OpenAPI Schema**: Full YAML specification in `/contracts/translation-api.yaml`

### 3. Quickstart Guide (`quickstart.md`)

- Environment setup (OPENAI_API_KEY configuration)
- Running translation endpoint locally
- Testing translation with sample chapter
- Debugging common issues (auth errors, API rate limits)

### 4. Agent Context Update

Run `.specify/scripts/bash/update-agent-context.sh claude` to update CLAUDE.md with:
- OpenAI Agents SDK (new technology)
- Translation endpoint architecture
- Constitution principles XVIII-XXII enforcement

**Deliverables**:
- `data-model.md`
- `contracts/translation-api.yaml`
- `quickstart.md`
- Updated `.specify/memory/claude-context.md` or `CLAUDE.md`

---

## Phase 2: Tasks Generation

**Status**: Not executed by `/sp.plan` - requires separate `/sp.tasks` command

After Phase 1 completion, run `/sp.tasks` to generate:
- Dependency-ordered implementation tasks
- Test cases for each functional requirement
- Acceptance criteria per user story

---

## Post-Design Constitution Re-Check

**Execute after Phase 1 artifacts are generated**

Verify:
- [✅] API contracts enforce authentication (principle XXI)
  - OpenAPI spec requires `bearerAuth` on all `/api/translate` endpoints
  - 401 responses defined for unauthenticated access
- [✅] Data model does not include original file modification (principle XXII)
  - `TranslationCache` stores translated content separately from source files
  - `ChapterMetadata` is read-only (virtual entity from filesystem)
  - No database schema touches original markdown files
- [✅] Translation service design preserves technical terms (principle XIX)
  - Placeholder substitution pattern extracts technical terms before translation
  - OpenAI Agent system prompt explicitly instructs: "NEVER translate __TERM_* placeholders"
  - Markdown parsing (markdown-it-py) preserves code blocks and LaTeX exactly
- [✅] Error handling maintains simple user experience (principle XX)
  - User-friendly error messages defined in OpenAPI spec (no technical jargon)
  - Examples: "Translation service is busy" vs "RateLimitError 429"
  - Frontend displays clear messages: "Translation failed. Please try again."
- [✅] OpenAI API key configuration follows secure practices (principle XVI)
  - API key stored in backend .env only (never frontend)
  - Environment variable pattern: `OPENAI_API_KEY` (not hardcoded)
  - Quickstart guide emphasizes .gitignore for .env files

**Final Gate**: ✅ **PASSED** - All constitution principles satisfied in design artifacts

**Ready for `/sp.tasks` execution**
