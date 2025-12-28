---
id: 003
title: Urdu Translation Implementation Plan
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: urdu-translation
branch: 004-urdu-translation
user: sohaib
command: /sp.plan
labels: ["translation", "urdu", "architecture", "planning", "openai-agents", "caching"]
links:
  spec: specs/004-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-urdu-translation/plan.md
  - specs/004-urdu-translation/research.md
  - specs/004-urdu-translation/data-model.md
  - specs/004-urdu-translation/quickstart.md
  - specs/004-urdu-translation/contracts/translation-api.yaml
  - .specify/memory/claude-context.md (updated with new technologies)
tests:
  - Constitution Check: All principles XVIII-XXII validated
  - Post-design Constitution Re-Check: All verification items passed
---

## Prompt

```
/sp.plan
Title: Urdu Translation — Plan

Architecture:
  - Frontend: "Translate to Urdu" button at the start of each chapter.
  - Backend: FastAPI endpoint to handle translation requests.
  - AI: OpenAI Agent implemented using the OpenAI Agents SDK for Urdu translation.

AI Strategy:
  - Use OpenAI Agents SDK to create a translation agent.
  - Agent translates English chapter content into Urdu.
  - Preserve meaning and structure.
  - Keep technical terms in English where appropriate.

Integration:
  - Frontend sends chapter content to backend on button click.
  - Backend calls the OpenAI Agent via Agents SDK.
  - Backend returns translated Urdu text.
  - Frontend renders translated content without modifying the original English text.

Access Control:
  - Translation available only to logged-in users.
  - Backend verifies user session before processing translation.

Configuration:
  - OpenAI API key provided via environment variables.
```

## Response snapshot

Successfully created comprehensive implementation plan for Urdu translation feature with complete Phase 0 (Research) and Phase 1 (Design) artifacts.

### Phase 0: Research Completed

Executed 3 parallel research agents investigating:
1. **OpenAI Agents SDK** - Selected official SDK over deprecated Assistants API; GPT-4o chosen over GPT-4o-mini for quality (~$0.08-$0.10/chapter); placeholder substitution pattern for code/LaTeX preservation; streaming API for 1,000-5,000 word chapters
2. **Caching Strategy** - PostgreSQL (existing Neon) selected over Redis; 99% cost savings ($0.08 vs $347 for 100 users × 43 chapters); version-based invalidation via content_hash; 90-day TTL; adequate 50-100ms latency vs Redis 1ms
3. **Markdown Preservation** - AST-based extraction with `markdown-it-py` (CommonMark compliant); placeholder tokenization strategy; preserves code blocks, LaTeX formulas, Docusaurus JSX components

**Research Deliverable**: `research.md` (comprehensive 6-section document with sources)

### Phase 1: Design Artifacts Created

1. **data-model.md**:
   - `TranslationCache` table (chapter_slug, language, content_hash, translated_content, TTL)
   - `TranslationLog` table (user_id, chapter_slug, action, timestamp)
   - Cache key structure: `{chapter_slug}:{language}:{content_hash}`
   - Storage estimates: ~2 MB for 50 chapters, 7 MB total Year 1

2. **contracts/translation-api.yaml**:
   - OpenAPI 3.0.3 specification
   - `POST /api/translate` (main translation endpoint with authentication)
   - `POST /api/translate/log` (analytics tracking)
   - `DELETE /api/translate/cache/{chapter_slug}` (admin cache invalidation)
   - Comprehensive error responses (400, 401, 429, 500)
   - Request/response schemas with examples

3. **quickstart.md**:
   - 7-step local setup guide (env config, database migration, service startup)
   - Testing instructions (translation, caching, authentication)
   - API testing with curl examples
   - Troubleshooting section (timeout, cache, font rendering, costs)
   - Development workflow and debugging tips

4. **Agent context update (CLAUDE.md)**:
   - Added OpenAI Agents SDK, markdown-it-py, tenacity, tiktoken to Active Technologies
   - Added Recent Changes entry for 004-urdu-translation

### Technical Context Established

- **Language**: Python 3.11+ (backend), TypeScript 5.3+ (frontend)
- **Primary Dependencies**: OpenAI Agents SDK, FastAPI, markdown-it-py, React Context API
- **Storage**: PostgreSQL (Neon) for caching + analytics
- **Performance**: 30-second translation target for 5,000-word chapters
- **Constraints**: No original file modification (principle XXII), auth-only access (principle XXI)

### Constitution Check Results

**Initial Check (Pre-Research)**: ✅ PASSED - All applicable principles satisfied

**Post-Design Re-Check**: ✅ PASSED
- ✅ Principle XVIII: Meaning-preserving via AST parsing + OpenAI Agent prompts
- ✅ Principle XIX: Technical term preservation via placeholder substitution
- ✅ Principle XX: Simple Urdu via system prompt constraints
- ✅ Principle XXI: Auth enforced in OpenAPI spec (bearerAuth required)
- ✅ Principle XXII: Original files untouched (TranslationCache separate storage)

### Key Architectural Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| AI Translation | OpenAI Agents SDK + GPT-4o | Future-proof (Assistants API deprecated 2026); superior quality for technical content |
| Caching | PostgreSQL (Neon) | 99% cost savings; $0 infrastructure; adequate 50ms latency |
| Markdown Handling | AST-based (markdown-it-py) | Robust edge case handling; CommonMark compliant |
| Frontend State | React Context API | Simple session-based state; no persistence complexity |
| Font | Noto Nastaliq Urdu (Google Fonts) | Free, RTL-native, good Nastaliq rendering |

### Complexity Tracking

**No violations** - All constitution principles satisfied without complexity trade-offs.

## Outcome

- ✅ Impact: Complete planning artifacts ready for implementation; zero unresolved unknowns or clarifications
- 🧪 Tests: Constitution Check passed (initial + post-design); all 5 verification items validated
- 📁 Files: 5 design documents created (plan.md, research.md, data-model.md, quickstart.md, translation-api.yaml)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks; potential `/sp.adr` for OpenAI Agents SDK adoption decision
- 🧠 Reflection: Research phase eliminated all technical unknowns through parallel agent execution. PostgreSQL caching decision ($0 cost vs $15/month Redis) demonstrates quantitative cost-benefit analysis. AST-based markdown preservation ensures robustness vs fragile regex approaches. Constitution principles XVIII-XXII provided clear guardrails that prevented over-engineering (e.g., no multi-language support, no persistent user preferences, no offline mode).

## Evaluation notes (flywheel)

- Failure modes observed: None. Setup script path resolution issue (expected `/home/sohaib/hackathon/ai-book/Physical-AI-Book/.specify/` but script referenced parent directory) - manually corrected by reading template directly.
- Graders run and results (PASS/FAIL): Constitution Check PASSED (initial), Constitution Re-Check PASSED (post-design)
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided architecture guidance (OpenAI Agents SDK specified upfront)
- Next experiment (smallest change to try): Test whether providing explicit cost constraints in initial prompt ($X budget, Y users) produces more quantitative research analysis (current approach derived cost estimates during research phase)
