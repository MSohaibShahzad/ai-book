# Tasks: Urdu Translation for Textbook Chapters

**Input**: Design documents from `/specs/004-urdu-translation/`
**Prerequisites**: plan.md, spec.md, data-model.md, research.md, contracts/translation-api.yaml, quickstart.md

**Feature Branch**: `004-urdu-translation`
**Generated**: 2025-12-20

**Tests**: Tests are NOT explicitly requested in the specification. This implementation follows a practical validation approach using quickstart.md test scenarios.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **Checkbox**: Always starts with `- [ ]`
- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency setup for translation feature

- [ ] T001 Install backend dependencies: openai-agents, markdown-it-py, mdit-py-plugins, tenacity, tiktoken in backend/requirements.txt
- [ ] T002 Update backend/.env.example with OPENAI_API_KEY, TRANSLATION_MODEL, MAX_TRANSLATION_TOKENS, RATE_LIMIT_PER_MINUTE
- [ ] T003 [P] Install Noto Nastaliq Urdu font configuration in textbook/docusaurus.config.js stylesheets
- [ ] T004 [P] Create backend/src/models/translation.py with Pydantic models (TranslationCache, TranslationLog, TranslationAction enum)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Create database migration script for translation_cache table in backend/alembic/versions/
- [ ] T006 Create database migration script for translation_log table in backend/alembic/versions/
- [ ] T007 Apply database migrations with alembic upgrade head
- [ ] T008 [P] Implement OpenAI Agents SDK service in backend/src/services/openai_agent_service.py with placeholder extraction and GPT-4o agent
- [ ] T009 [P] Implement markdown parser service in backend/src/services/markdown_service.py using markdown-it-py for AST-based extraction
- [ ] T010 [P] Implement translation cache service in backend/src/services/translation_cache_service.py with PostgreSQL lookup and TTL management
- [ ] T011 Create translation orchestration service in backend/src/services/translation_service.py integrating cache, parser, and OpenAI agent
- [ ] T012 Update backend/src/config.py to load OPENAI_API_KEY and translation configuration from environment
- [ ] T013 Create translation API route in backend/src/api/routes/translation.py with POST /api/translate endpoint
- [ ] T014 Register translation routes in backend/src/main.py FastAPI app

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Authenticated User Translates Chapter (Priority: P1) 🎯 MVP

**Goal**: Enable logged-in users to translate any chapter to Urdu by clicking a button, preserving technical terms and code blocks

**Independent Test**: Log in, navigate to any chapter, click "Translate to Urdu", verify Urdu content appears with preserved code blocks and technical terms (per quickstart.md section 4.2)

### Implementation for User Story 1

- [ ] T015 [P] [US1] Implement POST /api/translate endpoint handler in backend/src/api/routes/translation.py with authentication check
- [ ] T016 [P] [US1] Add request validation for chapter_slug format in backend/src/api/routes/translation.py
- [ ] T017 [US1] Implement translation workflow: check cache → read markdown → translate → cache → return in backend/src/services/translation_service.py
- [ ] T018 [US1] Add error handling for chapter not found, OpenAI API errors, and rate limits in backend/src/api/routes/translation.py
- [ ] T019 [US1] Implement retry logic with exponential backoff using tenacity in backend/src/services/openai_agent_service.py
- [ ] T020 [P] [US1] Create TranslationButton React component in textbook/src/components/TranslationButton/index.tsx
- [ ] T021 [P] [US1] Add TranslationButton styles with loading states in textbook/src/components/TranslationButton/styles.module.css
- [ ] T022 [US1] Implement useTranslation custom hook in textbook/src/hooks/useTranslation.ts for API calls
- [ ] T023 [US1] Integrate TranslationButton at the start of each chapter page in Docusaurus theme
- [ ] T024 [US1] Add analytics logging for translate_requested action in backend/src/api/routes/translation.py

**Checkpoint**: User Story 1 complete - users can translate chapters to Urdu with authentication

---

## Phase 4: User Story 4 - Unauthenticated User Blocked (Priority: P1)

**Goal**: Ensure translation feature is only accessible to logged-in users with clear feedback for unauthenticated visitors

**Independent Test**: Sign out, navigate to any chapter, verify translate button is disabled or shows "Please log in" tooltip (per quickstart.md section 4.4)

**Note**: This is grouped with P1 as authentication enforcement is non-negotiable per constitution principle XXI

### Implementation for User Story 4

- [ ] T025 [US4] Add authentication middleware check in backend/src/api/routes/translation.py using existing jwt_auth.py
- [ ] T026 [US4] Return 401 error with user-friendly message "Authentication required. Please log in to use Urdu translation."
- [ ] T027 [US4] Update TranslationButton component to check authentication state in textbook/src/components/TranslationButton/index.tsx
- [ ] T028 [US4] Disable button for logged-out users with tooltip "Please log in to use Urdu translation" in textbook/src/components/TranslationButton/index.tsx
- [ ] T029 [US4] Add analytics logging for failed authentication attempts in backend/src/api/routes/translation.py

**Checkpoint**: Authentication enforcement complete - only logged-in users can translate

---

## Phase 5: User Story 2 - Toggle Between English and Urdu (Priority: P2)

**Goal**: Allow users to switch between original English and Urdu translation without page reload

**Independent Test**: Translate a chapter to Urdu, click "Show Original English", verify instant switch back to English, then toggle to Urdu again (per quickstart.md section 4.3)

### Implementation for User Story 2

- [ ] T030 [P] [US2] Create TranslationContext React Context in textbook/src/contexts/TranslationContext.tsx with language state
- [ ] T031 [P] [US2] Implement toggleLanguage and setTranslation methods in TranslationContext
- [ ] T032 [US2] Add language toggle button in TranslationButton component (Show Original English / Show Urdu Translation)
- [ ] T033 [US2] Implement content rendering logic to switch between original and translated content in textbook/src/theme/DocItem/Content/index.tsx
- [ ] T034 [US2] Add RTL (dir="rtl") styling for Urdu content in textbook/src/css/custom.css
- [ ] T035 [US2] Preserve LTR for code blocks and formulas within RTL context in textbook/src/css/custom.css
- [ ] T036 [US2] Store translated content in React state to avoid re-fetching on toggle
- [ ] T037 [US2] Add analytics logging for toggle_to_english and toggle_to_urdu actions via POST /api/translate/log endpoint

**Checkpoint**: Language toggle working - users can switch between English and Urdu seamlessly

---

## Phase 6: User Story 3 - Translation Status and Loading Feedback (Priority: P3)

**Goal**: Provide clear visual feedback during translation with loading indicators and error messages

**Independent Test**: Simulate slow network, click translate, verify loading indicator shows "Translating to Urdu...", verify error messages appear for failures (per quickstart.md section 4.2)

### Implementation for User Story 3

- [ ] T038 [US3] Add loading state to TranslationButton component in textbook/src/components/TranslationButton/index.tsx
- [ ] T039 [US3] Display "Translating to Urdu..." text during translation request
- [ ] T040 [US3] Disable translate button while translation is in progress
- [ ] T041 [US3] Add error state for failed translations with user-friendly messages in textbook/src/components/TranslationButton/index.tsx
- [ ] T042 [US3] Implement error message mapping in textbook/src/services/translationApi.ts (rate_limit, timeout, api_error, network)
- [ ] T043 [US3] Add smooth transition animation when translated content appears in textbook/src/components/TranslationButton/styles.module.css
- [ ] T044 [US3] Add analytics logging for translation_failed action in backend/src/api/routes/translation.py

**Checkpoint**: User feedback complete - loading states and error messages working

---

## Phase 7: Analytics Tracking

**Purpose**: Implement analytics logging endpoint for translation usage tracking

- [ ] T045 [P] Create POST /api/translate/log endpoint in backend/src/api/routes/translation.py
- [ ] T046 [P] Implement log_translation_action function in backend/src/services/analytics_service.py
- [ ] T047 Insert translation logs into translation_log table with user_id, chapter_slug, action, timestamp
- [ ] T048 Add authentication check to analytics endpoint
- [ ] T049 Create analytics query utilities for usage reporting in backend/src/services/analytics_service.py

**Checkpoint**: Analytics tracking operational

---

## Phase 8: Cache Management (Admin Features)

**Purpose**: Admin tools for cache invalidation and monitoring

- [ ] T050 [P] Create DELETE /api/translate/cache/{chapter_slug} endpoint in backend/src/api/routes/translation.py
- [ ] T051 [P] Add admin authorization check for cache invalidation endpoint
- [ ] T052 Implement cache invalidation logic in backend/src/services/translation_cache_service.py
- [ ] T053 Add cleanup job for expired cache entries (WHERE expires_at < CURRENT_TIMESTAMP)
- [ ] T054 Add cache statistics endpoint GET /api/translate/stats (cache hit rate, total translations, cost estimates)

**Checkpoint**: Cache management tools ready

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T055 [P] Add cost tracking and logging using tiktoken in backend/src/services/openai_agent_service.py
- [ ] T056 [P] Implement chunking strategy for large chapters (> 100K tokens) in backend/src/services/translation_service.py
- [ ] T057 Validate all placeholders are restored after translation in backend/src/services/markdown_service.py
- [ ] T058 Add YAML frontmatter translation (title, description, sidebar_label) in backend/src/services/markdown_service.py
- [ ] T059 [P] Run quickstart.md validation scenarios (sections 4.1-4.4, step 5, step 6)
- [ ] T060 [P] Add Urdu font CSS with proper line-height (1.8) for Noto Nastaliq in textbook/src/css/custom.css
- [ ] T061 Update CLAUDE.md with OpenAI Agents SDK technology and constitution principle enforcement
- [ ] T062 Code cleanup: ensure no hardcoded API keys, verify .env in .gitignore
- [ ] T063 Security review: verify JWT validation, check for SQL injection risks, validate input sanitization
- [ ] T064 Performance optimization: add database indexes (idx_translation_lookup, idx_translation_expiry)

**Checkpoint**: Feature polished and production-ready

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - MVP baseline
- **User Story 4 (Phase 4)**: Depends on Foundational (Phase 2) - Can run parallel with US1 but critical for security
- **User Story 2 (Phase 5)**: Depends on User Story 1 completion - Enhances translation with toggle
- **User Story 3 (Phase 6)**: Depends on User Story 1 completion - Enhances UX with loading feedback
- **Analytics (Phase 7)**: Depends on Foundational (Phase 2) - Can run parallel with user stories
- **Cache Management (Phase 8)**: Depends on Foundational (Phase 2) - Can run after US1
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Core translation functionality
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - Authentication enforcement (critical)
- **User Story 2 (P2)**: Depends on US1 - Language toggle requires translation to exist
- **User Story 3 (P3)**: Depends on US1 - Loading feedback requires translation flow to exist

### Within Each User Story

- **Backend before Frontend**: API endpoints must exist before React components can call them
- **Services before Routes**: Service layer logic before API route handlers
- **Models before Services**: Pydantic models before services that use them
- **Database before Services**: Migration scripts before cache/log services
- **Core implementation before integration**: Translation service before React integration

### Parallel Opportunities

**Setup Phase (Phase 1)**: All tasks can run in parallel
- T001 (backend dependencies) || T003 (font config) || T004 (Pydantic models)

**Foundational Phase (Phase 2)**: Database tasks sequential, services parallel
- T005-T007 sequential (migrations), then T008-T010 parallel (services)

**User Story 1 + User Story 4**: Can develop in parallel after foundational
- Backend US1 (T015-T019) || Backend US4 (T025-T026)
- Frontend US1 (T020-T021) || Frontend US4 (T027-T028)

**User Story 2 + User Story 3**: Can develop in parallel after US1
- US2 tasks (T030-T037) || US3 tasks (T038-T044)

**Analytics + Cache Management**: Can develop in parallel
- Phase 7 (T045-T049) || Phase 8 (T050-T054)

---

## Parallel Example: Foundational Phase

```bash
# After migrations complete (T005-T007), launch services in parallel:
Task T008: "Implement OpenAI Agents SDK service in backend/src/services/openai_agent_service.py"
Task T009: "Implement markdown parser service in backend/src/services/markdown_service.py"
Task T010: "Implement translation cache service in backend/src/services/translation_cache_service.py"
```

## Parallel Example: User Story 1

```bash
# Backend and frontend can develop in parallel:
Task T015: "Implement POST /api/translate endpoint handler"
Task T016: "Add request validation for chapter_slug"
# Meanwhile:
Task T020: "Create TranslationButton React component"
Task T021: "Add TranslationButton styles"
```

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 4)

This delivers the core value: authenticated users can translate chapters to Urdu.

1. **Complete Phase 1**: Setup (T001-T004)
2. **Complete Phase 2**: Foundational (T005-T014) - CRITICAL blocking phase
3. **Complete Phase 3**: User Story 1 (T015-T024) - Core translation
4. **Complete Phase 4**: User Story 4 (T025-T029) - Authentication enforcement
5. **STOP and VALIDATE**: Test per quickstart.md sections 4.2 and 4.4
6. **Deploy/Demo**: MVP is ready

At this point you have:
- ✅ Authenticated users can translate chapters
- ✅ Urdu content displayed with preserved code/formulas
- ✅ Unauthenticated users blocked with clear feedback
- ✅ Caching working to minimize costs
- ✅ Analytics logging operational

### Incremental Delivery

1. **Phase 1 + 2**: Foundation ready (T001-T014)
2. **+ Phase 3 + 4**: MVP ready - authenticated translation working (T015-T029)
3. **+ Phase 5**: Enhanced with language toggle (T030-T037)
4. **+ Phase 6**: Polished UX with loading feedback (T038-T044)
5. **+ Phase 7 + 8**: Analytics and admin tools (T045-T054)
6. **+ Phase 9**: Production-ready polish (T055-T064)

Each phase adds value without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1. **All team**: Complete Setup + Foundational together (T001-T014)
2. **Once Foundational is done**:
   - Developer A: User Story 1 backend (T015-T019)
   - Developer B: User Story 1 frontend (T020-T023)
   - Developer C: User Story 4 authentication (T025-T029)
3. **After US1 complete**:
   - Developer A: User Story 2 toggle (T030-T037)
   - Developer B: User Story 3 loading (T038-T044)
   - Developer C: Analytics (T045-T049)
4. **Final sprint**: Polish together (T055-T064)

---

## Task Summary

- **Total Tasks**: 64
- **Setup Phase**: 4 tasks
- **Foundational Phase**: 10 tasks (CRITICAL blocking)
- **User Story 1 (P1)**: 10 tasks (MVP core)
- **User Story 4 (P1)**: 5 tasks (MVP security)
- **User Story 2 (P2)**: 8 tasks (Enhancement)
- **User Story 3 (P3)**: 7 tasks (Enhancement)
- **Analytics**: 5 tasks
- **Cache Management**: 5 tasks
- **Polish**: 10 tasks

**MVP Scope** (Phases 1-4): 29 tasks
**Full Feature** (All phases): 64 tasks

**Parallel Opportunities**: 18 tasks marked [P] can run in parallel with others

---

## Validation Checklist (Per Quickstart.md)

After completing MVP (Phases 1-4), verify:

- [ ] Section 4.1: Can sign in with existing Better-Auth credentials
- [ ] Section 4.2: Can translate chapter, Urdu content appears, code blocks preserved
- [ ] Section 4.3: Can toggle between English and Urdu (Phase 5 required)
- [ ] Section 4.4: Translate button disabled when logged out
- [ ] Section 5.1: Cached translations in translation_cache table
- [ ] Section 5.2: Second translation is instant (cache hit)
- [ ] Section 5.3: Analytics logs in translation_log table
- [ ] Section 6.2: API responds with translated content
- [ ] Section 6.3: 401 error for unauthenticated requests

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] label**: Maps task to specific user story for traceability
- **File paths**: All paths are absolute and match project structure from plan.md
- **Tests**: Not included as not explicitly requested in specification
- **Validation**: Uses quickstart.md test scenarios for practical validation
- **Constitution compliance**: All tasks respect principles XVIII-XXII (verified in plan.md)
- **No file modifications**: Original markdown files never touched (constitution principle XXII)
- **Authentication enforced**: All translation endpoints require JWT (constitution principle XXI)
- **Cost optimization**: Caching implemented in foundational phase to minimize OpenAI API costs

---

**Tasks Status**: ✅ Complete - Ready for `/sp.implement` execution or manual task-by-task implementation
