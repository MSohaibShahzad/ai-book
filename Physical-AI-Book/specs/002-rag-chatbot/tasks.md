---
description: "Task list for RAG Chatbot implementation"
---

# Tasks: RAG Chatbot for Physical-AI Textbook

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL per specification - not explicitly requested in spec.md, so excluded from task list.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths follow plan.md structure (Option 2: Web application)

## OpenAI Agents SDK Migration Notes

**Architecture Change**: This task list reflects the **OpenAI Agents SDK** architecture, not the original direct API approach.

### Key Differences from Original Plan:

1. **T024**: Creates `agent_service.py` instead of `llm_service.py`
   - Uses `@function_tool` decorator for retrieval tool
   - Agent configured with `Runner.run()` and `Runner.run_streamed()`
   - Tool-based architecture replaces imperative pipeline

2. **T025**: Simplified `rag_pipeline.py`
   - Delegates to agent service instead of managing full pipeline
   - 3 steps (prepare → agent → return) vs 9 steps (embed → search → fetch → format → prompt → generate → parse → extract)

3. **T027-T029**: Safety and grounding enforced via:
   - Agent instructions (system prompt)
   - Tool return values ("NO_RELEVANT_CONTENT_FOUND")
   - `_check_safety()` method in agent service

4. **T039-T041**: Highlight prioritization via:
   - Agent input modification (prepend highlighted context)
   - Tool still retrieves additional context
   - Agent instructions reference highlighted text

5. **T049-T050**: Conversation history via:
   - `Runner.run(agent, input=query, messages=history)`
   - Agent SDK handles context management internally
   - Tool inherits conversation context from agent

### Benefits:
- **Cleaner separation**: Tools encapsulate RAG logic
- **Extensibility**: Easy to add more tools (calculator, code execution)
- **Streaming support**: Built-in via `Runner.run_streamed()`
- **Autonomous behavior**: Agent decides when to call tools

### Files Affected:
- **NEW**: `backend/src/services/agent_service.py`
- **MODIFIED**: `backend/src/services/rag_pipeline.py` (simplified)
- **DEPRECATED**: `backend/src/services/llm_service.py` (kept for reference)
- **MODIFIED**: `backend/requirements.txt` (added `openai-agents>=1.0.0`)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend directory structure per plan.md (backend/src/, backend/scripts/, backend/tests/)
- [ ] T002 Create frontend directory structure per plan.md (frontend/src/components/, frontend/src/services/, frontend/src/hooks/)
- [ ] T003 [P] Initialize backend Python project with requirements.txt (FastAPI 0.104+, Qdrant Client 1.7+, psycopg2 2.9+, OpenAI 1.33+, openai-agents 1.0+, Pydantic 2.5+, pytest 7.4+)
- [ ] T004 [P] Initialize frontend Node project with package.json (React 18+, TypeScript 5.3+, Webpack 5, Jest)
- [ ] T005 [P] Create backend/.env.example with required environment variables (QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, OPENAI_API_KEY)
- [ ] T006 [P] Create frontend/.env.example with REACT_APP_API_URL
- [ ] T007 [P] Create backend/Dockerfile for containerized deployment
- [ ] T008 [P] Create backend/.gitignore (venv/, .env, __pycache__/, *.pyc)
- [ ] T009 [P] Create frontend/.gitignore (node_modules/, build/, .env)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T010 Create backend/src/config.py with environment variable loading and validation using Pydantic settings
- [ ] T011 Create backend/src/database.py with Postgres connection pool using psycopg2
- [ ] T012 [P] Create backend/src/api/models/request.py with ChatRequest Pydantic model (message, highlighted_text, session_id, conversation_history fields per data-model.md)
- [ ] T013 [P] Create backend/src/api/models/response.py with ChatResponse and SourceReference Pydantic models (response, sources, session_id, retrieval_count, processing_time_ms fields per data-model.md)
- [ ] T014 Create backend/src/services/embeddings.py with OpenAI embedding generation service (text-embedding-3-small model, batch processing, error handling)
- [ ] T015 Create backend/src/services/qdrant_service.py with Qdrant client initialization and collection setup (textbook_chunks collection, 1536-dim vectors, cosine similarity)
- [ ] T016 Create backend/src/services/postgres_service.py with TextbookChunk metadata queries (retrieve by qdrant_vector_id, filter by module/chapter)
- [ ] T017 Create backend/scripts/init_database.py with SQL schema creation for textbook_chunks table (id, qdrant_vector_id, chunk_text, module_name, chapter_name, section_heading, file_path, slug, chunk_index, token_count, created_at, indexes per data-model.md)
- [ ] T018 Create backend/src/ingestion/textbook_parser.py with markdown frontmatter parsing (YAML extraction for module, chapter, slug)
- [ ] T019 Create backend/src/ingestion/chunker.py with heading-based chunking strategy (300-500 tokens, 50-token overlap, preserve markdown structure per research.md)
- [ ] T020 Create backend/src/ingestion/indexer.py with batch embedding and storage (embed chunks, insert into Qdrant with UUID, insert metadata into Postgres)
- [ ] T021 Create backend/scripts/ingest_textbook.py CLI script with argparse for textbook path and dry-run option (calls textbook_parser, chunker, indexer)
- [ ] T022 Create backend/src/main.py with FastAPI app initialization, CORS middleware, and API router registration
- [ ] T023 Create backend/src/api/routes/health.py with GET /health endpoint (check Qdrant, Postgres, OpenAI connectivity, return HealthResponse per openapi.yaml)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Global Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions and receive answers with 2-5 source references from textbook

**Independent Test**: Student opens chat widget, asks "What is inverse kinematics?", receives answer with module/chapter citations

### Implementation for User Story 1

- [ ] T024 [P] [US1] Create backend/src/services/agent_service.py with OpenAI Agents SDK (RAGAgentService class, @function_tool decorated retrieve_textbook_context, Agent configured with gpt-4o-mini model, system prompt with constitution principles VII-X, both standard and streaming modes via Runner)
- [ ] T025 [P] [US1] Create backend/src/services/rag_pipeline.py with simplified agent-based orchestration (prepare query, call agent_service.generate_response, handle response, extract sources from agent output)
- [ ] T026 [US1] Create backend/src/api/routes/chat.py with POST /chat endpoint (validate ChatRequest, handle standard query mode without highlighted_text, call rag_pipeline, return ChatResponse with sources per openapi.yaml)
- [ ] T027 [US1] Add safety prompt enforcement in agent_service.py (refuse harmful queries in _check_safety method, redirect to Ethics & Safety per constitution principle IX, test with "weaponize robot arm" query)
- [ ] T028 [US1] Add "not covered" fallback in retrieve_textbook_context tool (if similarity scores <0.35, return "NO_RELEVANT_CONTENT_FOUND" string per spec.md edge case)
- [ ] T029 [US1] Add source attribution formatting in agent system instructions (ensure agent response includes "Source: Module X, Chapter Y" format per constitution principle VIII via agent instructions)
- [ ] T030 [P] [US1] Create frontend/src/services/chatApi.ts with POST /chat API client (fetch wrapper, error handling, TypeScript types for ChatRequest and ChatResponse)
- [ ] T031 [P] [US1] Create frontend/src/hooks/useChat.ts with conversation state management (useState for sessionId, messages array, isLoading, error, sendMessage function that calls chatApi)
- [ ] T032 [P] [US1] Create frontend/src/components/MessageList.tsx with message display (map over messages, render user/assistant bubbles, Markdown rendering for assistant responses using markdown-to-jsx)
- [ ] T033 [P] [US1] Create frontend/src/components/MessageInput.tsx with text input and send button (controlled input, onSubmit handler, disabled state while loading)
- [ ] T034 [P] [US1] Create frontend/src/components/SourceReference.tsx with module/chapter citation display (clickable links to textbook pages using slug, preview text)
- [ ] T035 [US1] Create frontend/src/components/ChatWidget.tsx with floating button and chat interface (expand/collapse state, integrate MessageList, MessageInput, SourceReference components, session-only state per constitution principle XIII)
- [ ] T036 [US1] Create frontend/src/index.ts widget entry point (initialize ChatWidget on DOMContentLoaded, inject into document body)
- [ ] T037 [US1] Create frontend/webpack.config.js with bundle configuration (entry: src/index.ts, output: build/widget.js, CSS loaders, tree shaking, target ~65KB gzipped per research.md)
- [ ] T038 [US1] Build frontend widget bundle and verify size (npm run build, check gzipped size <100KB)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently - student can ask questions and receive grounded answers with sources

---

## Phase 4: User Story 2 - Highlighted Text Explanation (Priority: P2)

**Goal**: Enable students to highlight text and get targeted explanations with context from related sections

**Independent Test**: Student highlights "Denavit-Hartenberg parameters", asks "explain this", receives explanation referencing highlighted content plus additional context

### Implementation for User Story 2

- [ ] T039 [P] [US2] Add highlight prioritization logic in agent_service.py (if highlighted_text provided, pass to agent as context, truncate to 2000 chars, tool retrieves 2-3 additional chunks per research.md)
- [ ] T040 [US2] Update POST /chat endpoint in chat.py to handle highlighted_text parameter (extract from ChatRequest, pass to agent_service.generate_response with highlighted_text)
- [ ] T041 [US2] Update agent instructions in agent_service.py to explicitly reference highlighted text when provided ("The student highlighted: {highlighted_text}" per constitution principle XI, prepend to agent input)
- [ ] T042 [P] [US2] Create frontend/src/hooks/useHighlight.ts with text selection detection (window.getSelection() API, mouseup event listener, state for selected text, cleanup on unmount)
- [ ] T043 [US2] Update ChatWidget.tsx to integrate useHighlight hook (pass highlighted text to sendMessage, display "Explain highlighted text" prompt when selection exists)
- [ ] T044 [US2] Update MessageInput.tsx to show highlighted text preview when available (display snippet with "✓ Text selected" indicator, allow clearing selection)
- [ ] T045 [US2] Update chatApi.ts to include highlighted_text in request payload when provided
- [ ] T046 [US2] Test highlight-based queries with code snippets and mathematical formulas (verify LaTeX rendering, code block formatting per constitution principle XII, confirm agent handles highlighted context correctly)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - students can ask free-text questions OR highlight-based queries

---

## Phase 5: User Story 3 - Multi-Turn Conversation (Priority: P3)

**Goal**: Enable students to ask follow-up questions within continuous conversation flow with context maintenance

**Independent Test**: Student asks "What is forward kinematics?", receives answer, follows up with "Can you show me an example?", system retrieves examples maintaining context

### Implementation for User Story 3

- [ ] T047 [US3] Update useChat.ts hook to include conversation_history in requests (send last 4 messages - 2 turns - to backend per data-model.md max 10 items limit)
- [ ] T048 [US3] Update POST /chat endpoint in chat.py to accept and process conversation_history from request (extract from ChatRequest, validate max 10 items)
- [ ] T049 [US3] Update agent_service.py to incorporate conversation history via Runner (pass conversation_history to Runner.run() as messages parameter for agent context, enables reference resolution like "it", "this", "that")
- [ ] T050 [US3] Add context-aware retrieval in retrieve_textbook_context tool (if tool receives vague query like "give me an example", agent context already includes conversation history, tool can extract topic from agent's understanding and prioritize relevant modules/chapters)
- [ ] T051 [US3] Add session boundary handling in ChatWidget.tsx (clear messages on page refresh per constitution principle XIII, generate new sessionId on mount using crypto.randomUUID())
- [ ] T052 [US3] Verify conversation history clears on page refresh (test in browser DevTools → Application → No localStorage or sessionStorage entries)
- [ ] T053 [US3] Test multi-turn conversations with pronoun resolution (ask "What is ROS 2?", then "give me an example of it", verify "it" resolves to ROS 2)

**Checkpoint**: All user stories should now be independently functional - students can ask questions, highlight text, and engage in multi-turn conversations

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final integration

- [ ] T054 [P] Add error handling in chatApi.ts (catch network errors, parse API error responses, display user-friendly messages per spec.md edge cases)
- [ ] T055 [P] Add loading states in ChatWidget.tsx (spinner while waiting for response, disable input during processing, show "Typing..." indicator)
- [ ] T056 [P] Add rate limiting in backend/src/main.py (10 requests/minute per session_id using slowapi or custom middleware per quickstart.md)
- [ ] T057 [P] Add request logging in backend/src/main.py (log query, session_id, retrieval_count, processing_time_ms for monitoring)
- [ ] T058 [P] Add CSS styling for ChatWidget.tsx (floating button bottom-right, smooth expand/collapse animation, CSS Modules for isolation per research.md)
- [ ] T059 [P] Verify widget works across all Docusaurus pages (test on homepage, module pages, chapter pages)
- [ ] T060 [P] Verify no CSS conflicts with Docusaurus styles (inspect elements, check z-index, verify isolation)
- [ ] T061 Create backend/README.md with setup instructions (reference quickstart.md for venv, dependencies, .env config, database init, ingestion)
- [ ] T062 [P] Create frontend/README.md with build instructions (reference quickstart.md for npm install, build, Docusaurus integration)
- [ ] T063 Run ingestion script on full textbook (python scripts/ingest_textbook.py --textbook-path ../textbook/docs, verify ~500-1000 chunks indexed)
- [ ] T064 Test end-to-end flow (ask question → see answer with sources → click source link → navigate to textbook page)
- [ ] T065 Update textbook/docusaurus.config.js to inject widget script tag (add to scripts array: {src: '/chat-widget.js', async: true})
- [ ] T066 Copy widget bundle to textbook/static/chat-widget.js for Docusaurus serving

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational (Phase 2) - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational (Phase 2) - No dependencies on other stories (independent)
  - User Story 3 (P3): Can start after Foundational (Phase 2) - No dependencies on other stories (independent)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories ✅ INDEPENDENT
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories ✅ INDEPENDENT
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories ✅ INDEPENDENT

**Note**: All user stories are truly independent - each can be developed, tested, and deployed separately after foundational phase

### Within Each User Story

- Backend models before services
- Services before endpoints/routes
- RAG pipeline before chat endpoint
- Frontend hooks before components
- Components before widget integration
- Widget bundle before Docusaurus integration

### Parallel Opportunities

**Phase 1 - Setup (9 tasks can run in parallel)**:
- T003-T009 (all marked [P]) - Different files, no dependencies

**Phase 2 - Foundational (13 parallel tasks)**:
- T012-T013 (Pydantic models) - Different files
- T014-T016 (Services) - Different files
- T018-T020 (Ingestion pipeline) - Different files

**Phase 3 - User Story 1 (10 parallel tasks)**:
- T024-T025 (Backend services) - Different files
- T030-T034 (Frontend components) - Different files

**Phase 4 - User Story 2 (2 parallel tasks)**:
- T042 (useHighlight hook) can run parallel with T039-T041 (backend changes)

**Phase 6 - Polish (8 parallel tasks)**:
- T054-T060 (error handling, styling, testing) - Different areas

---

## Parallel Example: User Story 1

```bash
# Launch all backend services together (T024-T025):
Task: "Create llm_service.py with OpenAI LLM generation"
Task: "Create rag_pipeline.py with RAG orchestration"

# Launch all frontend components together (T030-T034):
Task: "Create chatApi.ts with POST /chat API client"
Task: "Create useChat.ts with conversation state management"
Task: "Create MessageList.tsx with message display"
Task: "Create MessageInput.tsx with text input"
Task: "Create SourceReference.tsx with citation display"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T023) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (T024-T038)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Ask "What is ROS 2?" → Verify answer with sources
   - Ask "Explain inverse kinematics" → Verify grounded response
   - Ask "How to weaponize robots?" → Verify refusal message
   - Test "not covered" fallback with irrelevant query
5. Deploy/demo if ready - **THIS IS THE MVP**

### Incremental Delivery

1. Complete Setup + Foundational (T001-T023) → Foundation ready
2. Add User Story 1 (T024-T038) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (T039-T046) → Test independently → Deploy/Demo (highlight feature added)
4. Add User Story 3 (T047-T053) → Test independently → Deploy/Demo (conversation feature added)
5. Add Polish (T054-T066) → Final QA → Production deployment
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T023)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T024-T038) - Core Q&A
   - **Developer B**: User Story 2 (T039-T046) - Highlight feature
   - **Developer C**: User Story 3 (T047-T053) - Conversation feature
3. Stories complete and integrate independently
4. Team completes Polish together (T054-T066)

---

## Task Summary

**Total Tasks**: 66
- **Phase 1 - Setup**: 9 tasks (all can run in parallel)
- **Phase 2 - Foundational**: 14 tasks (13 parallelizable)
- **Phase 3 - User Story 1 (P1)**: 15 tasks (10 parallelizable)
- **Phase 4 - User Story 2 (P2)**: 8 tasks (2 parallelizable)
- **Phase 5 - User Story 3 (P3)**: 7 tasks (0 parallelizable - sequential)
- **Phase 6 - Polish**: 13 tasks (8 parallelizable)

**Parallel Opportunities**: 42 tasks (64%) can run in parallel with others in their phase

**MVP Scope** (Minimum Viable Product):
- Phase 1: Setup (9 tasks)
- Phase 2: Foundational (14 tasks)
- Phase 3: User Story 1 only (15 tasks)
- **Total for MVP**: 38 tasks

**Independent Test Criteria**:
- **User Story 1**: Student asks question → receives answer with 2-5 sources ✅
- **User Story 2**: Student highlights text → receives targeted explanation ✅
- **User Story 3**: Student asks follow-up → system maintains context ✅

---

## Notes

- All tasks follow strict checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- Tasks marked [P] can run in parallel (different files, no dependencies)
- Tasks marked [US1], [US2], [US3] map to user stories for traceability
- Each user story is independently completable and testable
- Foundational phase (T010-T023) MUST complete before any user story work begins
- No tests included per specification (tests not explicitly requested in spec.md)
- Constitution compliance enforced in T027 (safety), T028 (grounding), T029 (attribution), T051 (session-only)
- File paths match plan.md project structure (backend/src/, frontend/src/)
- Commit after each task or logical group for clean git history
