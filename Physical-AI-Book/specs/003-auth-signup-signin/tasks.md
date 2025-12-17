# Tasks: Authentication with Better-Auth (Signup & Signin)

**Input**: Design documents from `/specs/003-auth-signup-signin/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/auth-api.openapi.yaml, research.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in the feature specification. Test tasks are included for critical authentication flows but marked as optional.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/src/` (textbook/)
- Backend: TypeScript + Better-Auth server
- Frontend: React/Docusaurus + Better-Auth client

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and Better-Auth dependencies

- [ ] T001 Install Better-Auth dependencies in backend: `npm install better-auth drizzle-orm pg @better-auth/postgres` in backend/
- [ ] T002 [P] Install Better-Auth client in frontend: `npm install better-auth @better-auth/react` in textbook/
- [ ] T003 [P] Create backend TypeScript configuration in backend/tsconfig.json if not exists
- [ ] T004 [P] Create directory structure: backend/src/auth/, backend/drizzle/, textbook/src/lib/, textbook/src/components/auth/
- [ ] T005 Generate Better-Auth secret key using `openssl rand -hex 32` and document in .env.example files

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core authentication infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create Better-Auth configuration in backend/src/auth/config.ts with email/password enabled and custom user fields (softwareBackground, hardwareBackground, interestArea)
- [ ] T007 [P] Configure environment variables in backend/.env: DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, NODE_ENV
- [ ] T008 [P] Configure environment variables in textbook/.env: REACT_APP_AUTH_API_URL
- [ ] T009 Generate Drizzle schema from Better-Auth config: run `npx better-auth generate` in backend/
- [ ] T010 Review generated schema in backend/drizzle/schema.ts (verify user table has custom fields, session table has expiration)
- [ ] T011 Apply database migrations: run `npx drizzle-kit push` in backend/ to create user, session, account, verification tables
- [ ] T012 Verify tables created in PostgreSQL: `psql $DATABASE_URL -c "\dt"`
- [ ] T013 Create Better-Auth route handler in backend/src/auth/routes.ts to mount /api/auth/* endpoints
- [ ] T014 [P] Create Better-Auth TypeScript types in backend/src/auth/types.ts (User, Session, SignupRequest, SigninRequest)
- [ ] T015 Mount Better-Auth routes in FastAPI backend at /api/auth/* in backend/src/main.py (or main entrypoint)
- [ ] T016 [P] Configure CORS middleware in FastAPI to allow credentials (allow_credentials=True, specific origin)
- [ ] T017 Create Better-Auth client instance in textbook/src/lib/auth-client.ts with baseURL configured
- [ ] T018 [P] Export auth hooks (signUp, signIn, signOut, useSession) from textbook/src/lib/auth-client.ts
- [ ] T019 [Optional] Test foundational setup: run `curl http://localhost:3000/api/auth/ok` to verify Better-Auth server responding

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New User Signup with Background Collection (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with email, password, and learning background metadata (software, hardware, interest area)

**Independent Test**: Create a new account through signup form, verify account created in Better-Auth database, confirm background metadata (software level, hardware level, interest area) is stored and retrievable

### Implementation for User Story 1

- [ ] T020 [P] [US1] Create SignupForm component in textbook/src/components/auth/SignupForm.tsx with email, password, and three background dropdown fields
- [ ] T021 [P] [US1] Add form validation to SignupForm: email format (RFC 5322), password length (min 8 chars), required fields check
- [ ] T022 [US1] Implement signup submission in SignupForm.tsx calling authClient.signUp.email() with custom fields (softwareBackground, hardwareBackground, interestArea)
- [ ] T023 [US1] Add error handling in SignupForm for duplicate email (409), validation errors (400), display user-friendly messages
- [ ] T024 [US1] Add loading state and disabled button during signup API call in SignupForm.tsx
- [ ] T025 [US1] Implement redirect logic in SignupForm: on success redirect to textbook homepage with welcome message
- [ ] T026 [P] [US1] Create signup page in textbook/src/pages/signup.tsx rendering SignupForm component
- [ ] T027 [P] [US1] Add basic CSS styling to SignupForm (or integrate with Docusaurus theme)
- [ ] T028 [US1] Add navigation link "Sign Up" to Docusaurus navbar when user not authenticated

### Optional Tests for User Story 1 (run after implementation)

- [ ] T029 [Optional] [US1] Manual test: Fill signup form with valid data → verify user created in database → check custom fields persisted
- [ ] T030 [Optional] [US1] Manual test: Try signup with existing email → verify error message "This email is already registered"
- [ ] T031 [Optional] [US1] Manual test: Try signup with short password (< 8 chars) → verify error message "Password must be at least 8 characters long"
- [ ] T032 [Optional] [US1] Manual test: Complete signup → navigate to profile settings → verify background information displayed

**Checkpoint**: At this point, User Story 1 should be fully functional - users can create accounts with background metadata

---

## Phase 4: User Story 2 - Returning User Signin (Priority: P2)

**Goal**: Enable existing users to sign in with email/password and access their personalized content

**Independent Test**: Sign in with valid credentials, verify session creation, confirm user metadata is loaded and accessible after authentication

### Implementation for User Story 2

- [ ] T033 [P] [US2] Create SigninForm component in textbook/src/components/auth/SigninForm.tsx with email, password, and "Remember Me" checkbox
- [ ] T034 [P] [US2] Add form validation to SigninForm: required fields, email format check
- [ ] T035 [US2] Implement signin submission in SigninForm.tsx calling authClient.signIn.email() with rememberMe flag
- [ ] T036 [US2] Add error handling in SigninForm for invalid credentials (401), display message "Invalid email or password. Please try again."
- [ ] T037 [US2] Add loading state and disabled button during signin API call in SigninForm.tsx
- [ ] T038 [US2] Implement redirect logic in SigninForm: on success redirect to textbook homepage with user session active
- [ ] T039 [P] [US2] Create signin page in textbook/src/pages/signin.tsx rendering SigninForm component
- [ ] T040 [P] [US2] Add basic CSS styling to SigninForm (or integrate with Docusaurus theme)
- [ ] T041 [US2] Add navigation link "Sign In" to Docusaurus navbar when user not authenticated
- [ ] T042 [P] [US2] Create UserMenu component in textbook/src/components/auth/UserMenu.tsx using useSession() hook
- [ ] T043 [US2] Display user name in UserMenu when authenticated, show "Sign Up" / "Sign In" links when not authenticated
- [ ] T044 [US2] Add "Sign Out" button to UserMenu calling authClient.signOut()
- [ ] T045 [US2] Integrate UserMenu into Docusaurus navbar (replace/augment default navbar)
- [ ] T046 [P] [US2] Create session middleware in backend/src/auth/middleware.ts to extract user from Better-Auth session for protected routes
- [ ] T047 [US2] Apply session middleware to chatbot endpoint in backend/src/api/chatbot.py (or equivalent) to access user metadata
- [ ] T048 [US2] Extract user metadata (softwareBackground, hardwareBackground, interestArea) in chatbot handler and pass to LLM context
- [ ] T049 [US2] Update chatbot system prompt template to include user background context for personalization

### Optional Tests for User Story 2 (run after implementation)

- [ ] T050 [Optional] [US2] Manual test: Sign in with valid credentials → verify session created → check UserMenu shows username
- [ ] T051 [Optional] [US2] Manual test: Sign in with incorrect password → verify error message "Invalid email or password"
- [ ] T052 [Optional] [US2] Manual test: Sign in successfully → close browser → reopen within 7 days → verify still signed in (persistent session)
- [ ] T053 [Optional] [US2] Manual test: Sign in → open chatbot → ask question → verify backend receives user metadata (check logs)
- [ ] T054 [Optional] [US2] Manual test: Sign in with Remember Me unchecked → close browser → reopen → verify session expired

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - signup + signin + session management complete

---

## Phase 5: User Story 3 - Profile Background Update (Priority: P3)

**Goal**: Allow authenticated users to update their technical background information as they gain experience or change learning focus

**Independent Test**: Sign in, navigate to profile settings, update background fields, save, verify changes persisted and reflected in personalized content

### Implementation for User Story 3

- [ ] T055 [P] [US3] Create ProfileSettings component in textbook/src/components/auth/ProfileSettings.tsx with three background dropdown fields (pre-filled with current values)
- [ ] T056 [P] [US3] Fetch current user profile in ProfileSettings using useSession() hook to populate form fields
- [ ] T057 [US3] Implement profile update submission calling PATCH /api/auth/user/profile with updated background fields
- [ ] T058 [US3] Add success feedback in ProfileSettings: show "Profile updated successfully" message after save
- [ ] T059 [US3] Add error handling in ProfileSettings for validation errors (400), unauthorized (401)
- [ ] T060 [US3] Add loading state and disabled button during profile update API call
- [ ] T061 [P] [US3] Create profile page in textbook/src/pages/profile.tsx rendering ProfileSettings component (protected route, redirect if not authenticated)
- [ ] T062 [P] [US3] Add basic CSS styling to ProfileSettings (or integrate with Docusaurus theme)
- [ ] T063 [US3] Add "Profile" link to UserMenu dropdown/menu when user is authenticated
- [ ] T064 [US3] Re-fetch session after profile update to reflect new background values in useSession() hook
- [ ] T065 [US3] Verify updated background reflected in chatbot responses: update chatbot context with new metadata

### Optional Tests for User Story 3 (run after implementation)

- [ ] T066 [Optional] [US3] Manual test: Sign in → navigate to profile settings → change software background from "Beginner" to "Intermediate" → save → verify persisted in database
- [ ] T067 [Optional] [US3] Manual test: Update interest area from "AI" to "Robotics" → return to homepage → ask chatbot question → verify response prioritizes robotics content
- [ ] T068 [Optional] [US3] Manual test: Try to access profile page without signing in → verify redirected to signin page

**Checkpoint**: All user stories should now be independently functional - complete authentication flow with personalization

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, security hardening, and validation

- [ ] T069 [P] Add password strength indicator to SignupForm (visual feedback for weak/medium/strong passwords)
- [ ] T070 [P] Add "Forgot Password" link to SigninForm (link to future password reset flow, can be placeholder)
- [ ] T071 [P] Implement form input sanitization to prevent XSS attacks in all auth forms
- [ ] T072 [P] Add ARIA labels and accessibility attributes to all auth forms for screen readers
- [ ] T073 [P] Test signup/signin forms on mobile devices, adjust responsive CSS if needed
- [ ] T074 [P] Add loading skeletons to UserMenu while session is loading
- [ ] T075 [P] Implement session expiration handling: detect expired session, show message, redirect to signin
- [ ] T076 [P] Add rate limiting to prevent brute force attacks on signin endpoint (use slowapi or similar)
- [ ] T077 [P] Review and harden CORS configuration: ensure only production domain allowed in production
- [ ] T078 [P] Verify httpOnly and secure flags set correctly on session cookies in production
- [ ] T079 [P] Add logging for authentication events: signup, signin, signout, failed attempts (no passwords logged)
- [ ] T080 [P] Update quickstart.md with actual file paths and commands from implementation
- [ ] T081 [P] Create deployment checklist: environment variables, database migrations, CORS config, HTTPS setup
- [ ] T082 [P] Document API endpoints in README or API documentation (reference contracts/auth-api.openapi.yaml)
- [ ] T083 [P] Add user data export functionality for GDPR compliance (optional, future enhancement)
- [ ] T084 [Optional] Run through quickstart.md validation: verify setup instructions work on fresh environment
- [ ] T085 [Optional] Perform security audit: review password handling, session management, CSRF protection
- [ ] T086 [Optional] Load testing: verify session performance with 100+ concurrent users
- [ ] T087 [Optional] Create Playwright E2E test suite: signup flow, signin flow, profile update flow

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on US1 for core signin, but integrates with US1 for complete flow (users must exist to sign in)
- **User Story 3 (P3)**: Depends on US2 (requires authenticated session) - Should start after US2 complete

### Within Each User Story

- Forms before pages (components before routes)
- Frontend components before backend integration
- API calls after form validation
- Error handling after happy path
- CSS/styling can be done in parallel with functionality

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T003, T004)
- Many Foundational tasks marked [P] can run in parallel (T007, T008, T014, T016, T018)
- User Story 1 tasks T020, T021, T026, T027 can run in parallel (different files)
- User Story 2 tasks T033, T034, T039, T040, T042, T046 can run in parallel (different files)
- User Story 3 tasks T055, T056, T061, T062 can run in parallel (different files)
- All Polish phase tasks marked [P] can run in parallel (T069-T082)
- With team: US1 by Dev A, US2 by Dev B in parallel (after Foundational complete)

---

## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
Task: "Create SignupForm component in textbook/src/components/auth/SignupForm.tsx"
Task: "Add form validation to SignupForm: email format, password length, required fields"
Task: "Create signup page in textbook/src/pages/signup.tsx"
Task: "Add basic CSS styling to SignupForm"

# Then run sequential tasks:
Task: "Implement signup submission calling authClient.signUp.email()"
Task: "Add error handling for duplicate email, validation errors"
Task: "Add loading state and redirect logic"
Task: "Add navigation link to navbar"
```

## Parallel Example: User Story 2

```bash
# Launch all parallel tasks for User Story 2 together:
Task: "Create SigninForm component in textbook/src/components/auth/SigninForm.tsx"
Task: "Add form validation to SigninForm"
Task: "Create signin page in textbook/src/pages/signin.tsx"
Task: "Add basic CSS styling to SigninForm"
Task: "Create UserMenu component in textbook/src/components/auth/UserMenu.tsx"
Task: "Create session middleware in backend/src/auth/middleware.ts"

# Then run sequential tasks:
Task: "Implement signin submission calling authClient.signIn.email()"
Task: "Display user name in UserMenu, add Sign Out button"
Task: "Apply session middleware to chatbot endpoint"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (~1-2 hours)
2. Complete Phase 2: Foundational (~3-4 hours) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (~4-6 hours)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Can users create accounts?
   - Is background metadata stored correctly?
   - Do validation errors display properly?
5. Deploy/demo if ready (MVP: Signup working!)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (~4-6 hours total)
2. Add User Story 1 → Test independently → Deploy/Demo (~4-6 hours, MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (~6-8 hours, adds signin + chatbot integration)
4. Add User Story 3 → Test independently → Deploy/Demo (~3-4 hours, adds profile update)
5. Add Polish phase → Security hardening, accessibility, docs (~4-6 hours)
6. Each story adds value without breaking previous stories

**Total Estimated Time**: 21-30 hours for complete implementation (all 3 user stories + polish)

**MVP Time**: 8-12 hours (Setup + Foundational + User Story 1 only)

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (~4-6 hours)
2. **Once Foundational is done**:
   - Developer A: User Story 1 (Signup) - ~4-6 hours
   - Developer B: User Story 2 (Signin + Chatbot) - ~6-8 hours (can start in parallel, integrates at end)
   - Developer C: Polish tasks (prepare docs, security review) - ~2-3 hours
3. **After US1 + US2 complete**:
   - Any developer: User Story 3 (Profile Update) - ~3-4 hours
4. **Final integration**: Verify all stories work together - ~1-2 hours

**Total Parallel Team Time**: ~12-16 hours with 2-3 developers

---

## Notes

- [P] tasks = different files, no dependencies, safe to run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Optional tests can be run manually or automated later with Playwright
- Commit after each task or logical group (e.g., after completing a component)
- Stop at any checkpoint to validate story independently before moving to next
- Better-Auth handles password hashing, CSRF, session management automatically
- Focus on form validation and user experience in frontend tasks
- Backend tasks are minimal due to Better-Auth providing authentication logic
- Most complexity is in frontend (forms, validation, user flow)
- Chatbot integration (US2) requires backend session middleware
- Profile update (US3) requires both frontend form and backend PATCH endpoint support

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 14 tasks (BLOCKING)
- **Phase 3 (User Story 1 - P1)**: 13 tasks (9 implementation + 4 optional tests)
- **Phase 4 (User Story 2 - P2)**: 22 tasks (17 implementation + 5 optional tests)
- **Phase 5 (User Story 3 - P3)**: 14 tasks (11 implementation + 3 optional tests)
- **Phase 6 (Polish)**: 19 tasks

**Total**: 87 tasks (75 implementation + 12 optional tests)

**Core Implementation** (without optional tests): 75 tasks

**Parallel Opportunities**: 35+ tasks marked [P] can run in parallel

**MVP Scope** (Recommended): Phase 1 + Phase 2 + Phase 3 (User Story 1 only) = 32 tasks (~8-12 hours)
