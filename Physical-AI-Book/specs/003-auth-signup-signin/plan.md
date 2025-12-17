# Implementation Plan: Authentication with Better-Auth (Signup & Signin)

**Branch**: `003-auth-signup-signin` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-auth-signup-signin/spec.md`

## Summary

Implement email/password authentication using Better-Auth library with custom user metadata (software background, hardware background, interest area) to enable personalized textbook content and chatbot responses. Architecture includes Better-Auth server instance in backend (TypeScript + PostgreSQL), Better-Auth client in frontend (React/Docusaurus), and extended user schema with three custom fields for learning personalization.

## Technical Context

**Language/Version**: TypeScript 5.3+ (Better-Auth), Python 3.11+ (FastAPI backend integration), Node.js 18+ (runtime)
**Primary Dependencies**: Better-Auth 1.3+ (auth library), Drizzle ORM (database migrations), PostgreSQL adapter (database layer), @better-auth/react (client hooks)
**Storage**: PostgreSQL/Neon (existing from 002-rag-chatbot) with extended user schema
**Testing**: Jest (frontend unit tests), pytest (backend integration tests), Playwright (E2E auth flows)
**Target Platform**: Web application (Chrome, Firefox, Safari latest versions)
**Project Type**: Web (backend + frontend)
**Performance Goals**: Signup < 3min total (SC-001), Signin < 10sec (SC-002), 95%+ success rates (SC-003/004), metadata retrieval < 1sec (SC-005)
**Constraints**: httpOnly secure cookies required, no credential exposure to frontend, 7-day session expiration, GDPR compliance (user data access/update/delete)
**Scale/Scope**: Initial launch: 100-1000 concurrent users, session table growth ~10k rows/month, user table ~5k users first year

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication & User Management Principles (XIV-XVII)

✅ **XIV. Documentation-Driven Authentication (MANDATORY)**:
- Better-Auth official documentation consulted via Context7 MCP (`/better-auth/better-auth`)
- 2274 code snippets reviewed for email/password setup, custom schema fields, session management
- Implementation follows Better-Auth recommended patterns (additionalFields API, session config, httpOnly cookies)
- All code examples verified against current documentation (v1.3+)

✅ **XV. User Background Collection**:
- Three custom fields defined: `softwareBackground`, `hardwareBackground`, `interestArea`
- Fields stored as user metadata (extended schema via `additionalFields`)
- All fields optional with sensible defaults (Beginner, None, AI)
- User can update background via profile page (P3 user story)
- Metadata enables RAG chatbot personalization

✅ **XVI. Secure Credential Management**:
- Passwords hashed via Better-Auth's bcrypt implementation (never stored plaintext)
- Session tokens in httpOnly, secure, sameSite=Lax cookies (FR-014)
- `BETTER_AUTH_SECRET` stored in backend `.env` only (FR-015)
- No credentials or secrets exposed to frontend code
- CSRF protection built into Better-Auth

✅ **XVII. Simple & Minimal Authentication**:
- Email/password as sole authentication method (P1/P2)
- Signup form: 5 fields (email, password, 3 background dropdowns)
- Signin form: 2 fields + remember me checkbox
- Clear error messages for validation failures (FR-011)
- Password reset not in initial scope (future enhancement)

### RAG Chatbot Principles (VII-XIII) - Integration Points

✅ **VII. Content Grounding (MANDATORY)**:
- Chatbot accesses user metadata via Better-Auth session middleware
- User background passed as context to LLM for personalized recommendations
- No changes to core RAG grounding logic (still retrieves from textbook chunks)

✅ **XII. Markdown Response Formatting**:
- User profile data displayed in Markdown format (profile settings page)
- No changes to chatbot response formatting

### Result: **ALL GATES PASSED** ✅

No violations detected. Simple authentication with secure defaults aligns with constitution principles.

## Project Structure

### Documentation (this feature)

```text
specs/003-auth-signup-signin/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Better-Auth architecture decisions
├── data-model.md        # Phase 1 output: User/Session schema definitions
├── quickstart.md        # Phase 1 output: Developer setup guide
├── contracts/           # Phase 1 output: API specifications
│   └── auth-api.openapi.yaml  # OpenAPI 3.1 spec for auth endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created yet)
```

### Source Code (repository root)

```text
# Web application structure (backend + frontend)

backend/
├── src/
│   ├── auth/                    # NEW: Better-Auth integration
│   │   ├── config.ts            # Better-Auth instance + schema
│   │   ├── routes.ts            # Mount /api/auth/* endpoints
│   │   ├── middleware.ts        # Session validation middleware
│   │   └── types.ts             # TypeScript types for User/Session
│   ├── models/                  # Existing RAG models
│   ├── services/                # Existing RAG services
│   │   └── personalization.py  # NEW: User metadata integration
│   ├── api/                     # Existing RAG API endpoints
│   │   └── chatbot.py           # MODIFIED: Add session middleware
│   └── main.py                  # MODIFIED: Register auth routes
├── drizzle/                     # NEW: Database migrations
│   ├── schema.ts                # Generated from Better-Auth config
│   └── migrations/              # Auto-generated migration files
├── tests/                       # Existing tests
│   └── auth/                    # NEW: Auth integration tests
│       ├── test_signup.py
│       ├── test_signin.py
│       └── test_session.py
└── package.json                 # NEW: TypeScript/Better-Auth deps

frontend/ (textbook/)
├── src/
│   ├── components/              # Existing Docusaurus components
│   │   ├── SignupForm.tsx       # NEW: Signup form component
│   │   ├── SigninForm.tsx       # NEW: Signin form component
│   │   ├── UserMenu.tsx         # NEW: Session display + signout
│   │   └── ProfileSettings.tsx  # NEW: Update background fields (P3)
│   ├── pages/                   # Existing Docusaurus pages
│   │   ├── signup.tsx           # NEW: Signup page
│   │   ├── signin.tsx           # NEW: Signin page
│   │   └── profile.tsx          # NEW: Profile settings page
│   ├── lib/                     # NEW: Client libraries
│   │   └── auth-client.ts       # Better-Auth client instance
│   └── hooks/                   # Existing hooks
│       └── useAuth.ts           # NEW: Custom auth hooks
├── tests/                       # Existing tests
│   └── auth/                    # NEW: Auth component tests
│       ├── SignupForm.test.tsx
│       └── SigninForm.test.tsx
└── package.json                 # MODIFIED: Add Better-Auth deps

shared/ (optional)
└── types/                       # NEW: Shared TypeScript types
    └── auth.ts                  # User, Session, Background enums
```

**Structure Decision**: Selected **Option 2: Web application (backend + frontend)** because this feature integrates authentication across both backend (FastAPI with Better-Auth TypeScript server) and frontend (React/Docusaurus with Better-Auth client). Backend handles session management and database operations; frontend provides UI components and client-side validation.

## Complexity Tracking

> No constitution violations requiring justification. This section intentionally left empty.

## Phase Breakdown

### Phase 0: Research & Architecture (COMPLETED ✅)

**Artifacts Generated**:
- `research.md`: Better-Auth library selection, custom schema extension strategy, session management configuration, chatbot integration patterns

**Key Decisions**:
1. Use Better-Auth v1.3+ for framework-agnostic TypeScript authentication
2. Extend user schema via `additionalFields` API (softwareBackground, hardwareBackground, interestArea)
3. 7-day session expiration with httpOnly cookies
4. PostgreSQL adapter reusing existing database (002-rag-chatbot)
5. Chatbot accesses metadata via Better-Auth session middleware in FastAPI

### Phase 1: Design & Contracts (COMPLETED ✅)

**Artifacts Generated**:
1. `data-model.md`: User/Session entity schemas, indexes, validation rules, TypeScript types
2. `contracts/auth-api.openapi.yaml`: OpenAPI 3.1 spec for 6 endpoints (signup, signin, signout, session, profile GET/PATCH)
3. `quickstart.md`: Step-by-step developer setup guide with code examples

**Database Schema**:
- **user** table: id, name, email, emailVerified, image, createdAt, updatedAt, softwareBackground, hardwareBackground, interestArea
- **session** table: id, userId (FK), expiresAt, token, createdAt, updatedAt, ipAddress, userAgent
- Indexes: email (unique), token (unique), userId, expiresAt

**API Endpoints**:
- `POST /api/auth/sign-up/email`: Create user + session
- `POST /api/auth/sign-in/email`: Authenticate + create session
- `POST /api/auth/sign-out`: Invalidate session
- `GET /api/auth/session`: Get current user + session
- `GET /api/auth/user/profile`: Get user profile
- `PATCH /api/auth/user/profile`: Update background fields

### Phase 2: Task Generation (NEXT STEP)

**Output**: `tasks.md` with implementation tasks organized by priority

**Not included in `/sp.plan` output** - Generated by `/sp.tasks` command

## Implementation Strategy

### Backend Implementation (TypeScript + Python)

**1. Better-Auth Server Setup**:
- Install: `npm install better-auth drizzle-orm @better-auth/postgres`
- Create `backend/src/auth/config.ts` with Better-Auth instance
- Configure email/password + custom user fields
- Set session expiration to 7 days
- Generate Drizzle schema: `npx better-auth generate`

**2. Database Migrations**:
- Apply schema: `npx drizzle-kit push`
- Verify tables: user, session, account, verification

**3. FastAPI Integration**:
- Mount Better-Auth handler at `/api/auth/*` in `backend/src/auth/routes.ts`
- Create session middleware for protected endpoints
- Integrate with existing chatbot endpoint (`backend/src/api/chatbot.py`)

**4. Chatbot Personalization**:
- Extract user metadata from session in chatbot handler
- Pass `softwareBackground`, `hardwareBackground`, `interestArea` to LLM context
- Modify system prompt to use user background for recommendations

### Frontend Implementation (React/Docusaurus)

**1. Better-Auth Client Setup**:
- Install: `npm install better-auth @better-auth/react`
- Create `textbook/src/lib/auth-client.ts` with client instance
- Configure base URL to backend `/api/auth`

**2. Signup Form (P1)**:
- Component: `textbook/src/components/SignupForm.tsx`
- Fields: email, password, softwareBackground (dropdown), hardwareBackground (dropdown), interestArea (dropdown)
- Validation: email format, password length (8+ chars), required fields
- API call: `authClient.signUp.email()`
- Redirect on success to homepage

**3. Signin Form (P2)**:
- Component: `textbook/src/components/SigninForm.tsx`
- Fields: email, password, rememberMe (checkbox)
- API call: `authClient.signIn.email()`
- Error handling: "Invalid email or password" message

**4. Session Management**:
- Hook: `useSession()` from `@better-auth/react`
- Component: `UserMenu.tsx` for navbar (shows username, signout button)
- Conditional rendering: Show signup/signin links if not authenticated

**5. Profile Settings (P3)**:
- Page: `textbook/src/pages/profile.tsx`
- Component: `ProfileSettings.tsx` with background field dropdowns
- API call: `PATCH /api/auth/user/profile`
- Real-time update: Re-fetch session after save

### Testing Strategy

**1. Backend Integration Tests** (pytest):
- Test signup flow: user creation, custom fields persisted
- Test signin flow: authentication, session creation
- Test session validation: valid/expired/invalid tokens
- Test profile update: background fields modified
- Test chatbot integration: metadata passed to LLM

**2. Frontend Unit Tests** (Jest + React Testing Library):
- SignupForm: form validation, API call mocking, error display
- SigninForm: form submission, loading states
- UserMenu: authenticated/unauthenticated states
- ProfileSettings: dropdown interactions, save button

**3. E2E Tests** (Playwright):
- Complete signup flow: fill form → create account → redirect
- Complete signin flow: enter credentials → authenticate → redirect
- Session persistence: close browser → reopen → still signed in
- Profile update: change background → save → verify in chatbot response

### Deployment Checklist

**Environment Variables**:
- Backend: `DATABASE_URL`, `BETTER_AUTH_SECRET`, `BETTER_AUTH_URL`, `NODE_ENV`
- Frontend: `REACT_APP_AUTH_API_URL`

**Database**:
- Run migrations in production: `npx drizzle-kit push --prod`
- Verify user/session tables created
- Configure connection pooling

**CORS Configuration**:
- Allow frontend origin in FastAPI middleware
- Set `allow_credentials=True` for cookies

**HTTPS Configuration**:
- Enable secure cookies in production (`secure: true`)
- Verify SSL certificate valid for auth domain

**Security Audit**:
- Confirm passwords never logged or exposed
- Verify httpOnly cookies set correctly
- Test CSRF protection on state-changing endpoints
- Review session expiration behavior

## Risk Analysis

### Technical Risks

**Risk 1: Better-Auth TypeScript/Python Integration Complexity**
- **Likelihood**: Medium
- **Impact**: High (could block implementation)
- **Mitigation**: Better-Auth provides HTTP API that FastAPI can proxy. If direct Python integration unavailable, run Better-Auth as separate Node.js service alongside FastAPI, communicate via HTTP.
- **Fallback**: Mount Better-Auth endpoints in Express.js server, reverse-proxy from FastAPI.

**Risk 2: Session Cookie CORS Issues**
- **Likelihood**: Medium
- **Impact**: Medium (breaks authentication flow)
- **Mitigation**: Carefully configure CORS with `allow_credentials=True` and specific origin (not wildcard). Test with actual deployed frontend domain.
- **Fallback**: Use token-based authentication (store token in localStorage) instead of cookies if CORS unsolvable.

**Risk 3: Database Migration Conflicts with Existing Schema**
- **Likelihood**: Low
- **Impact**: Medium (could corrupt existing RAG data)
- **Mitigation**: Better-Auth creates separate tables (user, session, account, verification) that don't overlap with RAG tables. Review generated schema before `drizzle-kit push`.
- **Fallback**: Use separate database schema/namespace for auth tables.

### Business Risks

**Risk 4: User Adoption - Background Field Friction**
- **Likelihood**: Low
- **Impact**: Low (users skip background questions)
- **Mitigation**: Make background fields optional with sensible defaults. Users can update later in profile settings.
- **Acceptance**: If users skip, chatbot uses default Beginner/None/AI background. Personalization still possible based on usage patterns.

**Risk 5: Session Management Performance at Scale**
- **Likelihood**: Low (not initial concern)
- **Impact**: Medium (slower auth checks at >10k concurrent sessions)
- **Mitigation**: PostgreSQL indexes on session.token and session.expiresAt. Consider Redis cache for hot session data if needed.
- **Acceptance**: Handle via scaling (add read replicas) if performance degrades.

## Success Metrics Mapping

| Success Criterion | Implementation | Verification Method |
|-------------------|----------------|---------------------|
| **SC-001**: Signup < 3min | Form with 5 fields, inline validation | Playwright E2E timing test |
| **SC-002**: Signin < 10sec | Simple form, optimized DB query | Playwright E2E timing test |
| **SC-003**: 95% signup success rate | Robust validation, clear error messages | Analytics tracking in production |
| **SC-004**: 98% signin success rate | Password validation, account existence checks | Analytics tracking in production |
| **SC-005**: Metadata retrieval < 1sec | Indexed userId lookup, session cache | Backend API timing logs |
| **SC-006**: Zero credential exposure | httpOnly cookies, env var secrets | Security audit + code review |
| **SC-007**: 7-day session persistence | `expiresIn: 604800` config | Manual testing + session table inspection |
| **SC-008**: Profile update < 5sec | Single DB UPDATE query, immediate UI refresh | Playwright E2E timing test |

## Dependencies and Integration Points

### Internal Dependencies

1. **002-rag-chatbot**:
   - Reuses PostgreSQL database connection
   - Integrates user metadata into chatbot query context
   - Requires session middleware in chatbot API endpoint

2. **001-physical-ai-robotics-textbook**:
   - Signup/signin pages embedded in Docusaurus site
   - UserMenu component in Docusaurus navbar
   - No changes to textbook content (pure UI integration)

### External Dependencies

1. **Better-Auth** (npm): Authentication library
   - Version: 1.3+
   - License: MIT
   - Stability: Production-ready, active maintenance

2. **Drizzle ORM** (npm): Database migrations
   - Version: Latest
   - License: Apache 2.0
   - Purpose: Generate schema from Better-Auth config

3. **PostgreSQL/Neon**: Database
   - Already provisioned for 002-rag-chatbot
   - No additional setup required

## Architectural Decision Records (ADR) Suggestion

📋 **Architectural decision detected**:
- **Decision**: Use Better-Auth TypeScript library in Python FastAPI backend
- **Scope**: Cross-cutting (affects backend architecture, deployment, dependency management)
- **Impact**: Long-term (authentication foundation for all future features)
- **Alternatives**: Custom auth, Passport.js, NextAuth.js, Auth0/Clerk

**Suggested ADR**: Document reasoning and tradeoffs? Run `/sp.adr better-auth-typescript-python-integration`

## Next Steps

1. ✅ **Planning Complete** - This document (`plan.md`) generated
2. ✅ **Research Complete** - Better-Auth architecture documented in `research.md`
3. ✅ **Design Complete** - Data model and API contracts generated
4. ⏭️ **Run `/sp.tasks`** - Generate implementation task breakdown (tasks.md)
5. ⏭️ **Implement P1 Tasks** - Signup with background collection
6. ⏭️ **Implement P2 Tasks** - Signin and session management
7. ⏭️ **Implement P3 Tasks** - Profile update functionality
8. ⏭️ **Testing & Deployment** - E2E tests, production deployment

## Summary

This implementation plan establishes a secure, constitution-compliant authentication system using Better-Auth with custom user metadata for learning personalization. Backend uses Better-Auth TypeScript library with PostgreSQL adapter; frontend uses Better-Auth React client. All security requirements met (httpOnly cookies, password hashing, no credential exposure). Ready for task generation via `/sp.tasks`.

**Status**: ✅ **PLANNING COMPLETE** - Ready for `/sp.tasks` command
