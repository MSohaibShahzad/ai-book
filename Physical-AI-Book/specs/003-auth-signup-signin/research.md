# Research: Authentication with Better-Auth

**Feature**: 003-auth-signup-signin
**Date**: 2025-12-16
**Research Phase**: Phase 0

## Overview

This document consolidates research findings for implementing email/password authentication using Better-Auth library with custom user metadata for personalization.

## Technology Selection

### Authentication Library: Better-Auth

**Decision**: Use Better-Auth v1.3+ as the authentication framework

**Rationale**:
- Framework-agnostic TypeScript library compatible with FastAPI backend and React frontend
- Native support for email/password authentication with secure defaults
- Built-in session management with configurable duration
- Extensible schema system for adding custom user fields (software background, hardware background, interest area)
- Strong security defaults (password hashing, httpOnly cookies, CSRF protection)
- Official documentation available via Context7 MCP (2274 code snippets, 84.8 benchmark score)
- Active maintenance and high source reputation

**Alternatives Considered**:
- **NextAuth.js**: Rejected - primarily designed for Next.js, not framework-agnostic
- **Passport.js**: Rejected - older architecture, requires more manual security configuration
- **Auth0/Clerk**: Rejected - third-party services add cost and external dependencies
- **Custom implementation**: Rejected - security-critical code should use battle-tested libraries

## Architecture Pattern

### Full-Stack Integration Approach

**Decision**: Backend-first authentication with Better-Auth server instance, frontend client library

**Architecture Components**:

1. **Backend (FastAPI + Better-Auth)**:
   - Better-Auth server instance configured with PostgreSQL adapter
   - Authentication endpoints mounted at `/api/auth/*`
   - User metadata stored in extended user table schema
   - Session management with secure cookie configuration

2. **Frontend (React/Docusaurus + Better-Auth Client)**:
   - Better-Auth client library for API communication
   - React components for signup/signin forms
   - Session state management
   - Integration with existing chatbot for metadata access

3. **Database (PostgreSQL/Neon)**:
   - Extended user table with custom fields: `softwareBackground`, `hardwareBackground`, `interestArea`
   - Session table with httpOnly cookie tokens
   - Automated migrations via Better-Auth CLI

**Rationale**:
- Backend-first approach maintains security (credentials never exposed to frontend)
- Better-Auth provides type-safe client/server communication
- PostgreSQL already in use for RAG metadata (002-rag-chatbot)
- Schema extension system allows custom fields without migration complexity

## Custom User Metadata Implementation

### Schema Extension Strategy

**Decision**: Use Better-Auth's `additionalFields` API to extend user schema

**Implementation Pattern**:
```typescript
// Backend configuration
export const auth = betterAuth({
  database: postgresAdapter,
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        defaultValue: "Beginner",
        input: true, // Allow user input during signup
      },
      hardwareBackground: {
        type: "string",
        required: false,
        defaultValue: "None",
        input: true,
      },
      interestArea: {
        type: "string",
        required: false,
        defaultValue: "AI",
        input: true,
      },
    },
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Auto sign-in after signup
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Update session every 24 hours
  },
});
```

**Rationale**:
- `input: true` allows frontend to pass custom fields during signup
- Type-safe access to custom fields via Better-Auth's TypeScript inference
- Fields automatically available in session object and API endpoints
- No manual database migrations required (Better-Auth handles schema sync)

## Security Configuration

### Session Management

**Decision**: Secure session cookies with 7-day expiration, httpOnly and secure flags

**Configuration**:
- Session duration: 7 days (matches success criteria SC-007)
- Cookie flags: `httpOnly`, `secure` (HTTPS only), `sameSite: "lax"`
- CSRF protection: Built-in Better-Auth CSRF tokens
- Password hashing: Better-Auth's default bcrypt implementation

**Rationale**:
- httpOnly cookies prevent XSS attacks (meets FR-014, FR-015)
- 7-day expiration balances security and user convenience
- Built-in CSRF protection meets constitution requirement XVI
- bcrypt is industry-standard for password hashing (meets FR-005)

## Frontend Integration Points

### Signup Form Implementation

**Decision**: React component with controlled inputs, inline validation, background selection

**Form Fields**:
1. Email (text input with validation)
2. Password (password input with strength indicator)
3. Software Background (dropdown: Beginner/Intermediate/Advanced)
4. Hardware Background (dropdown: None/Basic/Hands-on)
5. Interest Area (dropdown: AI/Robotics/Simulation/Humanoids)

**API Call Pattern**:
```typescript
const { data, error } = await authClient.signUp.email({
  email: userEmail,
  password: userPassword,
  name: userEmail.split('@')[0], // Default name from email
  softwareBackground: selectedSoftware,
  hardwareBackground: selectedHardware,
  interestArea: selectedInterest,
});
```

**Rationale**:
- Controlled inputs provide real-time validation (meets SC-003)
- Dropdown selections ensure data consistency
- Better-Auth client handles request/response automatically
- Error handling built into client library

### Signin Form Implementation

**Decision**: Simple email/password form with "remember me" option

**Form Fields**:
1. Email (text input)
2. Password (password input)
3. Remember Me (checkbox - controls session persistence)

**API Call Pattern**:
```typescript
const { data, error } = await authClient.signIn.email({
  email: userEmail,
  password: userPassword,
  rememberMe: rememberMeChecked,
});
```

**Rationale**:
- Minimal fields reduce friction (meets SC-002: signin within 10 seconds)
- Remember me option gives users control over session duration
- Better-Auth handles session creation automatically

## Chatbot Integration

### Metadata Access Pattern

**Decision**: Extend FastAPI backend to expose authenticated user metadata to chatbot service

**Implementation Strategy**:
1. Better-Auth provides session middleware for FastAPI
2. Protected endpoints extract user ID from session
3. User metadata fetched from database via Better-Auth API
4. Metadata passed to RAG/LLM context for personalization

**API Pattern**:
```python
# FastAPI endpoint with Better-Auth session
@app.post("/api/chatbot/query")
async def chatbot_query(
    query: str,
    session: Session = Depends(get_better_auth_session)
):
    user = await auth.get_user(session.user_id)
    user_context = {
        "software_background": user.softwareBackground,
        "hardware_background": user.hardwareBackground,
        "interest_area": user.interestArea,
    }
    # Pass user_context to RAG system for personalization
    return personalized_response
```

**Rationale**:
- Session middleware protects chatbot endpoints
- User metadata available within 1 second (meets SC-005)
- Separation of concerns: auth logic in Better-Auth, personalization in RAG backend

## Database Migration Strategy

### Schema Generation and Sync

**Decision**: Use Better-Auth CLI for automated schema generation and migrations

**Migration Commands**:
```bash
# Generate Drizzle schema from Better-Auth config
npx better-auth generate

# Apply migrations to database
npx drizzle-kit push
```

**Schema Output** (PostgreSQL):
```sql
-- Extended user table
CREATE TABLE "user" (
  "id" TEXT PRIMARY KEY,
  "name" TEXT NOT NULL,
  "email" TEXT NOT NULL UNIQUE,
  "emailVerified" BOOLEAN DEFAULT false,
  "image" TEXT,
  "createdAt" TIMESTAMP DEFAULT NOW(),
  "updatedAt" TIMESTAMP DEFAULT NOW(),
  "softwareBackground" TEXT DEFAULT 'Beginner',
  "hardwareBackground" TEXT DEFAULT 'None',
  "interestArea" TEXT DEFAULT 'AI'
);

-- Session table
CREATE TABLE "session" (
  "id" TEXT PRIMARY KEY,
  "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
  "expiresAt" TIMESTAMP NOT NULL,
  "token" TEXT NOT NULL UNIQUE,
  "ipAddress" TEXT,
  "userAgent" TEXT
);
```

**Rationale**:
- Better-Auth CLI automates schema generation (reduces manual error)
- Drizzle ORM provides type-safe database queries
- Cascading deletes maintain referential integrity
- Schema matches existing PostgreSQL setup (002-rag-chatbot)

## Deployment Considerations

### Environment Variables

**Required Configuration**:
```bash
# Backend .env
DATABASE_URL=postgresql://user:pass@host:5432/dbname
BETTER_AUTH_SECRET=<generated-secret-key>
BETTER_AUTH_URL=https://textbook.example.com
NODE_ENV=production

# Frontend .env
REACT_APP_AUTH_API_URL=https://textbook.example.com/api/auth
```

**Rationale**:
- Secrets managed via environment variables (meets FR-015, constitution XVI)
- Separate frontend/backend configs for security
- Production URL required for cookie domain configuration

### CORS Configuration

**Decision**: Configure CORS to allow frontend origin for authentication endpoints

**FastAPI CORS Setup**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://textbook.example.com"],
    allow_credentials=True,  # Required for cookies
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type", "Authorization"],
)
```

**Rationale**:
- `allow_credentials=True` required for httpOnly cookies
- Specific origin prevents CSRF attacks
- Minimal allowed methods reduce attack surface

## Best Practices from Documentation

### Key Findings from Better-Auth Documentation

1. **Email/Password Configuration**:
   - Enable with `emailAndPassword.enabled: true`
   - Set minimum password length (default: 8, max: 128)
   - Use `autoSignIn: true` to automatically sign in users after signup

2. **Custom Fields Access**:
   - Custom fields with `input: true` are automatically available in signup endpoints
   - Fields are type-safe and inferred in TypeScript
   - Access via `res.user.fieldName` after signup/signin

3. **Session Management**:
   - Configure `expiresIn` in seconds for session duration
   - Use `rememberMe: false` in signin to create session-only cookies
   - Sessions automatically renewed on activity

4. **Security Defaults**:
   - Password hashing uses bcrypt by default (secure)
   - httpOnly cookies enabled automatically
   - CSRF protection built-in for state-changing operations

5. **Migration Workflow**:
   - Use `npx better-auth generate` to create schema from config
   - Apply migrations with Drizzle ORM (`npx drizzle-kit push`)
   - Schema changes automatically detected

## Implementation Priorities

Based on user stories and success criteria:

**P1 - Signup with Background Collection** (User Story 1):
- Implement Better-Auth backend configuration with extended user schema
- Create signup form with email, password, and background fields
- Database migration for user table with custom fields
- Form validation and error handling

**P2 - Signin and Session Management** (User Story 2):
- Implement signin form with remember me option
- Configure 7-day session expiration
- Session middleware for protected routes
- Chatbot integration to access user metadata

**P3 - Profile Update** (User Story 3):
- Profile settings page to view current background
- Update endpoint for changing background fields
- Real-time reflection in chatbot responses

## Open Questions Resolved

1. **Q: How to store custom user metadata?**
   **A**: Use Better-Auth's `additionalFields` API with `input: true` to extend user schema.

2. **Q: How to pass metadata to chatbot?**
   **A**: Use Better-Auth session middleware in FastAPI to extract user ID, fetch user object with metadata, pass to RAG context.

3. **Q: How to handle database migrations?**
   **A**: Use Better-Auth CLI (`npx better-auth generate`) + Drizzle ORM for automated schema generation and migrations.

4. **Q: How to configure 7-day sessions?**
   **A**: Set `session.expiresIn: 60 * 60 * 24 * 7` (seconds) in Better-Auth config.

5. **Q: How to ensure security?**
   **A**: Better-Auth provides secure defaults (bcrypt, httpOnly cookies, CSRF) - no additional configuration needed for basic security.

## Next Steps

1. Create `data-model.md` with detailed schema definitions
2. Generate API contracts in `/contracts/` directory
3. Write `quickstart.md` with setup instructions
4. Proceed to `/sp.tasks` for implementation task breakdown
