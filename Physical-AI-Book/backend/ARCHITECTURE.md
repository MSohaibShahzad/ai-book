# Backend Architecture - Microservices with Docker

## Overview

The Physical AI Book backend uses a **microservices architecture** with two independent Docker containers:

1. **auth-service** - Authentication & JWT issuance (Node.js)
2. **api-service** - RAG chatbot & business logic (Python)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                           Frontend                              │
│                    (React + Docusaurus)                         │
└────────────┬────────────────────────────────┬───────────────────┘
             │                                │
             │ 1. Sign In/Up                  │ 3. Chat (with JWT)
             │ POST /api/auth/sign-in/email   │ POST /v1/chat
             │                                │ Header: Authorization: Bearer <JWT>
             ▼                                ▼
┌────────────────────────────────┐  ┌────────────────────────────────┐
│       auth-service:3001        │  │       api-service:8000         │
│      (Node.js + TypeScript)    │  │    (Python + FastAPI)          │
├────────────────────────────────┤  ├────────────────────────────────┤
│                                │  │                                │
│  • Better Auth                 │  │  • JWT Verification            │
│  • Email/Password signup       │  │  • RAG Pipeline                │
│  • OAuth providers (future)    │  │  • Vector DB queries           │
│  • Session management          │  │  • LLM generation              │
│  • JWT generation              │  │  • Business APIs               │
│                                │  │                                │
│  Routes:                       │  │  Routes:                       │
│  • POST /api/auth/sign-up      │  │  • POST /v1/chat               │
│  • POST /api/auth/sign-in      │  │  • POST /v1/chat/stream        │
│  • POST /api/auth/sign-out     │  │  • GET /health                 │
│  • GET /api/auth/jwt ←─────────┤  │                                │
│  • GET /health                 │  │  Middleware:                   │
│                                │  │  • JWT verification            │
│  Dependencies:                 │  │  • Rate limiting               │
│  • better-auth                 │  │  • Request logging             │
│  • jsonwebtoken                │  │                                │
│  • pg (PostgreSQL)             │  │  Dependencies:                 │
│  • drizzle-orm                 │  │  • FastAPI                     │
│                                │  │  • PyJWT                       │
│                                │  │  • Qdrant Client               │
│                                │  │  • OpenAI SDK                  │
└────────────┬───────────────────┘  └────────────┬───────────────────┘
             │                                   │
             │ 2. Returns JWT                    │
             │ {"token": "eyJhbG..."}            │
             │                                   │
             │                                   │
             ▼                                   ▼
┌────────────────────────────────┐  ┌────────────────────────────────┐
│      PostgreSQL Database       │  │       Qdrant Cloud             │
│      (Neon or Self-hosted)     │  │    (Vector Database)           │
├────────────────────────────────┤  ├────────────────────────────────┤
│  Tables:                       │  │  Collection:                   │
│  • user                        │  │  • textbook_chunks             │
│  • session                     │  │                                │
│  • account                     │  │  Vectors:                      │
│                                │  │  • text-embedding-3-small      │
│  Used by: auth-service         │  │  • 1536 dimensions             │
└────────────────────────────────┘  │                                │
                                    │  Used by: api-service          │
                                    └────────────────────────────────┘
                                               │
                                               ▼
                                    ┌────────────────────────────────┐
                                    │        OpenAI API              │
                                    ├────────────────────────────────┤
                                    │  • Embeddings                  │
                                    │  • LLM (gpt-4o-mini)           │
                                    │                                │
                                    │  Used by: api-service          │
                                    └────────────────────────────────┘
```

## Service Responsibilities

### ✅ auth-service (Node.js) - HANDLES:
- User registration (email/password)
- User login
- Session creation and management
- JWT token generation
- OAuth providers (Google, GitHub - future)
- Password hashing and validation
- User profile updates
- Rate limiting for auth endpoints

### ✅ api-service (Python) - HANDLES:
- JWT token verification ONLY
- RAG chatbot logic
- Vector database queries (Qdrant)
- Semantic search
- LLM prompt construction
- OpenAI API calls
- Response streaming
- User context personalization
- Business logic

### ❌ api-service - DOES NOT HANDLE:
- User signup/signin
- Password validation
- Session creation
- OAuth
- Any authentication logic

## Data Flow

### Authentication Flow (Login)

```
1. User submits credentials
   ↓
2. Frontend → auth-service:3001 POST /api/auth/sign-in/email
   ↓
3. auth-service validates password (bcrypt)
   ↓
4. auth-service creates session in PostgreSQL
   ↓
5. auth-service generates JWT token
   ↓
6. auth-service returns: {token: "eyJhbG...", user: {...}}
   ↓
7. Frontend stores JWT (localStorage/memory)
```

### API Request Flow (Chat)

```
1. User sends chat message
   ↓
2. Frontend → api-service:8000 POST /v1/chat
   Headers: {Authorization: "Bearer eyJhbG..."}
   ↓
3. api-service JWT middleware verifies token (PyJWT)
   ├─ Valid → Continue
   └─ Invalid → Return 401 Unauthorized
   ↓
4. api-service extracts user context from JWT payload
   ↓
5. api-service generates embedding (OpenAI)
   ↓
6. api-service queries Qdrant for relevant chunks
   ↓
7. api-service constructs personalized prompt
   ↓
8. api-service calls OpenAI LLM
   ↓
9. api-service returns response + sources
```

## JWT Token Structure

**Generated by**: auth-service
**Verified by**: api-service

### Payload:
```json
{
  "userId": "user_abc123",
  "email": "user@example.com",
  "name": "John Doe",
  "emailVerified": true,
  "softwareBackground": "Intermediate",
  "hardwareBackground": "Beginner",
  "interestArea": "AI",
  "iat": 1703001234,
  "exp": 1703606034,
  "iss": "auth-service",
  "aud": "api-service"
}
```

### Signature:
- **Algorithm**: HS256 (symmetric)
- **Secret**: Shared `JWT_SECRET` environment variable
- **Expiry**: 7 days

## Environment Variables

### Shared (Both Services):
```bash
DATABASE_URL          # PostgreSQL connection string
BETTER_AUTH_SECRET    # Better Auth encryption key
JWT_SECRET           # JWT signing secret (MUST match!)
```

### auth-service only:
```bash
AUTH_SERVER_PORT     # Default: 3001
AUTH_SERVER_HOST     # Default: 0.0.0.0
BETTER_AUTH_URL      # Public URL of auth service
NODE_ENV             # production | development
```

### api-service only:
```bash
QDRANT_URL           # Qdrant Cloud URL
QDRANT_API_KEY       # Qdrant API key
OPENAI_API_KEY       # OpenAI API key
EMBEDDING_MODEL      # text-embedding-3-small
LLM_MODEL            # gpt-4o-mini
AUTH_SERVICE_URL     # Internal Docker URL: http://auth-service:3001
```

## Docker Networking

Services communicate via internal Docker network: `ai-book-network`

### Internal URLs (Container-to-Container):
- auth-service: `http://auth-service:3001`
- api-service: `http://api-service:8000`

### External URLs (Host Access):
- auth-service: `http://localhost:3001`
- api-service: `http://localhost:8000`

## Security Features

### 🔒 auth-service:
- ✅ Password hashing (bcrypt via Better Auth)
- ✅ Rate limiting (5 attempts / 15 min)
- ✅ httpOnly session cookies
- ✅ CORS protection
- ✅ SQL injection prevention (Drizzle ORM)
- ✅ JWT expiry (7 days)

### 🔒 api-service:
- ✅ JWT signature verification
- ✅ JWT expiry validation
- ✅ Issuer/audience validation
- ✅ Rate limiting (10 req/min)
- ✅ CORS protection
- ✅ No direct database access (stateless)

## Deployment Strategy

### Development:
```bash
docker compose up --build
```

### Production (Separate Deployments):

**Option 1: Same Platform**
- Deploy both services to Railway/Render
- Use internal network for service-to-service calls

**Option 2: Separate Platforms**
- auth-service → Vercel (serverless)
- api-service → Railway (long-running)
- Update `AUTH_SERVICE_URL` to public URL

**Environment Variables**:
- Ensure `JWT_SECRET` is identical across deployments
- Use production database URLs
- Enable SSL/TLS for all connections

## Monitoring

### Health Checks:
- `GET /health` on both services
- Docker health checks every 30s
- Restart on failure (3 retries)

### Logs:
```bash
docker compose logs -f auth-service
docker compose logs -f api-service
```

### Metrics (Future):
- Request latency (p50, p95, p99)
- Error rates
- JWT verification failures
- RAG pipeline performance

## Scaling Considerations

### Horizontal Scaling:
- ✅ api-service: Stateless, can scale infinitely
- ⚠️ auth-service: Requires session storage (use Redis for multi-instance)

### Database:
- PostgreSQL: Use connection pooling (pgBouncer)
- Qdrant: Managed cloud service (auto-scales)

### Caching:
- Add Redis for:
  - Session storage (auth-service)
  - JWT blacklist (logout)
  - Frequently accessed data

## Future Enhancements

- [ ] Refresh tokens (extend sessions without re-login)
- [ ] OAuth providers (Google, GitHub)
- [ ] API keys for server-to-server auth
- [ ] Asymmetric JWT (RS256) for public key distribution
- [ ] Role-based access control (RBAC)
- [ ] Multi-factor authentication (MFA)
- [ ] Audit logging
- [ ] Rate limiting per user (not just IP)
