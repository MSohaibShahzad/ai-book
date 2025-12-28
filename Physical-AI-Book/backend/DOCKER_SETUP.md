# Docker Setup Guide - Physical AI Book Backend

## Architecture Overview

The backend is split into **two independent Docker services**:

### 1. **auth-service** (Node.js + Better Auth)
- **Port**: 3001
- **Responsibilities**:
  - User signup/signin (email/password)
  - OAuth providers (future)
  - JWT token issuance
  - Session management
- **Technology**: Node.js 20, TypeScript, Better Auth, PostgreSQL

### 2. **api-service** (Python + FastAPI)
- **Port**: 8000
- **Responsibilities**:
  - RAG chatbot logic
  - Vector database queries (Qdrant)
  - LLM generation (OpenAI)
  - Business APIs
- **Technology**: Python 3.11, FastAPI, Qdrant, OpenAI

### Communication Flow

```
Frontend
   ↓
   ↓ (1) POST /api/auth/sign-in
   ↓
auth-service:3001
   ↓
   ↓ (2) Returns JWT token
   ↓
Frontend (stores JWT)
   ↓
   ↓ (3) GET /v1/chat (Authorization: Bearer <JWT>)
   ↓
api-service:8000
   ↓ (verifies JWT locally)
   ↓
   ↓ (4) Returns RAG response
   ↓
Frontend
```

## Directory Structure

```
backend/
├── auth/                      # Auth Service (Node.js)
│   ├── src/
│   │   ├── server.ts         # Auth server entry point
│   │   ├── config.ts         # Better Auth configuration
│   │   └── jwt.ts            # JWT generation utilities
│   ├── package.json
│   ├── tsconfig.json
│   ├── Dockerfile
│   └── .env.example
│
├── src/                       # API Service (Python/FastAPI)
│   ├── main.py               # FastAPI entry point
│   ├── config.py             # Settings
│   ├── middleware/
│   │   └── jwt_auth.py       # JWT verification middleware
│   ├── api/routes/
│   │   ├── chat.py           # RAG endpoints
│   │   └── health.py         # Health checks
│   └── services/
│       └── rag_pipeline.py   # RAG logic
│
├── docker-compose.yml         # Multi-service orchestration
├── Dockerfile.api            # FastAPI Dockerfile
├── requirements.txt          # Python dependencies
└── .env                      # Environment variables (git-ignored)
```

## Prerequisites

1. **Docker & Docker Compose** installed
2. **Environment variables** configured (see Setup below)
3. **Database**: PostgreSQL (Neon or local)
4. **Vector DB**: Qdrant Cloud account
5. **OpenAI API key**

## Setup Instructions

### Step 1: Clone and Navigate

```bash
cd backend/
```

### Step 2: Configure Environment Variables

```bash
cp .env.example .env
```

Edit `.env` and fill in your credentials:

```bash
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
DATABASE_URL=postgresql://user:password@host/dbname?sslmode=require

# OpenAI
OPENAI_API_KEY=sk-...

# Generate secrets (run these commands):
# openssl rand -hex 32
BETTER_AUTH_SECRET=<generated-secret-1>
JWT_SECRET=<generated-secret-2>
```

### Step 3: Install Auth Service Dependencies

```bash
cd auth/
npm install
cd ..
```

### Step 4: Run Database Migrations (Better Auth)

```bash
cd auth/
npm run db:push
cd ..
```

This creates the necessary user tables in your PostgreSQL database.

### Step 5: Start Services with Docker Compose

```bash
docker compose up --build
```

This will:
- Build both Docker images
- Start auth-service on port 3001
- Start api-service on port 8000
- Create an internal Docker network for inter-service communication

### Step 6: Verify Services

**Auth Service Health Check:**
```bash
curl http://localhost:3001/health
# Expected: {"status":"healthy","service":"auth-service"}
```

**API Service Health Check:**
```bash
curl http://localhost:8000/health
# Expected: {"status":"healthy","service":"api-service"}
```

## Usage

### 1. User Signup

```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "SecurePassword123",
    "name": "John Doe",
    "softwareBackground": "Intermediate",
    "hardwareBackground": "Beginner",
    "interestArea": "AI"
  }'
```

Response includes a session cookie.

### 2. Get JWT Token

```bash
curl http://localhost:3001/api/auth/jwt \
  -H "Cookie: better-auth.session_token=<session-token-from-signup>"
```

Response:
```json
{
  "token": "eyJhbGciOiJIUzI1NiIs...",
  "user": { ... }
}
```

### 3. Use JWT with API Service

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIs..." \
  -d '{
    "message": "What is a Vision-Language-Action model?"
  }'
```

Response:
```json
{
  "response": "A Vision-Language-Action (VLA) model...",
  "sources": [...],
  "processing_time_ms": 1234
}
```

## Development Mode

### Run auth-service locally (without Docker):

```bash
cd auth/
npm install
npm run dev
```

Runs on `http://localhost:3001` with hot reload.

### Run api-service locally (without Docker):

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run server
uvicorn src.main:app --reload --port 8000
```

Runs on `http://localhost:8000` with hot reload.

## Docker Commands

### Start services in background:
```bash
docker compose up -d
```

### View logs:
```bash
docker compose logs -f
docker compose logs -f auth-service
docker compose logs -f api-service
```

### Stop services:
```bash
docker compose down
```

### Rebuild after code changes:
```bash
docker compose up --build
```

### Remove volumes (reset database):
```bash
docker compose down -v
```

## Troubleshooting

### Services can't communicate

**Problem**: api-service can't reach auth-service

**Solution**: Ensure both services are on the same Docker network (`ai-book-network`). Check `docker-compose.yml` network configuration.

### JWT verification fails

**Problem**: `Invalid JWT` errors in api-service

**Solution**:
1. Ensure `JWT_SECRET` is identical in both services
2. Check token expiry (tokens expire after 7 days)
3. Verify token format: `Authorization: Bearer <token>`

### Database connection errors

**Problem**: `database connection refused`

**Solution**:
- For Docker: Ensure `DATABASE_URL` uses the Docker service name (e.g., `postgres:5432`)
- For local dev: Use `localhost:5432`

### Port already in use

**Problem**: `port 3001 already allocated`

**Solution**:
```bash
# Find and kill process using port
lsof -ti:3001 | xargs kill -9
lsof -ti:8000 | xargs kill -9
```

## Production Deployment

For production deployment (Vercel, Railway, etc.):

1. **Deploy auth-service** separately (Vercel, Render, etc.)
2. **Deploy api-service** separately
3. **Update environment variables**:
   - Set `AUTH_SERVICE_URL` to your deployed auth service URL
   - Use production database URLs
   - Ensure `JWT_SECRET` matches across both services

## Security Notes

✅ **JWT tokens are stateless** - API service verifies them locally (no database calls)
✅ **Secrets are environment variables** - Never hardcoded
✅ **Separate runtimes** - Node.js auth logic never runs in Python process
✅ **CORS configured** - Only allowed origins can access APIs
✅ **Rate limiting** - 10 requests/minute per IP for auth endpoints

## Next Steps

- [ ] Add OAuth providers (Google, GitHub) to auth-service
- [ ] Implement refresh tokens
- [ ] Add API key authentication for server-to-server calls
- [ ] Set up monitoring and logging (Sentry, LogDNA)
