# Environment Variables Setup

## ✅ Configuration Complete

Both `.env` files have been created and configured correctly.

---

## 📂 File Locations

```
backend/
├── .env              ✅ Main backend (API service)
└── auth/.env         ✅ Auth service
```

---

## 🔐 Auth Service (.env)

**Location:** `backend/auth/.env`

```bash
# Database
DATABASE_URL=postgresql://...

# Secrets
BETTER_AUTH_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41
JWT_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41

# Server
BETTER_AUTH_URL=http://localhost:3001
AUTH_SERVER_PORT=3001
AUTH_SERVER_HOST=0.0.0.0
NODE_ENV=development
```

**Purpose:**
- Handles authentication
- Issues JWT tokens
- Manages user sessions

---

## 🤖 API Service (.env)

**Location:** `backend/.env`

```bash
# Qdrant Cloud (Vector Database)
QDRANT_URL=https://...
QDRANT_API_KEY=...

# Database
DATABASE_URL=postgresql://...

# OpenAI
OPENAI_API_KEY=sk-proj-...

# Secrets (MUST MATCH auth/.env!)
BETTER_AUTH_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41
JWT_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41

# Auth Service URLs
BETTER_AUTH_URL=http://localhost:3001
AUTH_SERVICE_URL=http://auth-service:3001  # Docker internal
```

**Purpose:**
- RAG chatbot
- JWT verification
- Vector search
- LLM generation

---

## ⚠️ CRITICAL: JWT_SECRET

**The `JWT_SECRET` MUST be identical in both files!**

```bash
# backend/auth/.env
JWT_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41

# backend/.env
JWT_SECRET=d9464a4e40689d845f892d3a9f906ee1040f76c74a4303303a1416ef585f1c41
```

**Why?**
- Auth service **signs** JWT with this secret
- API service **verifies** JWT with this secret
- If they don't match → JWT verification fails → users can't authenticate

**Verification:**
```bash
cd backend/
grep "^JWT_SECRET=" .env
grep "^JWT_SECRET=" auth/.env
# Should return identical values
```

---

## 🚀 Usage

### Local Development

```bash
cd backend/

# Option 1: Docker Compose (recommended)
docker compose up --build

# Option 2: Run services separately
# Terminal 1 - Auth service
cd auth/
npm install
npm run dev

# Terminal 2 - API service
source venv/bin/activate
uvicorn src.main:app --reload --port 8000
```

### Production

Update these variables for production:

**backend/auth/.env:**
```bash
BETTER_AUTH_URL=https://your-auth.vercel.app
NODE_ENV=production
```

**backend/.env:**
```bash
BETTER_AUTH_URL=https://your-auth.vercel.app
AUTH_SERVICE_URL=https://your-auth.vercel.app
ENVIRONMENT=production
```

---

## 🔒 Security Notes

### ⚠️ Never Commit `.env` Files

Both `.env` files are git-ignored:
```bash
# .gitignore
.env
.env.local
auth/.env
```

### 🔑 Generate New Secrets for Production

**Current secrets are for development only!**

Generate new secrets:
```bash
# Generate BETTER_AUTH_SECRET
openssl rand -hex 32

# Generate JWT_SECRET (different from above)
openssl rand -hex 32
```

Then update both `.env` files with the new values.

---

## 📋 Environment Variables Reference

### Auth Service Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `DATABASE_URL` | ✅ Yes | - | PostgreSQL connection string |
| `BETTER_AUTH_SECRET` | ✅ Yes | - | Better Auth encryption key |
| `JWT_SECRET` | ✅ Yes | - | JWT signing secret |
| `BETTER_AUTH_URL` | ✅ Yes | `http://localhost:3001` | Public auth service URL |
| `AUTH_SERVER_PORT` | No | `3001` | Port to run on |
| `AUTH_SERVER_HOST` | No | `0.0.0.0` | Host to bind to |
| `NODE_ENV` | No | `development` | Environment mode |

### API Service Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `QDRANT_URL` | ✅ Yes | - | Qdrant Cloud URL |
| `QDRANT_API_KEY` | ✅ Yes | - | Qdrant API key |
| `DATABASE_URL` | ✅ Yes | - | PostgreSQL connection string |
| `OPENAI_API_KEY` | ✅ Yes | - | OpenAI API key |
| `JWT_SECRET` | ✅ Yes | - | JWT verification secret |
| `BETTER_AUTH_SECRET` | ✅ Yes | - | Fallback auth secret |
| `AUTH_SERVICE_URL` | No | `http://auth-service:3001` | Internal auth service URL (Docker) |
| `EMBEDDING_MODEL` | No | `text-embedding-3-small` | OpenAI embedding model |
| `LLM_MODEL` | No | `gpt-4o-mini` | OpenAI LLM model |

---

## ✅ Verification Checklist

- [x] `backend/.env` exists
- [x] `backend/auth/.env` exists
- [x] `JWT_SECRET` matches in both files
- [x] `DATABASE_URL` is set
- [x] `OPENAI_API_KEY` is set
- [x] `QDRANT_URL` and `QDRANT_API_KEY` are set
- [x] Both `.env` files are git-ignored

**Status: All configurations verified! ✅**

---

## 🆘 Troubleshooting

### Issue: "Invalid JWT" errors

**Cause:** `JWT_SECRET` mismatch between services

**Solution:**
```bash
# Check if they match
cd backend/
diff <(grep "^JWT_SECRET=" .env) <(grep "^JWT_SECRET=" auth/.env)

# If different, update one to match the other
```

### Issue: "Auth server unavailable"

**Cause:** API service can't reach auth service

**Solution:**
- Local dev: Use `BETTER_AUTH_URL=http://localhost:3001`
- Docker: Use `AUTH_SERVICE_URL=http://auth-service:3001`

### Issue: "Database connection refused"

**Cause:** Invalid `DATABASE_URL`

**Solution:**
- Check connection string format
- Ensure database is accessible
- Verify credentials are correct

---

## 📝 Summary

✅ **Auth service** has its own `.env` with:
- Database URL
- Better Auth secret
- JWT secret (for signing)
- Server configuration

✅ **API service** has its own `.env` with:
- Database URL
- OpenAI credentials
- Qdrant credentials
- JWT secret (for verification - MUST match auth!)
- Auth service URLs

✅ **JWT_SECRET** matches in both files (verified)

**Your backend is now properly configured!** 🎉
