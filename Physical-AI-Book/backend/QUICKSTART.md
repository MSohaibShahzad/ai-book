# Quick Start - Docker Setup

## 🚀 Start Services in 3 Steps

### 1. Configure Environment

```bash
cd backend/
cp .env.example .env
# Edit .env with your credentials
```

### 2. Install Auth Dependencies

```bash
cd auth/
npm install
npm run db:push  # Run database migrations
cd ..
```

### 3. Start with Docker Compose

```bash
docker compose up --build
```

**Services will be available at:**
- 🔐 **Auth Service**: http://localhost:3001
- 🤖 **API Service**: http://localhost:8000

---

## 📝 Test the Services

### 1. Check Health

```bash
curl http://localhost:3001/health
curl http://localhost:8000/health
```

### 2. Sign Up

```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Password123",
    "name": "Test User"
  }'
```

### 3. Get JWT Token

```bash
curl http://localhost:3001/api/auth/jwt \
  -H "Cookie: better-auth.session_token=<token-from-step-2>"
```

### 4. Call API with JWT

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <jwt-from-step-3>" \
  -d '{"message": "What is a VLA model?"}'
```

---

## 🛑 Stop Services

```bash
docker compose down
```

---

## 📖 Full Documentation

See [DOCKER_SETUP.md](./DOCKER_SETUP.md) for detailed architecture, troubleshooting, and production deployment.
