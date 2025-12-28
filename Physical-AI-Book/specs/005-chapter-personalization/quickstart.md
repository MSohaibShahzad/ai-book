# Quickstart: Chapter Personalization

**Feature**: Chapter Personalization
**Date**: 2025-12-23
**Purpose**: Get the chapter personalization feature running locally for development and testing

## Prerequisites

Before starting, ensure you have:

- ✅ **Node.js 18+** and **npm** installed
- ✅ **Python 3.11+** installed
- ✅ **PostgreSQL** database access (Neon or local)
- ✅ **OpenAI API key** with GPT-4o access
- ✅ **Better-Auth** already configured (feature 003-auth-signup-signin)
- ✅ **Git** repository cloned and on branch `005-chapter-personalization`

## Architecture Overview

```
┌─────────────┐          ┌──────────────┐          ┌───────────────┐
│  Docusaurus │  HTTP    │    FastAPI   │   API    │  OpenAI Agent │
│  Frontend   │ ────────>│   Backend    │ ────────>│  (GPT-4o)     │
│             │  JSON    │              │  Prompt  │               │
└─────────────┘          └──────────────┘          └───────────────┘
      │                        │
      │ Session State          │ PostgreSQL
      │ (React Context)        ▼
      │                  ┌───────────────┐
      │                  │ Personalization│
      └──────────────────│ Quota (DB)    │
         Cached Content  └───────────────┘
```

## Setup Steps

### 1. Backend Setup

#### 1.1 Install Python Dependencies

```bash
cd backend/
pip install openai-agents-python markdown-it-py tenacity
pip install -r requirements.txt  # Existing dependencies
```

**New Dependencies**:
- `openai-agents-python`: OpenAI Agents SDK for reusable personalization agent
- `markdown-it-py`: Markdown parsing with AST support (already installed for 004-urdu-translation)
- `tenacity`: Retry logic with exponential backoff (optional but recommended)

#### 1.2 Configure Environment Variables

Add to `backend/.env`:

```bash
# OpenAI Configuration (for personalization agent)
OPENAI_API_KEY=sk-proj-...your-key...

# PostgreSQL Connection (should already exist)
DATABASE_URL=postgresql://user:password@host:5432/dbname

# Better-Auth Configuration (should already exist)
BETTER_AUTH_SECRET=your-secret
BETTER_AUTH_URL=http://localhost:3001
```

**Security Note**: NEVER commit `.env` file. Keep `OPENAI_API_KEY` secret.

#### 1.3 Run Database Migration

Create the `personalization_quota` table:

```bash
# Using Alembic (if set up)
cd backend/
alembic upgrade head

# OR manually execute SQL
psql $DATABASE_URL < migrations/005_add_personalization_quota.sql
```

**Migration File** (`migrations/005_add_personalization_quota.sql`):
```sql
CREATE TABLE IF NOT EXISTS personalization_quota (
    user_id UUID PRIMARY KEY,
    request_count INTEGER NOT NULL DEFAULT 0,
    first_request_timestamp TIMESTAMPTZ NOT NULL,
    reset_at TIMESTAMPTZ NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT fk_user FOREIGN KEY (user_id)
        REFERENCES users(id)
        ON DELETE CASCADE,

    CONSTRAINT chk_request_count
        CHECK (request_count >= 0 AND request_count <= 3)
);

CREATE INDEX idx_personalization_quota_reset ON personalization_quota(reset_at);
CREATE INDEX idx_personalization_quota_user ON personalization_quota(user_id);
```

#### 1.4 Verify Migration

```bash
psql $DATABASE_URL -c "\d personalization_quota"
```

Expected output:
```
Table "public.personalization_quota"
        Column            |           Type           | Nullable | Default
--------------------------+--------------------------+----------+---------
 user_id                  | uuid                     | not null |
 request_count            | integer                  | not null | 0
 first_request_timestamp  | timestamp with time zone | not null |
 reset_at                 | timestamp with time zone | not null |
 created_at               | timestamp with time zone | not null | now()
 updated_at               | timestamp with time zone | not null | now()
Indexes:
    "personalization_quota_pkey" PRIMARY KEY, btree (user_id)
    "idx_personalization_quota_reset" btree (reset_at)
    "idx_personalization_quota_user" btree (user_id)
Check constraints:
    "chk_request_count" CHECK (request_count >= 0 AND request_count <= 3)
Foreign-key constraints:
    "fk_user" FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
```

#### 1.5 Start Backend Server

```bash
cd backend/
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Backend should start on `http://localhost:8000`

**Verify Backend**:
```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy"}

curl http://localhost:8000/metrics
# Expected: {"total_requests": 0, "success_rate_percent": 0, ...}
```

---

### 2. Frontend Setup

#### 2.1 Install Node Dependencies

```bash
cd textbook/
npm install
```

**Note**: No new dependencies needed. Feature uses existing React, TypeScript, and Docusaurus packages.

#### 2.2 Configure API Endpoint

Update `textbook/src/services/personalizationService.ts`:

```typescript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api';
```

For production, set environment variable:
```bash
export REACT_APP_API_URL=https://ai-book-api.vercel.app/api
```

#### 2.3 Start Docusaurus Dev Server

```bash
cd textbook/
npm run start
```

Frontend should start on `http://localhost:3000`

**Verify Frontend**:
- Navigate to `http://localhost:3000`
- Sign in with test account (or sign up)
- Navigate to any chapter
- Verify "Personalize for Me" button appears (if profile complete)

---

### 3. Testing the Feature

#### 3.1 Create Test User

```bash
# Sign up via UI at http://localhost:3000/signup
# OR use curl:
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPassword123!",
    "name": "Test User",
    "softwareBackground": "Beginner",
    "hardwareBackground": "None",
    "interestArea": "AI"
  }'
```

#### 3.2 Test Personalization Flow

**Step 1**: Sign in and complete profile
```
1. Navigate to http://localhost:3000/signin
2. Sign in with test credentials
3. Go to Profile → Settings
4. Ensure software_background, hardware_background, interest_area are filled
```

**Step 2**: Personalize a chapter
```
1. Navigate to any chapter (e.g., /docs/foundations-ros2/nodes-topics)
2. Click "Personalize for Me" button at top of chapter
3. Wait for personalization (should complete in < 30 seconds)
4. Verify personalized content displays with "Personalized for [Your Name]" badge
5. Verify remaining limit shows "Remaining limit: 2"
```

**Step 3**: Toggle between views
```
1. Click "View Original" toggle
2. Verify original chapter content displays
3. Click "View Personalized" toggle
4. Verify personalized content displays instantly (no re-generation)
```

**Step 4**: Test rate limiting
```
1. Personalize 3 different chapters (uses all 3 daily requests)
2. Attempt to personalize a 4th chapter
3. Verify "Limit exceeded" message appears
4. Check profile page - should show "0 requests remaining"
```

**Step 5**: Test timeout (optional)
```
1. Edit backend/personalization/agent.py to add artificial delay:
   time.sleep(35)  # Force timeout
2. Attempt personalization
3. Verify timeout error after 30 seconds
4. Verify retry button appears
5. Verify remaining limit decreased despite timeout
```

#### 3.3 Test Profile Update Cache Invalidation

```
1. Personalize a chapter (e.g., Beginner level content)
2. Toggle to personalized view
3. Go to Profile → Settings
4. Change software_background from "Beginner" to "Advanced"
5. Save profile
6. Return to chapter
7. Verify notification: "Profile updated. Re-personalize your chapters for adapted content."
8. Toggle to personalized view → should show original (cache cleared)
9. Click "Personalize for Me" again
10. Verify new personalized content reflects Advanced level
```

---

### 4. Monitoring & Debugging

#### 4.1 Check Logs

**Backend Logs** (structured JSON):
```bash
cd backend/
tail -f logs/personalization.log | jq .
```

Expected log format:
```json
{
  "event": "personalization_request",
  "request_id": "uuid",
  "user_id": "uuid",
  "chapter_id": "01-foundations-ros2/02-nodes-topics",
  "status": "success",
  "duration_ms": 15342,
  "timestamp": "2025-12-23T10:30:45.123Z"
}
```

**Frontend Logs** (browser console):
```
[Personalization] Request started for chapter: 01-foundations-ros2/02-nodes-topics
[Personalization] Response received in 15.3s
[Personalization] Cache updated, remaining limit: 2
```

#### 4.2 Query Metrics

```bash
curl http://localhost:8000/metrics | jq .
```

Expected output:
```json
{
  "total_requests": 5,
  "successful_requests": 4,
  "failed_requests": 1,
  "timeout_requests": 0,
  "avg_duration_ms": 18234.5,
  "success_rate_percent": 80.0,
  "last_updated": "2025-12-23T15:30:00Z"
}
```

**Alert if**: `success_rate_percent < 90` (violates SC-007 requirement)

#### 4.3 Query Database

**Check quota status**:
```sql
SELECT
    u.email,
    pq.request_count,
    pq.reset_at,
    (3 - pq.request_count) AS remaining
FROM personalization_quota pq
JOIN users u ON pq.user_id = u.id
ORDER BY pq.updated_at DESC
LIMIT 10;
```

**Check expired quotas**:
```sql
SELECT COUNT(*)
FROM personalization_quota
WHERE reset_at < NOW();
```

---

### 5. Common Issues & Troubleshooting

#### Issue 1: "OPENAI_API_KEY not found"

**Symptom**: Backend fails to start or returns 500 errors on personalization requests

**Solution**:
```bash
# Add to backend/.env
OPENAI_API_KEY=sk-proj-your-key-here

# Restart backend
uvicorn main:app --reload
```

#### Issue 2: "Personalize for Me" button not visible

**Symptom**: Button doesn't appear on chapter pages

**Possible Causes**:
- User not logged in → Sign in first
- Profile incomplete → Complete software_background, hardware_background, interest_area
- Frontend not detecting auth session → Check browser cookies for `better-auth-session`

**Solution**:
```bash
# Check auth session
curl http://localhost:3001/api/auth/session -H "Cookie: better-auth-session=..."

# Verify profile completeness
curl http://localhost:8000/api/personalization/quota -H "Authorization: Bearer <jwt>"
```

#### Issue 3: Personalization times out

**Symptom**: Request fails after 30 seconds with timeout error

**Possible Causes**:
- Chapter too long (>10,000 words) → AI takes longer to process
- OpenAI API slow/rate limited → Check OpenAI dashboard
- Network latency → Check backend→OpenAI connectivity

**Solution**:
```bash
# Test OpenAI API directly
python -c "
from openai import OpenAI
client = OpenAI(api_key='sk-proj-...')
response = client.chat.completions.create(
    model='gpt-4o',
    messages=[{'role': 'user', 'content': 'Hello'}]
)
print(response.choices[0].message.content)
"

# If successful, issue is with markdown processing or agent config
# Check backend/personalization/agent.py configuration
```

#### Issue 4: Rate limit not resetting

**Symptom**: User stuck at 0 requests even after 24 hours

**Possible Causes**:
- `reset_at` timestamp not updating correctly
- Database clock vs server clock mismatch

**Solution**:
```sql
-- Manually reset quota for testing
UPDATE personalization_quota
SET request_count = 0,
    first_request_timestamp = NOW(),
    reset_at = NOW() + INTERVAL '24 hours'
WHERE user_id = '<user-uuid>';

-- Check server time vs database time
SELECT NOW() AS db_time, CURRENT_TIMESTAMP AS server_time;
```

#### Issue 5: Cached content not invalidating on profile update

**Symptom**: User updates profile but still sees old personalized content

**Possible Causes**:
- Frontend not calling `/api/personalization/cache` DELETE endpoint
- React Context not clearing state

**Solution**:
```typescript
// In PersonalizationContext.tsx, add console log
const clearCache = useCallback(() => {
  console.log('[Personalization] Cache cleared:', cachedContent.size);
  setCachedContent(new Map());
  setCurrentView('original');
}, [cachedContent]);

// Verify clearCache is called on profile update
// Check browser console for log message
```

---

### 6. Running Tests

#### 6.1 Backend Tests

```bash
cd backend/
pytest tests/test_personalization_agent.py -v
pytest tests/test_personalization_api.py -v
pytest tests/test_rate_limiter.py -v
```

**Key Test Cases**:
- ✅ Agent preserves code blocks, LaTeX, frontmatter
- ✅ Rate limiting enforces 3 requests/day limit
- ✅ Timeout triggers after 30 seconds
- ✅ Profile update invalidates cache
- ✅ Quota resets after 24 hours

#### 6.2 Frontend Tests

```bash
cd textbook/
npm test -- PersonalizeButton
npm test -- PersonalizedChapter
npm test -- usePersonalization
```

**Key Test Cases**:
- ✅ Button visibility based on auth and profile completeness
- ✅ Toggle functionality switches views instantly
- ✅ Rate limit display updates correctly
- ✅ Error messages display on timeout/failure

---

### 7. Deployment Checklist

Before deploying to production:

- [ ] Set `OPENAI_API_KEY` in production environment variables (Vercel/AWS/etc.)
- [ ] Run database migration on production database
- [ ] Update `REACT_APP_API_URL` to production backend URL
- [ ] Verify Better-Auth trustedOrigins includes production frontend URL
- [ ] Configure monitoring alerts for `success_rate_percent < 90`
- [ ] Test rate limiting with production database
- [ ] Verify CORS configuration allows production frontend
- [ ] Set up log aggregation (CloudWatch, Datadog, etc.)
- [ ] Configure backup/restore for `personalization_quota` table

---

## Next Steps

1. **Generate Tasks**: Run `/sp.tasks` to create detailed implementation tasks
2. **Start Implementation**: Begin with backend personalization agent (`backend/personalization/agent.py`)
3. **Iterate**: Implement, test, and refine based on acceptance criteria in `spec.md`

## Reference Documentation

- **Spec**: `specs/005-chapter-personalization/spec.md`
- **Plan**: `specs/005-chapter-personalization/plan.md`
- **Data Model**: `specs/005-chapter-personalization/data-model.md`
- **API Contract**: `specs/005-chapter-personalization/contracts/personalization-api.yaml`
- **Research**: `specs/005-chapter-personalization/research.md`

## Support

For issues or questions:
- Check `specs/005-chapter-personalization/plan.md` → Troubleshooting section
- Review structured logs in `backend/logs/personalization.log`
- Query metrics endpoint: `http://localhost:8000/metrics`
- Create issue in GitHub repository with logs and reproduction steps
