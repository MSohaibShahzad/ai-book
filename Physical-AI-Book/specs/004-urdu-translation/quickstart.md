# Quickstart: Urdu Translation Feature

**Feature**: 004-urdu-translation
**Date**: 2025-12-20
**Status**: Development Guide

## Overview

This guide walks you through setting up and testing the Urdu translation feature locally.

**Estimated Setup Time**: 15 minutes

---

## Prerequisites

Ensure you have these installed and configured:

- ✅ Python 3.11+ (for backend)
- ✅ Node.js 18+ (for Docusaurus frontend)
- ✅ PostgreSQL (Neon or local instance)
- ✅ Git (for version control)
- ✅ OpenAI API key (with GPT-4o access)

---

## Step 1: Environment Setup

### 1.1 Clone Repository

```bash
cd /home/sohaib/hackathon/ai-book/Physical-AI-Book
git checkout 004-urdu-translation
```

### 1.2 Backend Environment Configuration

Create or update `backend/.env`:

```bash
cd backend
cp .env.example .env
```

Add the following to `backend/.env`:

```env
# OpenAI Configuration (NEW)
OPENAI_API_KEY=sk-proj-...your-key-here...
TRANSLATION_MODEL=gpt-4o
MAX_TRANSLATION_TOKENS=128000
RATE_LIMIT_PER_MINUTE=10

# Database Configuration (Existing)
DATABASE_URL=postgresql://user:password@localhost:5432/physical_ai_book

# Auth Configuration (Existing)
JWT_SECRET=your-jwt-secret
BETTER_AUTH_SECRET=your-better-auth-secret
```

**Important**: Never commit `.env` to version control. Add to `.gitignore` if not already present.

### 1.3 Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**New dependencies added for translation feature**:
- `openai-agents` - OpenAI Agents SDK
- `markdown-it-py` - Markdown parsing
- `mdit-py-plugins` - Markdown plugins (frontmatter, math)
- `tenacity` - Retry logic with exponential backoff
- `tiktoken` - Token counting for cost estimation

### 1.4 Frontend Environment Configuration

```bash
cd ../textbook
npm install
```

No environment changes needed for frontend (API calls authenticated via session cookies).

---

## Step 2: Database Migration

### 2.1 Run Migration Script

```bash
cd backend

# Apply translation tables migration
alembic upgrade head
```

This creates:
- `translation_cache` table (caching translations)
- `translation_log` table (analytics tracking)

### 2.2 Verify Tables

```bash
psql $DATABASE_URL -c "\dt translation*"
```

Expected output:
```
                 List of relations
 Schema |        Name         | Type  |  Owner
--------+---------------------+-------+---------
 public | translation_cache   | table | postgres
 public | translation_log     | table | postgres
```

---

## Step 3: Start Services

### 3.1 Start Backend API

```bash
cd backend

# Development mode with hot reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete.
```

Verify backend is running:
```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy"}
```

### 3.2 Start Frontend (Docusaurus)

```bash
cd textbook

# Development mode
npm run start
```

Expected output:
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

---

## Step 4: Test Translation Feature

### 4.1 Sign In (Required for Translation)

1. Open browser: http://localhost:3000
2. Click "Sign In" (top right)
3. Enter credentials or create account
4. Verify you're logged in (username appears in header)

### 4.2 Translate a Chapter

1. Navigate to any chapter (e.g., "Introduction to ROS 2")
2. Scroll to top of chapter content
3. Click **"Translate to Urdu"** button
4. Wait for translation (2-30 seconds depending on chapter length)
5. Verify Urdu content appears with right-to-left (RTL) formatting
6. Code blocks and formulas should remain in English/original format

### 4.3 Toggle Between Languages

1. After translation loads, click **"Show Original English"** button
2. Verify content switches back to English instantly (no reload)
3. Click **"Show Urdu Translation"** again
4. Verify it switches back to Urdu (should be instant if cached)

### 4.4 Test Unauthenticated Access

1. Sign out (top right)
2. Navigate to a chapter
3. Verify "Translate to Urdu" button is disabled or hidden
4. Tooltip should show: "Please log in to use Urdu translation"

---

## Step 5: Verify Caching

### 5.1 Check Database Cache

After translating a chapter, verify it's cached:

```bash
psql $DATABASE_URL -c "SELECT chapter_slug, language, created_at FROM translation_cache LIMIT 5;"
```

Expected output:
```
   chapter_slug    | language |        created_at
-------------------+----------+----------------------------
 01-what-is-ros2   | ur       | 2025-12-20 10:30:15.123456
```

### 5.2 Test Cache Hit

1. Translate a chapter (cache miss, ~5-30 seconds)
2. Refresh page
3. Translate same chapter again (cache hit, < 1 second)
4. Verify response includes `from_cache: true` in browser dev tools (Network tab → Response)

### 5.3 View Analytics Logs

```bash
psql $DATABASE_URL -c "SELECT user_id, chapter_slug, action, timestamp FROM translation_log ORDER BY timestamp DESC LIMIT 10;"
```

Expected output:
```
 user_id |   chapter_slug    |       action        |        timestamp
---------+-------------------+---------------------+----------------------------
       1 | 01-what-is-ros2   | translate_requested | 2025-12-20 10:30:15.456789
       1 | 01-what-is-ros2   | toggle_to_english   | 2025-12-20 10:31:00.123456
```

---

## Step 6: Test API Directly (Optional)

### 6.1 Get Authentication Token

```bash
# Sign in via API
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123"
  }'

# Extract JWT token from response
# Example response: {"token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."}
```

### 6.2 Call Translation API

```bash
curl -X POST http://localhost:8000/api/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN_HERE" \
  -d '{
    "chapter_slug": "01-what-is-ros2"
  }'
```

Expected response:
```json
{
  "status": "success",
  "translated_content": "# ROS 2 کیا ہے؟\n\nROS 2 روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے...",
  "from_cache": false,
  "cached_at": null
}
```

### 6.3 Test Error Handling

**Unauthenticated Request**:
```bash
curl -X POST http://localhost:8000/api/translate \
  -H "Content-Type: application/json" \
  -d '{"chapter_slug": "01-what-is-ros2"}'
```

Expected response (401):
```json
{
  "status": "error",
  "message": "Authentication required. Please log in to use Urdu translation."
}
```

**Invalid Chapter**:
```bash
curl -X POST http://localhost:8000/api/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -d '{"chapter_slug": "nonexistent-chapter"}'
```

Expected response (400):
```json
{
  "status": "error",
  "message": "Chapter not found: nonexistent-chapter"
}
```

---

## Step 7: Monitor Costs

### 7.1 Enable Cost Tracking

Add to `backend/src/services/openai_agent_service.py`:

```python
import tiktoken

def estimate_cost(input_text: str, output_text: str, model: str = "gpt-4o"):
    encoding = tiktoken.encoding_for_model(model)
    input_tokens = len(encoding.encode(input_text))
    output_tokens = len(encoding.encode(output_text))

    if model == "gpt-4o":
        input_cost = (input_tokens / 1_000_000) * 2.50
        output_cost = (output_tokens / 1_000_000) * 10.00
    elif model == "gpt-4o-mini":
        input_cost = (input_tokens / 1_000_000) * 0.150
        output_cost = (output_tokens / 1_000_000) * 0.600

    total_cost = input_cost + output_cost
    print(f"[COST] {model}: {input_tokens} in + {output_tokens} out = ${total_cost:.4f}")
    return total_cost
```

### 7.2 View Cost Logs

Check backend logs for cost estimates:

```bash
tail -f backend/logs/app.log | grep COST
```

Example output:
```
[COST] gpt-4o: 6500 in + 7800 out = $0.0943
```

---

## Troubleshooting

### Issue: Translation Times Out

**Symptoms**: Request hangs for 30+ seconds, then fails

**Solutions**:
1. Check OpenAI API status: https://status.openai.com
2. Verify API key has GPT-4o access (not all keys do)
3. Reduce chapter size (test with shorter chapters first)
4. Increase timeout in `backend/src/config.py`:
   ```python
   TRANSLATION_TIMEOUT = 60  # seconds
   ```

### Issue: Cache Not Working

**Symptoms**: Same chapter takes 5+ seconds every time

**Solutions**:
1. Verify `translation_cache` table exists:
   ```bash
   psql $DATABASE_URL -c "\d translation_cache"
   ```
2. Check cache entries:
   ```bash
   psql $DATABASE_URL -c "SELECT COUNT(*) FROM translation_cache;"
   ```
3. Verify `content_hash` computation is consistent

### Issue: Urdu Text Displays as Boxes/Gibberish

**Symptoms**: ▯▯▯ or random characters instead of Urdu

**Solutions**:
1. Verify browser supports UTF-8 (all modern browsers do)
2. Check font loading in browser dev tools (Network tab → Fonts)
3. Ensure `Noto Nastaliq Urdu` font is loading from Google Fonts
4. Verify `dir="rtl"` attribute on Urdu content container

### Issue: "Translate to Urdu" Button Not Appearing

**Symptoms**: No button visible on chapter pages

**Solutions**:
1. Verify you're logged in (check browser session cookies)
2. Clear browser cache and reload
3. Check browser console for JavaScript errors
4. Verify `TranslationButton` component is rendered in Docusaurus theme

### Issue: High Translation Costs

**Symptoms**: OpenAI bills higher than expected

**Solutions**:
1. Verify caching is working (check cache hit rate)
2. Use `gpt-4o-mini` for development/testing:
   ```env
   TRANSLATION_MODEL=gpt-4o-mini
   ```
3. Implement rate limiting per user:
   ```python
   MAX_TRANSLATIONS_PER_DAY = 10
   ```
4. Add cost alerts in OpenAI dashboard

---

## Running Tests

### Backend Tests

```bash
cd backend
pytest tests/unit/test_translation_service.py -v
pytest tests/integration/test_translation_api.py -v
```

### Frontend Tests

```bash
cd textbook
npm run test -- TranslationButton
```

---

## Development Workflow

### Making Changes

1. **Backend changes** (API, translation logic):
   - Edit files in `backend/src/`
   - Uvicorn hot-reloads automatically
   - Test via `curl` or Postman

2. **Frontend changes** (UI, button):
   - Edit files in `textbook/src/components/TranslationButton/`
   - Webpack hot-reloads automatically
   - Test in browser at http://localhost:3000

3. **Database changes** (schema):
   - Create Alembic migration: `alembic revision --autogenerate -m "description"`
   - Review migration file in `backend/alembic/versions/`
   - Apply: `alembic upgrade head`

### Debugging Tips

1. **Backend logs**: `tail -f backend/logs/app.log`
2. **Frontend console**: Open browser DevTools → Console tab
3. **Network requests**: DevTools → Network tab → Filter "translate"
4. **Database queries**: Add `echo=True` to SQLAlchemy engine config

---

## Next Steps

After verifying the feature works locally:

1. **Code review**: Create PR with `/sp.git.commit_pr` command
2. **Testing**: Run full test suite with `/sp.tasks` generated tests
3. **Documentation**: Update user-facing docs (if needed)
4. **Deployment**: Deploy to staging, then production
5. **Monitoring**: Set up cost alerts and usage dashboards

---

## Useful Commands Cheat Sheet

```bash
# Start backend
cd backend && uvicorn src.main:app --reload

# Start frontend
cd textbook && npm run start

# Apply migrations
cd backend && alembic upgrade head

# Check cache
psql $DATABASE_URL -c "SELECT * FROM translation_cache;"

# View logs
psql $DATABASE_URL -c "SELECT * FROM translation_log ORDER BY timestamp DESC LIMIT 20;"

# Run tests
cd backend && pytest
cd textbook && npm test

# Invalidate cache (admin)
curl -X DELETE http://localhost:8000/api/translate/cache/01-what-is-ros2 \
  -H "Authorization: Bearer YOUR_ADMIN_TOKEN"
```

---

## Support

- **Documentation**: `/specs/004-urdu-translation/`
- **API Spec**: `/specs/004-urdu-translation/contracts/translation-api.yaml`
- **Research**: `/specs/004-urdu-translation/research.md`
- **Data Model**: `/specs/004-urdu-translation/data-model.md`

---

**Quickstart Status**: ✅ Complete - Ready for development
