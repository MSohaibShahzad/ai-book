# Chapter Personalization - Deployment Checklist

**Feature**: Chapter Personalization (005-chapter-personalization)
**Last Updated**: 2025-12-24
**Branch**: `005-chapter-personalization`

## Pre-Deployment Checklist

### 1. Backend Setup

#### 1.1 Environment Variables

- [ ] Set `OPENAI_API_KEY` in backend environment
  ```bash
  # Production environment
  export OPENAI_API_KEY="sk-proj-..."

  # Or in .env file
  echo "OPENAI_API_KEY=sk-proj-..." >> backend/.env
  ```

- [ ] Verify API key has sufficient quota for expected usage
  - Estimated cost: ~$0.01-0.03 per personalization (GPT-4o)
  - Daily budget: 500 personalizations = ~$5-15/day

- [ ] Set optional observability variables (if using external monitoring)
  ```bash
  export PERSONALIZATION_TIMEOUT=30  # Timeout in seconds (default: 30)
  export PERSONALIZATION_MAX_RETRIES=0  # Retries on failure (default: 0)
  ```

#### 1.2 Database Migration

- [ ] Run personalization quota migration
  ```bash
  cd backend
  psql $DATABASE_URL < migrations/005_add_personalization_quota.sql
  ```

- [ ] Verify migration succeeded
  ```bash
  psql $DATABASE_URL -c "\d personalization_quota"
  ```

- [ ] Expected schema:
  ```sql
  Column                  | Type                     | Nullable
  ------------------------+--------------------------+---------
  user_id                 | uuid                     | not null (PK)
  request_count           | integer                  | not null (default 0, check 0-3)
  first_request_timestamp | timestamp with time zone | null
  reset_at                | timestamp with time zone | null
  created_at              | timestamp with time zone | not null (default now())
  updated_at              | timestamp with time zone | not null (default now())
  ```

#### 1.3 Dependencies

- [ ] Install Python dependencies
  ```bash
  cd backend
  pip install -r requirements.txt
  ```

- [ ] Verify critical packages installed:
  ```bash
  pip show openai-agents-python markdown-it-py mdit-py-plugins SQLAlchemy tenacity
  ```

#### 1.4 Better-Auth Integration

- [ ] Verify Better-Auth is configured and working
- [ ] Test session validation
  ```bash
  curl -X GET http://localhost:8000/api/personalization/quota \
    -H "Cookie: better-auth-session=<test-session-token>"
  ```

- [ ] Expected response: `{"remaining_requests": 3, "total_requests": 3, ...}`

### 2. Frontend Setup

#### 2.1 Dependencies

- [ ] Install frontend dependencies
  ```bash
  cd textbook
  npm install
  ```

- [ ] Verify React dependencies:
  ```bash
  npm list react react-dom react-markdown remark-gfm remark-math rehype-katex
  ```

#### 2.2 API Configuration

- [ ] Set backend API URL in frontend environment
  ```bash
  # Production
  echo "REACT_APP_API_URL=https://api.yourdomain.com/api" >> textbook/.env

  # Staging
  echo "REACT_APP_API_URL=https://staging-api.yourdomain.com/api" >> textbook/.env.staging
  ```

- [ ] Verify API URL in production build
  ```bash
  cd textbook
  npm run build
  grep -r "REACT_APP_API_URL" build/
  ```

#### 2.3 PersonalizationProvider Integration

- [ ] Wrap root component with `PersonalizationProvider`
  ```tsx
  // src/theme/Root.tsx
  import { PersonalizationProvider } from '@site/src/contexts/PersonalizationContext';

  export default function Root({ children }) {
    return (
      <PersonalizationProvider>
        {children}
      </PersonalizationProvider>
    );
  }
  ```

- [ ] Add `ProfileUpdateNotification` to layout
  ```tsx
  // src/theme/Layout/index.tsx
  import { ProfileUpdateNotification } from '@site/src/components/ProfileUpdateNotification';

  export default function Layout(props) {
    return (
      <>
        <ProfileUpdateNotification />
        {/* ... rest of layout */}
      </>
    );
  }
  ```

### 3. Testing

#### 3.1 Backend API Tests

- [ ] Test personalization endpoint
  ```bash
  curl -X POST http://localhost:8000/api/personalization/personalize \
    -H "Content-Type: application/json" \
    -H "Cookie: better-auth-session=<test-session>" \
    -d '{
      "chapter_id": "foundations-ros2/what-is-ros2",
      "chapter_content": "# What is ROS 2?\n\nROS 2 is a robotics framework...",
      "user_profile": {
        "softwareBackground": "Intermediate",
        "hardwareBackground": "Beginner",
        "interestArea": "AI"
      }
    }'
  ```

- [ ] Expected response: `{"personalized_content": "...", "remaining_limit": 2, "generation_time_ms": ...}`

- [ ] Test quota endpoint
  ```bash
  curl -X GET http://localhost:8000/api/personalization/quota \
    -H "Cookie: better-auth-session=<test-session>"
  ```

- [ ] Test cache invalidation
  ```bash
  curl -X DELETE http://localhost:8000/api/personalization/cache \
    -H "Cookie: better-auth-session=<test-session>"
  ```

- [ ] Expected response: `204 No Content`

- [ ] Test rate limiting (make 4 requests, verify 4th fails with 429)

- [ ] Test timeout handling (if possible, artificially delay agent response)

#### 3.2 Frontend Component Tests

- [ ] Test PersonalizeButton component
  - [ ] Shows for authenticated users with complete profiles
  - [ ] Hides for logged-out users
  - [ ] Shows "Complete profile" message for incomplete profiles
  - [ ] Displays loading state during personalization
  - [ ] Shows remaining limit after first use

- [ ] Test PersonalizedChapter component
  - [ ] Renders personalized content correctly
  - [ ] Toggle switches between original and personalized views
  - [ ] Scroll position preserved on toggle
  - [ ] Error messages display correctly

- [ ] Test RateLimitDisplay component
  - [ ] Shows correct remaining count
  - [ ] Displays countdown timer
  - [ ] Visual indicators (dots) update correctly

#### 3.3 End-to-End Testing

- [ ] **Full Flow Test**:
  1. Sign up as new user → ✅
  2. Complete profile with all fields → ✅
  3. Navigate to chapter → ✅
  4. Click "Personalize for Me" → ✅
  5. Verify personalized content generated → ✅
  6. Toggle to original view → ✅
  7. Toggle back to personalized → ✅
  8. Scroll position preserved → ✅
  9. Personalize 2 more chapters → ✅
  10. Verify limit reached (3/3) → ✅
  11. Try 4th personalization → ❌ (429 error expected)
  12. Update profile → ✅
  13. Verify cache cleared notification → ✅
  14. Re-personalize chapter → ✅

### 4. Performance & Monitoring

#### 4.1 Metrics Endpoint

- [ ] Verify metrics endpoint accessible
  ```bash
  curl -X GET http://localhost:8000/api/personalization/metrics
  ```

- [ ] Expected response:
  ```json
  {
    "total_requests": 0,
    "successful_requests": 0,
    "failed_requests": 0,
    "timeout_requests": 0,
    "avg_duration_ms": 0.0,
    "success_rate_percent": 0.0,
    "last_updated": null
  }
  ```

#### 4.2 Performance Targets

- [ ] Verify success rate >= 90% (SC-007 from spec)
  ```bash
  # Monitor over 24 hours
  watch -n 60 'curl -s http://localhost:8000/api/personalization/metrics | jq .success_rate_percent'
  ```

- [ ] Verify avg_duration_ms < 25000 (target: under 25 seconds)

- [ ] Verify timeout_requests < 10% of total

#### 4.3 Load Testing

- [ ] Simulate concurrent requests (50 users)
  ```bash
  # Using Apache Bench or similar
  ab -n 50 -c 10 -H "Cookie: better-auth-session=<test-session>" \
    -p test_payload.json \
    http://localhost:8000/api/personalization/personalize
  ```

- [ ] Monitor server resources (CPU, memory, DB connections)

### 5. Security Review

#### 5.1 Authentication

- [ ] Verify all endpoints require authentication
- [ ] Test with invalid session token → 401 expected
- [ ] Test with expired session → 401 expected

#### 5.2 Input Validation

- [ ] Test with missing profile fields → 400 expected
- [ ] Test with invalid chapter_id → handled gracefully
- [ ] Test with malformed markdown → handled gracefully

#### 5.3 API Key Security

- [ ] Verify `OPENAI_API_KEY` not exposed in frontend
- [ ] Verify API key not logged in server logs
- [ ] Verify API key not returned in error messages

### 6. Production Deployment

#### 6.1 Pre-Deployment

- [ ] Create backup of production database
  ```bash
  pg_dump $PRODUCTION_DB_URL > backup_pre_personalization_$(date +%Y%m%d).sql
  ```

- [ ] Tag release in git
  ```bash
  git tag -a v1.0.0-personalization -m "Release: Chapter Personalization"
  git push origin v1.0.0-personalization
  ```

- [ ] Notify team of deployment window

#### 6.2 Deployment Steps

1. [ ] Deploy backend
   ```bash
   # Pull latest code
   git pull origin 005-chapter-personalization

   # Run migration
   psql $DATABASE_URL < backend/migrations/005_add_personalization_quota.sql

   # Restart backend
   systemctl restart backend-api
   ```

2. [ ] Deploy frontend
   ```bash
   cd textbook
   npm run build
   # Deploy build/ to CDN or static hosting
   ```

3. [ ] Verify backend health
   ```bash
   curl http://api.yourdomain.com/health
   ```

4. [ ] Verify frontend deployed
   ```bash
   curl https://yourdomain.com
   ```

#### 6.3 Post-Deployment Verification

- [ ] Test personalization on production (use test account)
- [ ] Monitor metrics for 1 hour
  - [ ] Success rate >= 90%
  - [ ] No 500 errors
  - [ ] Avg response time < 25s

- [ ] Check error logs
  ```bash
  tail -f /var/log/backend-api/error.log | grep personalization
  ```

- [ ] Verify rate limiting working
  - [ ] Make 3 personalizations with test account
  - [ ] Verify 4th request fails with 429

### 7. Rollback Plan

#### 7.1 If Critical Issues Detected

- [ ] Rollback frontend
  ```bash
  # Restore previous build
  aws s3 sync s3://backup-bucket/previous-build/ s3://production-bucket/
  ```

- [ ] Rollback backend
  ```bash
  git checkout <previous-commit>
  systemctl restart backend-api
  ```

- [ ] Rollback database migration (if needed)
  ```bash
  psql $DATABASE_URL -c "DROP TABLE IF EXISTS personalization_quota;"
  ```

#### 7.2 Graceful Degradation

If OpenAI API is down:
- [ ] Frontend should show: "Personalization temporarily unavailable"
- [ ] Original content remains accessible
- [ ] Backend returns 503 with retry-after header

### 8. Monitoring & Alerts

#### 8.1 Set Up Alerts

- [ ] Alert if success_rate < 90% for 1 hour
- [ ] Alert if avg_duration_ms > 30000 for 5 minutes
- [ ] Alert if timeout_requests > 20% for 10 minutes
- [ ] Alert if OpenAI API key quota exceeded

#### 8.2 Dashboard Metrics

- [ ] Add to monitoring dashboard:
  - Total personalizations today
  - Success rate (last hour, last 24h)
  - Avg generation time
  - Timeout rate
  - Rate limit hits
  - Active users using personalization

### 9. Documentation

- [ ] Update API documentation with new endpoints
- [ ] Add personalization to user onboarding guide
- [ ] Create internal runbook for troubleshooting
- [ ] Document rollback procedures

### 10. Post-Launch

#### 10.1 Week 1

- [ ] Monitor daily metrics
- [ ] Collect user feedback
- [ ] Fix critical bugs
- [ ] Adjust rate limits if needed

#### 10.2 Month 1

- [ ] Analyze usage patterns
- [ ] Optimize prompt engineering based on feedback
- [ ] Consider increasing rate limits
- [ ] Plan Phase 2 features

---

## Quick Reference

### Environment Variables

```bash
# Backend
OPENAI_API_KEY=sk-proj-...
DATABASE_URL=postgresql://...
PERSONALIZATION_TIMEOUT=30
```

### Database Commands

```bash
# Run migration
psql $DATABASE_URL < backend/migrations/005_add_personalization_quota.sql

# Check quota table
psql $DATABASE_URL -c "SELECT * FROM personalization_quota LIMIT 10;"

# Reset user quota (for testing)
psql $DATABASE_URL -c "DELETE FROM personalization_quota WHERE user_id = '<user-uuid>';"
```

### API Endpoints

- `POST /api/personalization/personalize` - Generate personalized content
- `GET /api/personalization/quota` - Get quota status
- `DELETE /api/personalization/cache` - Invalidate cache
- `GET /api/personalization/metrics` - Public metrics (no auth)

### Support Contacts

- **Backend Issues**: backend-team@yourdomain.com
- **Frontend Issues**: frontend-team@yourdomain.com
- **OpenAI API Issues**: Contact OpenAI support
- **Database Issues**: dba-team@yourdomain.com

---

**Deployment Owner**: [Your Name]
**Deployment Date**: [YYYY-MM-DD]
**Sign-off**: [ ] QA, [ ] Product, [ ] Engineering Lead
