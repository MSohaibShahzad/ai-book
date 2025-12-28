# Data Model: Chapter Personalization

**Feature**: Chapter Personalization
**Date**: 2025-12-23
**Purpose**: Define all data entities, relationships, storage strategies, and validation rules for the personalization system

## Overview

The Chapter Personalization feature uses a hybrid storage approach:
- **PostgreSQL (Neon)**: Rate limiting quota tracking (persistent)
- **Frontend Session State**: Cached personalized content (temporary, cleared on session end)
- **Logs Only**: Request metadata for observability (not queryable entities)

## Entity Diagram

```
┌─────────────────┐
│     users       │ (Existing table from 003-auth)
│─────────────────│
│ id (UUID) PK    │
│ email           │
│ software_bg     │──┐
│ hardware_bg     │  │ Referenced by quota
│ interest_area   │  │
└─────────────────┘  │
                     │
                     │
                     ▼
        ┌─────────────────────────┐
        │ personalization_quota   │ (New table)
        │─────────────────────────│
        │ user_id (UUID) PK,FK    │
        │ request_count (0-3)     │
        │ first_request_timestamp │
        │ reset_at                │
        │ created_at              │
        │ updated_at              │
        └─────────────────────────┘

┌────────────────────────────────┐
│  PersonalizedContent           │ (Frontend State - Not Persisted)
│────────────────────────────────│
│ chapterId: string              │
│ originalMarkdown: string       │
│ personalizedMarkdown: string   │
│ generatedAt: Date              │
│ profileSnapshot: UserProfile   │
└────────────────────────────────┘

┌────────────────────────────────┐
│  PersonalizationRequest        │ (Logs Only - Not Stored)
│────────────────────────────────│
│ requestId: UUID                │
│ userId: UUID                   │
│ chapterId: string              │
│ status: enum                   │
│ durationMs: number             │
│ errorMessage: string?          │
│ timestamp: Date                │
└────────────────────────────────┘
```

## Entity Definitions

### 1. PersonalizationQuota (PostgreSQL)

**Purpose**: Track rate limiting for personalization requests per user.

**Table Schema**:
```sql
CREATE TABLE personalization_quota (
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

CREATE INDEX idx_personalization_quota_reset
    ON personalization_quota(reset_at);

CREATE INDEX idx_personalization_quota_user
    ON personalization_quota(user_id);
```

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | PRIMARY KEY, NOT NULL, FOREIGN KEY → users(id) | User who made personalization requests |
| request_count | INTEGER | NOT NULL, DEFAULT 0, CHECK (0-3) | Number of requests made in current 24-hour window |
| first_request_timestamp | TIMESTAMPTZ | NOT NULL | When the first request of current window was made |
| reset_at | TIMESTAMPTZ | NOT NULL | When the quota resets (first_request_timestamp + 24 hours) |
| created_at | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | When this quota entry was created |
| updated_at | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last time this quota was updated |

**Lifecycle**:
1. **Creation**: Row created on user's first personalization request
2. **Increment**: `request_count` incremented on each request (max 3)
3. **Reset**: When `NOW() > reset_at`, reset `request_count` to 1 and update `first_request_timestamp` and `reset_at`
4. **Deletion**: Cascade deleted when user is deleted

**Validation Rules**:
- `request_count` must be between 0 and 3 (enforced by CHECK constraint)
- `reset_at` must be exactly `first_request_timestamp + 24 hours`
- `user_id` must exist in `users` table

**Python Model (SQLAlchemy)**:
```python
from sqlalchemy import Column, Integer, DateTime, UUID, CheckConstraint, ForeignKey
from sqlalchemy.orm import relationship
from datetime import datetime, timedelta

class PersonalizationQuota(Base):
    __tablename__ = "personalization_quota"

    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), primary_key=True)
    request_count = Column(Integer, nullable=False, default=0)
    first_request_timestamp = Column(DateTime(timezone=True), nullable=False)
    reset_at = Column(DateTime(timezone=True), nullable=False)
    created_at = Column(DateTime(timezone=True), nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime(timezone=True), nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationship
    user = relationship("User", back_populates="personalization_quota")

    __table_args__ = (
        CheckConstraint("request_count >= 0 AND request_count <= 3", name="chk_request_count"),
    )

    @property
    def is_expired(self) -> bool:
        """Check if quota window has expired."""
        return datetime.utcnow() > self.reset_at

    @property
    def remaining_requests(self) -> int:
        """Calculate remaining requests in current window."""
        if self.is_expired:
            return 3  # Full quota after reset
        return max(0, 3 - self.request_count)

    def increment(self):
        """Increment request count or reset if window expired."""
        if self.is_expired:
            self.request_count = 1
            self.first_request_timestamp = datetime.utcnow()
            self.reset_at = self.first_request_timestamp + timedelta(hours=24)
        else:
            self.request_count += 1
        self.updated_at = datetime.utcnow()
```

**Migration (Alembic)**:
```python
"""Add personalization quota table

Revision ID: 005_personalization_quota
Revises: 004_urdu_translation
Create Date: 2025-12-23
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

def upgrade():
    op.create_table(
        'personalization_quota',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('request_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('first_request_timestamp', sa.DateTime(timezone=True), nullable=False),
        sa.Column('reset_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.CheckConstraint('request_count >= 0 AND request_count <= 3', name='chk_request_count'),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('user_id')
    )
    op.create_index('idx_personalization_quota_reset', 'personalization_quota', ['reset_at'])
    op.create_index('idx_personalization_quota_user', 'personalization_quota', ['user_id'])

def downgrade():
    op.drop_index('idx_personalization_quota_user', table_name='personalization_quota')
    op.drop_index('idx_personalization_quota_reset', table_name='personalization_quota')
    op.drop_table('personalization_quota')
```

---

### 2. PersonalizedContent (Frontend State - Not Persisted)

**Purpose**: Cache personalized chapter content in frontend session state for instant toggling between original and personalized views.

**TypeScript Interface**:
```typescript
interface UserProfile {
  softwareBackground: 'Beginner' | 'Intermediate' | 'Advanced' | 'Expert';
  hardwareBackground: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  interestArea: 'AI' | 'Robotics' | 'Computer Vision' | 'Motion Control' | 'General';
}

interface PersonalizedContent {
  chapterId: string;               // Unique chapter identifier (e.g., "01-foundations-ros2/02-nodes-topics")
  originalMarkdown: string;        // Original chapter content (unmodified)
  personalizedMarkdown: string;    // AI-generated personalized version
  generatedAt: Date;               // Timestamp when personalization was generated
  profileSnapshot: UserProfile;    // User profile at time of generation (for cache invalidation)
}

// Frontend State Management (React Context)
interface PersonalizationState {
  cachedContent: Map<string, PersonalizedContent>;  // chapterId -> PersonalizedContent
  rateLimitRemaining: number;                       // Remaining requests (0-3)
  currentView: 'original' | 'personalized';         // Current view mode
  isLoading: boolean;                               // Loading state for API calls
  error: string | null;                             // Error message if generation failed
}
```

**Storage Strategy**: React Context API (session-only, cleared on page reload)

**Lifecycle**:
1. **Creation**: Entry added to `cachedContent` map when personalization succeeds
2. **Access**: Retrieved from cache when toggling views (no re-generation)
3. **Invalidation**: Entire cache cleared when:
   - User updates profile (softwareBackground, hardwareBackground, or interestArea)
   - User logs out
   - Session ends (page reload/close)
4. **Expiration**: Automatic (session end)

**Validation Rules**:
- `chapterId` must match format: `{module-slug}/{chapter-slug}`
- `originalMarkdown` must be valid markdown with YAML frontmatter
- `personalizedMarkdown` must have same structure as `originalMarkdown` (validated by comparing AST)
- `profileSnapshot` must match user's profile at generation time (for cache invalidation)

**React Context Implementation**:
```typescript
import React, { createContext, useContext, useState, useCallback } from 'react';

const PersonalizationContext = createContext<PersonalizationState | null>(null);

export function PersonalizationProvider({ children }: { children: React.ReactNode }) {
  const [cachedContent, setCachedContent] = useState<Map<string, PersonalizedContent>>(new Map());
  const [rateLimitRemaining, setRateLimitRemaining] = useState<number>(3);
  const [currentView, setCurrentView] = useState<'original' | 'personalized'>('original');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  const addToCache = useCallback((content: PersonalizedContent) => {
    setCachedContent(prev => new Map(prev).set(content.chapterId, content));
  }, []);

  const clearCache = useCallback(() => {
    setCachedContent(new Map());
    setCurrentView('original');
  }, []);

  const value = {
    cachedContent,
    rateLimitRemaining,
    currentView,
    isLoading,
    error,
    addToCache,
    clearCache,
    setRateLimitRemaining,
    setCurrentView,
    setIsLoading,
    setError,
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
}

export function usePersonalizationState() {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalizationState must be used within PersonalizationProvider');
  }
  return context;
}
```

---

### 3. PersonalizationRequest (Logs Only - Not Stored)

**Purpose**: Capture request metadata for observability, monitoring, and debugging. Not stored as queryable entities, only logged as structured JSON.

**Log Format (JSON)**:
```json
{
  "event": "personalization_request",
  "request_id": "uuid-v4",
  "user_id": "uuid",
  "chapter_id": "01-foundations-ros2/02-nodes-topics",
  "status": "success",
  "duration_ms": 15342,
  "profile_snapshot": {
    "software_background": "Beginner",
    "hardware_background": "None",
    "interest_area": "AI"
  },
  "error_message": null,
  "timestamp": "2025-12-23T10:30:45.123Z",
  "timeout": false
}
```

**Status Enum**:
```python
from enum import Enum

class PersonalizationStatus(str, Enum):
    SUCCESS = "success"        # Personalization completed successfully
    TIMEOUT = "timeout"        # Request exceeded 30-second timeout
    RATE_LIMITED = "rate_limited"  # User exceeded 3 requests/day limit
    ERROR = "error"            # AI generation failed or other error
    PROFILE_INCOMPLETE = "profile_incomplete"  # User missing required profile fields
```

**Python Logging**:
```python
import logging
import json
from datetime import datetime
from uuid import uuid4

logger = logging.getLogger("personalization")

def log_personalization_request(
    user_id: str,
    chapter_id: str,
    status: PersonalizationStatus,
    duration_ms: float,
    profile_snapshot: dict,
    error_message: str | None = None,
    timeout: bool = False
):
    """Log structured personalization request for observability."""
    log_entry = {
        "event": "personalization_request",
        "request_id": str(uuid4()),
        "user_id": user_id,
        "chapter_id": chapter_id,
        "status": status.value,
        "duration_ms": duration_ms,
        "profile_snapshot": profile_snapshot,
        "error_message": error_message,
        "timeout": timeout,
        "timestamp": datetime.utcnow().isoformat()
    }

    if status == PersonalizationStatus.SUCCESS:
        logger.info(json.dumps(log_entry))
    else:
        logger.warning(json.dumps(log_entry))
```

---

## Relationships

```
users (1) ----< (1) personalization_quota
  │
  │ Referenced in:
  └─> PersonalizedContent.profileSnapshot (frontend state)
  └─> PersonalizationRequest.userId (logs)
```

- **One-to-One**: Each user has at most one `personalization_quota` entry (PK on user_id)
- **Cascade Delete**: Deleting a user deletes their quota entry
- **No Foreign Keys**: PersonalizedContent and PersonalizationRequest don't create database relationships (state and logs respectively)

---

## Data Access Patterns

### Pattern 1: Check Rate Limit Before Personalization

**SQL Query**:
```sql
SELECT request_count, reset_at
FROM personalization_quota
WHERE user_id = $1;
```

**Logic**:
1. If no row exists → create new entry with `request_count=0`
2. If `NOW() > reset_at` → reset counter to 0
3. If `request_count >= 3` → return rate limit exceeded
4. Else → increment counter and proceed

**Performance**: Indexed lookup on `user_id` (primary key) → O(1) average case

### Pattern 2: Retrieve Cached Personalized Content

**Frontend State Access**:
```typescript
const getCachedPersonalization = (chapterId: string): PersonalizedContent | null => {
  return cachedContent.get(chapterId) || null;
};
```

**Logic**:
1. Check if `chapterId` exists in `cachedContent` map
2. If exists and `profileSnapshot` matches current profile → return cached content
3. If profile mismatch or not found → return null (trigger new personalization)

**Performance**: Map lookup → O(1) average case

### Pattern 3: Aggregate Metrics for Monitoring

**Log Aggregation Query** (e.g., CloudWatch Logs Insights):
```
fields @timestamp, status, duration_ms
| filter event = "personalization_request"
| stats count() as total_requests,
        count(status = "success") as success_count,
        count(status = "timeout") as timeout_count,
        avg(duration_ms) as avg_duration
```

**Metrics Calculation**:
- **Success Rate**: `(success_count / total_requests) * 100`
- **Timeout Rate**: `(timeout_count / total_requests) * 100`
- **Average Duration**: `avg_duration`

---

## Data Validation & Integrity

### Quota Validation

- **Constraint**: `request_count` must be 0-3 (enforced by CHECK constraint)
- **Foreign Key**: `user_id` must exist in `users` table (enforced by FK constraint)
- **Business Rule**: Quota resets exactly 24 hours after first request
- **Test Case**: Attempt to set `request_count=4` → should fail with constraint violation

### Content Validation

- **Structure**: Personalized markdown must have identical AST structure to original (same headings, code blocks, math blocks)
- **Preservation**: Code blocks, LaTeX formulas, YAML frontmatter must be byte-for-byte identical
- **Test Case**: Compare AST of original vs personalized → should match structure, differ only in prose content

### Profile Validation

- **Completeness**: User must have non-null values for `softwareBackground`, `hardwareBackground`, `interestArea`
- **Enum Values**: Each field must match allowed enum values
- **Test Case**: Attempt personalization with incomplete profile → should return 400 error with profile completion prompt

---

## Storage Size Estimates

### PostgreSQL Storage

**PersonalizationQuota Table**:
- Row size: ~120 bytes (UUID + 2 integers + 4 timestamps + overhead)
- Expected rows: ~10,000 users (grow with user base)
- Total storage: ~1.2 MB (negligible)
- Growth rate: Linear with user signups (~100 KB per 1000 new users)

### Frontend Session State

**PersonalizedContent Cache**:
- Average chapter size: ~50 KB markdown
- Cached content per session: ~2-3 chapters (3 requests/day limit)
- Total session state: ~150 KB (acceptable for modern browsers)
- Cleared on session end (no persistent growth)

### Logs

**PersonalizationRequest Logs**:
- Log entry size: ~500 bytes JSON
- Expected daily volume: 500 requests/day × 500 bytes = 250 KB/day
- Retention: 30 days (configurable)
- Total log storage: ~7.5 MB/month (negligible)

---

## Summary

| Entity | Storage | Purpose | Persistence |
|--------|---------|---------|-------------|
| PersonalizationQuota | PostgreSQL | Rate limiting tracking | Permanent (until user deleted) |
| PersonalizedContent | Frontend State (React Context) | Session cache for instant toggling | Session-only (cleared on reload) |
| PersonalizationRequest | Structured Logs (JSON) | Observability and metrics | 30-day retention |

**Key Design Decisions**:
- ✅ **Lightweight Storage**: Only rate limiting requires database persistence
- ✅ **Session-Only Caching**: Respects "no persistent personalized content" requirement
- ✅ **Structured Logging**: Enables metrics tracking without database overhead
- ✅ **Cascading Deletes**: User deletion automatically cleans up quota entries

## Next Steps

1. **API Contracts**: Define REST endpoints using these entities (`contracts/personalization-api.yaml`)
2. **Quickstart**: Document setup steps for migrations and testing (`quickstart.md`)
3. **Tasks**: Generate implementation tasks for each entity (`/sp.tasks`)
