# Data Model: Urdu Translation

**Feature**: 004-urdu-translation
**Date**: 2025-12-20
**Status**: Design Complete

## Overview

This document defines the data entities for the Urdu translation feature. The feature primarily uses caching and analytics logging to minimize translation costs and track usage.

---

## Entity Diagram

```
┌─────────────────────┐
│  TranslationCache   │
├─────────────────────┤
│ id (PK)             │
│ chapter_slug        │
│ language            │
│ content_hash        │
│ translated_content  │
│ created_at          │
│ accessed_at         │
│ expires_at          │
└─────────────────────┘
         │
         │ (tracks usage)
         ▼
┌─────────────────────┐
│  TranslationLog     │
├─────────────────────┤
│ id (PK)             │
│ user_id (FK)        │
│ chapter_slug        │
│ action              │
│ timestamp           │
└─────────────────────┘
         │
         │
         ▼
┌─────────────────────┐
│  User               │ (Existing table)
├─────────────────────┤
│ id (PK)             │
│ email               │
│ ...                 │
└─────────────────────┘
```

---

## 1. TranslationCache

**Purpose**: Store translated chapter content to avoid redundant OpenAI API calls.

**Lifecycle**: Cached entries expire after TTL (90 days default). When source content changes, the `content_hash` changes, making old cached translations stale.

### Attributes

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `SERIAL` | PRIMARY KEY | Auto-incrementing unique identifier |
| `chapter_slug` | `VARCHAR(255)` | NOT NULL | Chapter identifier (e.g., "01-what-is-ros2") |
| `language` | `VARCHAR(10)` | NOT NULL | Target language code ("ur" for Urdu, "en" for English) |
| `content_hash` | `VARCHAR(64)` | NOT NULL | SHA-256 hash of source markdown content (version tracking) |
| `translated_content` | `TEXT` | NOT NULL | Full translated markdown content in target language |
| `created_at` | `TIMESTAMP` | DEFAULT CURRENT_TIMESTAMP | When translation was first cached |
| `accessed_at` | `TIMESTAMP` | DEFAULT CURRENT_TIMESTAMP | Last time this cached translation was retrieved (for analytics) |
| `expires_at` | `TIMESTAMP` | NOT NULL | When this cached translation expires (created_at + TTL) |

### Indexes

```sql
CREATE INDEX idx_translation_lookup
ON translation_cache(chapter_slug, language, content_hash);

CREATE INDEX idx_translation_expiry
ON translation_cache(expires_at)
WHERE expires_at < CURRENT_TIMESTAMP;
```

### Constraints

```sql
UNIQUE(chapter_slug, language, content_hash)
```

This ensures one cached translation per unique combination of chapter + language + source version.

### Cache Key Structure

**Format**: `{chapter_slug}:{language}:{content_hash}`

**Examples**:
- `01-what-is-ros2:ur:a3f5e9b2c1d8f4a7...`
- `03-llms-for-cognitive-planning:ur:d9c2e1a8b5f3c6e2...`

### TTL Strategy

| Chapter Type | TTL | Rationale |
|--------------|-----|-----------|
| Preface, appendices | 180 days | Rarely change |
| Core curriculum chapters | 90 days (default) | Moderate update frequency |
| Code examples | 30 days | Library versions update frequently |

### Invalidation Rules

1. **Version-based (automatic)**: When source content changes, `content_hash` changes → cache miss → new translation generated
2. **TTL expiration**: Daily cleanup job deletes entries where `expires_at < CURRENT_TIMESTAMP`
3. **Manual purge (admin)**: Emergency endpoint to delete specific chapter translations

### SQL Schema

```sql
CREATE TABLE IF NOT EXISTS translation_cache (
    id SERIAL PRIMARY KEY,
    chapter_slug VARCHAR(255) NOT NULL,
    language VARCHAR(10) NOT NULL,
    content_hash VARCHAR(64) NOT NULL,
    translated_content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    accessed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP NOT NULL,
    UNIQUE(chapter_slug, language, content_hash)
);

CREATE INDEX idx_translation_lookup
ON translation_cache(chapter_slug, language, content_hash);

CREATE INDEX idx_translation_expiry
ON translation_cache(expires_at)
WHERE expires_at < CURRENT_TIMESTAMP;
```

### Pydantic Model

```python
from pydantic import BaseModel, Field
from datetime import datetime

class TranslationCache(BaseModel):
    id: int
    chapter_slug: str = Field(..., max_length=255)
    language: str = Field(..., max_length=10)
    content_hash: str = Field(..., min_length=64, max_length=64)
    translated_content: str
    created_at: datetime
    accessed_at: datetime
    expires_at: datetime

    class Config:
        from_attributes = True
```

---

## 2. TranslationLog

**Purpose**: Track translation requests for usage analytics and monitoring.

**Lifecycle**: Permanent (or retained per data retention policy, e.g., 1 year).

### Attributes

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `SERIAL` | PRIMARY KEY | Auto-incrementing unique identifier |
| `user_id` | `INTEGER` | FOREIGN KEY (users.id), NOT NULL | User who requested translation (from existing auth system) |
| `chapter_slug` | `VARCHAR(255)` | NOT NULL | Chapter identifier |
| `action` | `VARCHAR(50)` | NOT NULL | Action type: "translate_requested", "toggle_to_english", "toggle_to_urdu" |
| `timestamp` | `TIMESTAMP` | DEFAULT CURRENT_TIMESTAMP | When action occurred |

### Indexes

```sql
CREATE INDEX idx_translation_log_user
ON translation_log(user_id);

CREATE INDEX idx_translation_log_timestamp
ON translation_log(timestamp DESC);

CREATE INDEX idx_translation_log_chapter
ON translation_log(chapter_slug);
```

### Action Types

| Action | Description | When Triggered |
|--------|-------------|----------------|
| `translate_requested` | User clicked "Translate to Urdu" button | First translation of a chapter in session |
| `toggle_to_english` | User clicked "Show Original English" | Toggling back from Urdu to English |
| `toggle_to_urdu` | User clicked "Show Urdu Translation" again after viewing English | Re-toggling to Urdu (after viewing English) |
| `translation_failed` | Translation request failed | API error, rate limit, timeout |

### SQL Schema

```sql
CREATE TABLE IF NOT EXISTS translation_log (
    id SERIAL PRIMARY KEY,
    user_id INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_slug VARCHAR(255) NOT NULL,
    action VARCHAR(50) NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_translation_log_user ON translation_log(user_id);
CREATE INDEX idx_translation_log_timestamp ON translation_log(timestamp DESC);
CREATE INDEX idx_translation_log_chapter ON translation_log(chapter_slug);
```

### Pydantic Model

```python
from pydantic import BaseModel, Field
from datetime import datetime
from enum import Enum

class TranslationAction(str, Enum):
    TRANSLATE_REQUESTED = "translate_requested"
    TOGGLE_TO_ENGLISH = "toggle_to_english"
    TOGGLE_TO_URDU = "toggle_to_urdu"
    TRANSLATION_FAILED = "translation_failed"

class TranslationLog(BaseModel):
    id: int
    user_id: int
    chapter_slug: str = Field(..., max_length=255)
    action: TranslationAction
    timestamp: datetime

    class Config:
        from_attributes = True

class TranslationLogCreate(BaseModel):
    user_id: int
    chapter_slug: str = Field(..., max_length=255)
    action: TranslationAction
```

---

## 3. User (Existing Entity)

**Purpose**: Existing authentication entity (from Better-Auth integration).

**Relationship**: One user can have many translation logs.

### Relevant Attributes (for reference)

| Field | Type | Description |
|-------|------|-------------|
| `id` | `INTEGER` | PRIMARY KEY |
| `email` | `VARCHAR` | User email |
| `created_at` | `TIMESTAMP` | Account creation timestamp |

**Note**: No modifications needed to existing User table. Translation feature only reads `user.id` for logging.

---

## 4. ChapterMetadata (Virtual Entity)

**Purpose**: Represents source chapter content (not stored in database; read from filesystem).

**Source**: Markdown files in `textbook/docs/` directory.

### Attributes (in-memory only)

| Field | Type | Description |
|-------|------|-------------|
| `chapter_slug` | `string` | Derived from filename (e.g., "01-what-is-ros2.md" → "01-what-is-ros2") |
| `title` | `string` | Extracted from YAML frontmatter or first heading |
| `module` | `string` | Parent directory name (e.g., "01-foundations-ros2") |
| `content` | `string` | Full markdown content (including frontmatter) |
| `content_hash` | `string` | SHA-256 hash of `content` (for cache versioning) |

### Retrieval Pattern

```python
import hashlib
from pathlib import Path

def get_chapter_metadata(chapter_slug: str):
    # Locate file
    chapter_path = Path(f"textbook/docs/{chapter_slug}.md")
    if not chapter_path.exists():
        raise FileNotFoundError(f"Chapter not found: {chapter_slug}")

    # Read content
    content = chapter_path.read_text(encoding='utf-8')

    # Compute hash
    content_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()

    return {
        "chapter_slug": chapter_slug,
        "content": content,
        "content_hash": content_hash
    }
```

---

## Data Flow Diagram

```
User Request (Frontend)
    │
    ▼
POST /api/translate
    │
    ├─> Check Auth (user_id from session)
    │
    ├─> Get Chapter Metadata (read markdown file)
    │      │
    │      ▼
    │   Compute content_hash
    │
    ├─> Query TranslationCache
    │      │
    │      ├─ Cache Hit ──> Update accessed_at ──> Return cached translation
    │      │
    │      └─ Cache Miss ──┐
    │                      ▼
    │            Call OpenAI Agents SDK (translate)
    │                      │
    │                      ▼
    │            Insert into TranslationCache (with expires_at = now + 90 days)
    │                      │
    │                      ▼
    │            Return translated content
    │
    └─> Log Translation Request (TranslationLog)
           │
           ▼
        Return JSON response
```

---

## Database Migration Script

### Alembic Migration (example)

```python
# alembic/versions/004_add_translation_tables.py
"""Add translation cache and log tables

Revision ID: 004_urdu_translation
Revises: 003_auth_tables
Create Date: 2025-12-20
"""
from alembic import op
import sqlalchemy as sa

def upgrade():
    # Create translation_cache table
    op.create_table(
        'translation_cache',
        sa.Column('id', sa.Integer(), primary_key=True),
        sa.Column('chapter_slug', sa.String(255), nullable=False),
        sa.Column('language', sa.String(10), nullable=False),
        sa.Column('content_hash', sa.String(64), nullable=False),
        sa.Column('translated_content', sa.Text(), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.func.current_timestamp()),
        sa.Column('accessed_at', sa.TIMESTAMP(), server_default=sa.func.current_timestamp()),
        sa.Column('expires_at', sa.TIMESTAMP(), nullable=False),
        sa.UniqueConstraint('chapter_slug', 'language', 'content_hash', name='uq_translation_cache')
    )

    op.create_index('idx_translation_lookup', 'translation_cache', ['chapter_slug', 'language', 'content_hash'])
    op.create_index('idx_translation_expiry', 'translation_cache', ['expires_at'])

    # Create translation_log table
    op.create_table(
        'translation_log',
        sa.Column('id', sa.Integer(), primary_key=True),
        sa.Column('user_id', sa.Integer(), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('chapter_slug', sa.String(255), nullable=False),
        sa.Column('action', sa.String(50), nullable=False),
        sa.Column('timestamp', sa.TIMESTAMP(), server_default=sa.func.current_timestamp())
    )

    op.create_index('idx_translation_log_user', 'translation_log', ['user_id'])
    op.create_index('idx_translation_log_timestamp', 'translation_log', ['timestamp'], postgresql_ops={'timestamp': 'DESC'})
    op.create_index('idx_translation_log_chapter', 'translation_log', ['chapter_slug'])

def downgrade():
    op.drop_table('translation_log')
    op.drop_table('translation_cache')
```

---

## Storage Estimates

### TranslationCache

**Assumptions**:
- 50 chapters in textbook
- Average chapter: 5,000 words ≈ 35 KB raw text
- Translated Urdu: ~40 KB (Urdu text slightly longer due to script)
- Total per chapter: ~40 KB translated content + metadata (~500 bytes) = **41 KB**

**Total Storage**:
- 50 chapters × 41 KB = **2.05 MB** (English + Urdu)
- Scaling to 1,000 chapters = **41 MB** (still well within PostgreSQL limits)

### TranslationLog

**Assumptions**:
- 100 users × 50 chapters = 5,000 initial translation requests
- Each log entry: ~100 bytes (user_id, chapter_slug, action, timestamp)
- Total: 5,000 × 100 bytes = **500 KB**

**Annual Growth** (500 active users):
- 500 users × 50 chapters × 2 actions (translate + toggle) = 50,000 logs
- 50,000 × 100 bytes = **5 MB/year**

**Total Year 1**: 2.05 MB (cache) + 5 MB (logs) = **~7 MB**

---

## Summary

| Entity | Purpose | Storage | Lifecycle |
|--------|---------|---------|-----------|
| TranslationCache | Store translated content to avoid redundant API calls | 2 MB (50 chapters) | Expires after 90 days TTL or when content_hash changes |
| TranslationLog | Track user translation requests for analytics | 5 MB/year | Permanent (or 1-year retention) |
| User (existing) | Authentication | N/A | Existing table |
| ChapterMetadata (virtual) | Source markdown content | N/A (filesystem) | Read on-demand |

**Database Impact**: Minimal (< 10 MB Year 1), easily fits in Neon free tier (500 MB limit).

---

**Data Model Status**: ✅ Complete - Ready for API contract design
