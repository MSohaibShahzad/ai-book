# Data Model: RAG Chatbot for Physical-AI Textbook

**Feature**: 002-rag-chatbot
**Phase**: 1 - Design & Contracts
**Date**: 2025-12-10
**Purpose**: Define entities, schemas, and relationships for backend and frontend

---

## Overview

The RAG chatbot system uses a hybrid storage approach:
- **Qdrant Cloud**: Stores vector embeddings for semantic search
- **Neon Postgres**: Stores metadata for source attribution and chunk management
- **Frontend State**: Ephemeral conversation history (session-only, no persistence)

---

## Backend Entities

### 1. TextbookChunk (Postgres)

**Purpose**: Stores metadata for each textbook chunk, linking to Qdrant vectors

**Schema**:
```sql
CREATE TABLE textbook_chunks (
    id SERIAL PRIMARY KEY,
    qdrant_vector_id UUID UNIQUE NOT NULL,
    chunk_text TEXT NOT NULL,
    module_name VARCHAR(255) NOT NULL,
    chapter_name VARCHAR(255) NOT NULL,
    section_heading VARCHAR(500),
    file_path VARCHAR(500) NOT NULL,
    slug VARCHAR(500) NOT NULL,
    chunk_index INTEGER NOT NULL,
    token_count INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Indexes for performance
    CONSTRAINT unique_chunk UNIQUE (file_path, chunk_index),
    INDEX idx_qdrant_vector_id (qdrant_vector_id),
    INDEX idx_module_chapter (module_name, chapter_name),
    INDEX idx_slug (slug)
);
```

**Pydantic Model**:
```python
from pydantic import BaseModel, UUID4
from datetime import datetime

class TextbookChunkMetadata(BaseModel):
    id: int
    qdrant_vector_id: UUID4
    chunk_text: str
    module_name: str
    chapter_name: str
    section_heading: str | None
    file_path: str
    slug: str
    chunk_index: int
    token_count: int
    created_at: datetime

    class Config:
        from_attributes = True  # For SQLAlchemy compatibility
```

**Fields**:
- `id`: Auto-incrementing primary key
- `qdrant_vector_id`: Foreign key to Qdrant vector (UUID generated during ingestion)
- `chunk_text`: Full text content of the chunk (for display in retrieval results)
- `module_name`: Module title from frontmatter (e.g., "Foundations - ROS 2")
- `chapter_name`: Chapter title from frontmatter (e.g., "Chapter 1: Introduction")
- `section_heading`: Heading level (## or ###) where chunk originates
- `file_path`: Relative path from textbook root (e.g., "docs/module-1/chapter-1.md")
- `slug`: URL slug for deep linking (e.g., "/docs/foundations-ros2")
- `chunk_index`: Sequential index within file (0-based)
- `token_count`: Number of tokens in chunk (for context window management)
- `created_at`: Timestamp of ingestion (for versioning)

**Relationships**:
- 1:1 with Qdrant vector via `qdrant_vector_id`

**Validation Rules**:
- `chunk_text`: Max 10,000 characters (safety limit)
- `token_count`: Range [50, 800] (based on chunking strategy)
- `file_path + chunk_index`: Unique constraint prevents duplicate chunks

---

### 2. QdrantVector (Qdrant Cloud)

**Purpose**: Stores embedding vectors for semantic similarity search

**Schema** (Qdrant collection):
```python
from qdrant_client.models import VectorParams, Distance

collection_config = {
    "collection_name": "textbook_chunks",
    "vectors_config": VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimension
        distance=Distance.COSINE
    )
}
```

**Point Structure**:
```python
{
    "id": UUID("..."),  # Same as textbook_chunks.qdrant_vector_id
    "vector": [0.123, -0.456, ...],  # 1536-dim embedding
    "payload": {
        "chunk_id": 42,  # Reference to textbook_chunks.id
        "module_name": "Foundations - ROS 2",  # Cached for filtering
        "chapter_name": "Chapter 1: Introduction"
    }
}
```

**Fields**:
- `id`: UUID (primary key, matches Postgres `qdrant_vector_id`)
- `vector`: 1536-dimensional float array (OpenAI embedding)
- `payload.chunk_id`: Foreign key to Postgres `textbook_chunks.id`
- `payload.module_name`: Cached for filter-based retrieval
- `payload.chapter_name`: Cached for filter-based retrieval

**Operations**:
- **Insert**: Batch upsert during ingestion
- **Query**: Cosine similarity search with optional filters
- **Delete**: Remove vectors when textbook content is updated

---

### 3. ChatRequest (API Request Model)

**Purpose**: Represents incoming chat request from frontend

**Pydantic Model**:
```python
from pydantic import BaseModel, Field

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000, description="User's question")
    highlighted_text: str | None = Field(None, max_length=5000, description="Optional highlighted passage")
    session_id: str = Field(..., min_length=36, max_length=36, description="Client-generated UUID")
    conversation_history: list[dict] | None = Field(None, max_items=10, description="Last N messages for context")

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is inverse kinematics?",
                "highlighted_text": None,
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "conversation_history": []
            }
        }
```

**Fields**:
- `message`: Student's question (required)
- `highlighted_text`: Optional text selected on page
- `session_id`: Client-generated UUID for multi-turn conversations
- `conversation_history`: Array of recent messages (max 10 for context window management)

**Validation Rules**:
- `message`: 1-2000 characters (prevents abuse)
- `highlighted_text`: Max 5000 characters (2000 char used, rest for overflow)
- `conversation_history`: Max 10 items (limits context window to ~2k tokens)

---

### 4. ChatResponse (API Response Model)

**Purpose**: Represents chat response with answer and sources

**Pydantic Model**:
```python
from pydantic import BaseModel

class SourceReference(BaseModel):
    module_name: str
    chapter_name: str
    slug: str
    chunk_text_preview: str = Field(..., max_length=200)  # First 200 chars

class ChatResponse(BaseModel):
    response: str = Field(..., description="Generated answer in Markdown")
    sources: list[SourceReference] = Field(..., min_items=0, max_items=5)
    session_id: str = Field(..., description="Echo session_id for client correlation")
    retrieval_count: int = Field(..., description="Number of chunks retrieved from RAG")
    processing_time_ms: float = Field(..., description="Total processing time in milliseconds")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "Inverse kinematics is...\n\nSource: Module 2, Chapter 3",
                "sources": [
                    {
                        "module_name": "Robot Manipulation",
                        "chapter_name": "Chapter 3: Inverse Kinematics",
                        "slug": "/docs/robot-manipulation/inverse-kinematics",
                        "chunk_text_preview": "Inverse kinematics (IK) solves the problem of..."
                    }
                ],
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "retrieval_count": 3,
                "processing_time_ms": 847.3
            }
        }
```

**Fields**:
- `response`: LLM-generated answer with Markdown formatting
- `sources`: Array of source references (0-5 items based on retrieval)
- `session_id`: Echo of request session_id
- `retrieval_count`: Number of chunks retrieved (for debugging/analytics)
- `processing_time_ms`: Total latency (for monitoring)

---

### 5. RetrievalResult (Internal Model)

**Purpose**: Intermediate representation of Qdrant search results with metadata

**Pydantic Model**:
```python
class RetrievalResult(BaseModel):
    chunk_text: str
    module_name: str
    chapter_name: str
    section_heading: str | None
    slug: str
    similarity_score: float = Field(..., ge=0.0, le=1.0)
    qdrant_vector_id: UUID4

    def to_source_reference(self) -> SourceReference:
        return SourceReference(
            module_name=self.module_name,
            chapter_name=self.chapter_name,
            slug=self.slug,
            chunk_text_preview=self.chunk_text[:200]
        )
```

**Fields**:
- `chunk_text`: Full chunk content for LLM context
- `module_name`, `chapter_name`, `section_heading`: Metadata for attribution
- `slug`: URL for deep linking
- `similarity_score`: Cosine similarity (0.0-1.0)
- `qdrant_vector_id`: For joining with Postgres metadata

**Usage**:
- RAG pipeline retrieves from Qdrant → joins with Postgres → constructs RetrievalResult
- Sorted by `similarity_score` descending
- Top 3-5 results used for LLM context

---

## Frontend Entities

### 6. Message (Frontend TypeScript)

**Purpose**: Represents a single message in conversation UI

**TypeScript Interface**:
```typescript
interface Message {
  id: string;  // Client-generated UUID
  role: 'user' | 'assistant';
  content: string;  // Plain text (user) or Markdown (assistant)
  sources?: SourceReference[];  // Only for assistant messages
  timestamp: Date;
  processing_time_ms?: number;  // Only for assistant messages
}
```

**Fields**:
- `id`: Unique identifier for React key prop
- `role`: Message sender
- `content`: Message text (Markdown for assistant)
- `sources`: Source references (assistant only)
- `timestamp`: Message creation time
- `processing_time_ms`: Latency (assistant only, for UX feedback)

---

### 7. ConversationState (Frontend React State)

**Purpose**: Ephemeral conversation state (session-only, no persistence)

**TypeScript Interface**:
```typescript
interface ConversationState {
  sessionId: string;  // Generated on mount
  messages: Message[];  // Conversation history
  isLoading: boolean;  // True when waiting for response
  error: string | null;  // Last error message
}
```

**State Management**:
```typescript
const [conversation, setConversation] = useState<ConversationState>({
  sessionId: crypto.randomUUID(),
  messages: [],
  isLoading: false,
  error: null
});
```

**Lifecycle**:
- **Created**: On ChatWidget mount
- **Updated**: On user message, assistant response, error
- **Destroyed**: On page refresh, widget unmount

**Constitution Compliance**: No localStorage, sessionStorage, or cookies (Principle XIII)

---

## Entity Relationships

```
┌─────────────────────────────────────────────────────────┐
│                  Postgres (Neon)                        │
│                                                         │
│  ┌──────────────────────────────────────┐             │
│  │ textbook_chunks                      │             │
│  │------------------------------------ │             │
│  │ id (PK)                              │             │
│  │ qdrant_vector_id (UUID) ───────────────────┐       │
│  │ chunk_text                           │      │       │
│  │ module_name                          │      │       │
│  │ chapter_name                         │      │       │
│  │ section_heading                      │      │       │
│  │ file_path                            │      │       │
│  │ slug                                 │      │       │
│  │ chunk_index                          │      │       │
│  │ token_count                          │      │       │
│  │ created_at                           │      │       │
│  └──────────────────────────────────────┘      │       │
└─────────────────────────────────────────────────│───────┘
                                                   │
                                                   │ 1:1
                                                   │
┌──────────────────────────────────────────────────┼───────┐
│                Qdrant Cloud                      │       │
│                                                  │       │
│  ┌───────────────────────────────────────────┐  │       │
│  │ textbook_chunks collection                │  │       │
│  │─────────────────────────────────────────  │  │       │
│  │ id (UUID) <──────────────────────────────────┘       │
│  │ vector (1536 floats)                      │          │
│  │ payload:                                  │          │
│  │   - chunk_id (FK to Postgres)             │          │
│  │   - module_name (cached)                  │          │
│  │   - chapter_name (cached)                 │          │
│  └───────────────────────────────────────────┘          │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│             Frontend (React State)                      │
│                                                         │
│  ┌──────────────────────────────────────┐              │
│  │ ConversationState                    │              │
│  │------------------------------------ │              │
│  │ sessionId (UUID)                     │              │
│  │ messages: Message[]                  │              │
│  │   ├─ id (UUID)                       │              │
│  │   ├─ role (user | assistant)         │              │
│  │   ├─ content (string)                │              │
│  │   ├─ sources?: SourceReference[]     │              │
│  │   └─ timestamp (Date)                │              │
│  │ isLoading (boolean)                  │              │
│  │ error (string | null)                │              │
│  └──────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────┘
```

---

## Data Flow

### Ingestion Flow
1. Parse markdown files → extract frontmatter (module, chapter, slug)
2. Chunk by headings (300-500 tokens) → generate `TextbookChunk` records
3. Embed chunks via OpenAI → get 1536-dim vectors
4. Insert vectors into Qdrant with UUID
5. Insert metadata into Postgres with same UUID

### Query Flow (Standard RAG)
1. User sends message → frontend generates `ChatRequest`
2. Backend embeds query → searches Qdrant (cosine similarity)
3. Retrieve top 3-5 vectors → join with Postgres metadata via UUID
4. Format context → call OpenAI LLM with system prompt
5. Return `ChatResponse` with answer + sources

### Query Flow (Highlighted Text)
1. User selects text → frontend captures in `highlighted_text`
2. Backend uses highlighted text as primary context
3. Optionally retrieve 2-3 additional chunks via RAG
4. Combine highlighted text + retrieved chunks → LLM
5. Return `ChatResponse` explicitly referencing highlighted passage

---

## Validation Rules Summary

| Entity | Field | Constraint |
|--------|-------|-----------|
| TextbookChunk | chunk_text | Max 10,000 chars |
| TextbookChunk | token_count | Range [50, 800] |
| TextbookChunk | file_path + chunk_index | Unique |
| ChatRequest | message | 1-2000 chars |
| ChatRequest | highlighted_text | Max 5000 chars |
| ChatRequest | conversation_history | Max 10 items |
| ChatResponse | sources | 0-5 items |
| Message | content | Max 10,000 chars (Markdown) |

---

## Indexing Strategy

### Postgres Indexes
```sql
-- Primary key (auto-created)
CREATE UNIQUE INDEX textbook_chunks_pkey ON textbook_chunks (id);

-- Foreign key to Qdrant
CREATE UNIQUE INDEX idx_qdrant_vector_id ON textbook_chunks (qdrant_vector_id);

-- Filter by module/chapter
CREATE INDEX idx_module_chapter ON textbook_chunks (module_name, chapter_name);

-- Deep linking
CREATE INDEX idx_slug ON textbook_chunks (slug);

-- Prevent duplicate chunks
CREATE UNIQUE INDEX unique_chunk ON textbook_chunks (file_path, chunk_index);
```

### Qdrant Index
- **HNSW** (Hierarchical Navigable Small World) - default for fast ANN search
- **M parameter**: 16 (balance between recall and speed)
- **ef_construct**: 100 (indexing time vs quality)

---

## Migration Strategy

### Initial Setup
```sql
-- Run once during deployment
CREATE TABLE textbook_chunks (...);
CREATE INDEX idx_qdrant_vector_id ...;
CREATE INDEX idx_module_chapter ...;
```

### Content Updates
1. Increment chunk version (add `version` column)
2. Mark old chunks as archived
3. Insert new chunks with new version
4. Rebuild Qdrant collection with new vectors
5. Update Postgres foreign keys

---

## Next Steps

- Generate `contracts/openapi.yaml` with REST API specification
- Create `quickstart.md` with setup instructions
- Update `CLAUDE.md` with new technologies
