# RAG Chatbot Quickstart Guide

This guide will help you set up and run the RAG (Retrieval-Augmented Generation) chatbot for the Physical-AI Textbook.

## Prerequisites

- Python 3.11+
- Node.js 18+
- Qdrant Cloud account (free tier available)
- Neon Postgres account (free tier available)
- OpenAI API key

## Architecture Overview

```
┌─────────────────┐
│  Docusaurus UI  │
│  (React + Chat  │
│    Widget)      │
└────────┬────────┘
         │ HTTP
         ↓
┌─────────────────┐
│  FastAPI Backend│
│   (RAG Pipeline)│
└────┬────────────┘
     │
     ├──→ OpenAI API (Embeddings + LLM)
     ├──→ Qdrant Cloud (Vector Search)
     └──→ Neon Postgres (Metadata)
```

## Step 1: Clone and Navigate

```bash
cd textbook
```

## Step 2: Backend Setup

### 2.1 Create Environment File

Create `backend/.env`:

```bash
# Qdrant Cloud - Get from https://cloud.qdrant.io
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=textbook_chunks

# Neon Postgres - Get from https://neon.tech
DATABASE_URL=postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/neondb?sslmode=require

# OpenAI - Get from https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-proj-...
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o-mini

# App Settings
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=["http://localhost:3000"]
RATE_LIMIT_PER_MINUTE=10

# Retrieval Settings
MAX_RETRIEVAL_CHUNKS=5
CHUNK_OVERLAP_TOKENS=50

# LLM Settings
LLM_TEMPERATURE=0.7
LLM_MAX_TOKENS=1000
```

### 2.2 Install Python Dependencies

```bash
cd backend
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2.3 Initialize Database

```bash
# Create Postgres schema
python scripts/init_database.py
```

Expected output:
```
Creating textbook_chunks table...
Creating indexes...
✅ Database schema initialized successfully
```

### 2.4 Index Textbook Content

```bash
# Index all markdown files from docs/
python scripts/ingest_textbook.py ../docs --dry-run  # Preview first

# Then run actual indexing
python scripts/ingest_textbook.py ../docs
```

This will:
1. Parse Markdown files with YAML frontmatter
2. Chunk content (300-500 tokens per chunk)
3. Generate embeddings using OpenAI
4. Store vectors in Qdrant
5. Store metadata in Postgres

Expected duration: ~5-10 minutes for full textbook

### 2.5 Test Backend Setup

```bash
# Run comprehensive test
python scripts/test_setup.py
```

Expected output:
```
✓ Configuration loaded
✓ Postgres connected (X chunks indexed)
✓ Qdrant connected (X vectors stored)
✓ OpenAI API accessible
✓ RAG pipeline working
```

### 2.6 Start Backend Server

```bash
# Development mode with auto-reload
uvicorn src.main:app --reload --port 8000
```

Backend will be available at: http://localhost:8000

API docs at: http://localhost:8000/docs

## Step 3: Frontend Setup

### 3.1 Create Environment File

Create `textbook/.env`:

```bash
REACT_APP_API_URL=http://localhost:8000/v1
```

### 3.2 Install Node Dependencies

```bash
cd textbook  # If not already there
npm install
```

### 3.3 Start Docusaurus Development Server

```bash
npm start
```

Frontend will be available at: http://localhost:3000

## Step 4: Test the Chatbot

1. Open http://localhost:3000 in your browser
2. You should see a purple chat button (💬) in the bottom-right corner
3. Click to open the chat widget
4. Try example questions:
   - "What is inverse kinematics?"
   - "Explain the Denavit-Hartenberg convention"
   - "How do VLAs work?"
   - "What are the applications of physical AI?"

Expected behavior:
- Response with 2-5 source references
- Clickable links to textbook chapters
- Response time: 500-1500ms

## Step 5: Verify End-to-End Flow

### Test Health Check

```bash
curl http://localhost:8000/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "qdrant": "connected",
  "postgres": "connected",
  "openai": "connected"
}
```

### Test Chat API Directly

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is inverse kinematics?",
    "session_id": "test-session"
  }'
```

Expected response:
```json
{
  "response": "Inverse kinematics is the process...",
  "sources": [
    {
      "module_name": "Kinematics",
      "chapter_name": "Inverse Kinematics",
      "slug": "kinematics/inverse-kinematics",
      "url": "/docs/kinematics/inverse-kinematics",
      "preview": "..."
    }
  ],
  "session_id": "test-session",
  "retrieval_count": 3,
  "processing_time_ms": 850
}
```

## Troubleshooting

### Backend Issues

**Issue**: `ModuleNotFoundError: No module named 'src'`

Solution: Make sure you're in the `backend` directory and virtual environment is activated:
```bash
cd backend
source venv/bin/activate
```

**Issue**: "This topic is not covered" for all queries

Solution:
1. Check if textbook content is indexed: `python scripts/test_setup.py`
2. Lower similarity threshold in `src/services/rag_pipeline.py`
3. Verify Qdrant collection has vectors

**Issue**: Slow responses (>3 seconds)

Solution:
1. Check OpenAI API status
2. Verify network connectivity to Qdrant/Postgres
3. Reduce `MAX_RETRIEVAL_CHUNKS` in `.env`

### Frontend Issues

**Issue**: Chat widget not appearing

Solution:
1. Check browser console for errors
2. Verify `REACT_APP_API_URL` in `textbook/.env`
3. Ensure backend is running on port 8000

**Issue**: CORS errors in browser console

Solution: Update `CORS_ORIGINS` in `backend/.env` to include your frontend URL

**Issue**: "Failed to fetch" error

Solution:
1. Verify backend is running: `curl http://localhost:8000/v1/health`
2. Check firewall settings
3. Try `http://127.0.0.1:8000/v1` instead of `localhost`

## Production Deployment

### Backend (Recommended: Fly.io, Railway, Render)

```bash
cd backend

# Build Docker image
docker build -t rag-chatbot-backend .

# Run container
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

Set environment variables:
- `ENVIRONMENT=production`
- Update `CORS_ORIGINS` to your domain
- Use production database URLs

### Frontend (Recommended: Vercel, Netlify, GitHub Pages)

```bash
cd textbook

# Build static site
npm run build

# Deploy build/ directory
```

Update `REACT_APP_API_URL` to your production backend URL.

## Cost Estimates (Free Tier)

- **Qdrant Cloud**: Free tier (1GB storage, 1M vectors)
- **Neon Postgres**: Free tier (0.5GB storage)
- **OpenAI API**: Pay-as-you-go
  - Embeddings: ~$0.0001 per 1K tokens (~$0.50 to index full textbook)
  - LLM calls: ~$0.0015 per 1K tokens (~$0.02 per student query)
  - Estimated: $5-10/month for 500 queries

## Next Steps

1. **Customize System Prompt**: Edit `backend/src/services/llm_service.py`
2. **Add Module Filtering**: Update chat widget to filter by module
3. **Enable Highlighted Text**: Implement text selection in Docusaurus
4. **Add Analytics**: Track query patterns and user engagement
5. **Improve Chunking**: Tune chunk size based on query performance

## Support

- Backend API docs: http://localhost:8000/docs
- Backend README: `backend/README.md`
- Report issues: Create an issue in the repository

## Architecture Decisions

Key design choices (see ADRs for details):
- **Vector Database**: Qdrant for performance and managed service
- **Chunking Strategy**: Heading-based (300-500 tokens) for context coherence
- **LLM Model**: GPT-4o-mini for cost/quality balance
- **Deployment**: Stateless backend for horizontal scaling
- **Session Management**: Client-side only (no persistence)

---

**Congratulations!** 🎉 Your RAG chatbot is now running. Students can ask questions and receive grounded answers with source references.
