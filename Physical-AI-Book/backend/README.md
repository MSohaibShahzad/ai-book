# RAG Chatbot Backend

Backend API for the Physical-AI Textbook RAG (Retrieval-Augmented Generation) chatbot.

## Architecture

- **Framework**: FastAPI 0.104+
- **Vector Database**: Qdrant Cloud (embeddings)
- **Metadata Database**: Neon Postgres (chunk metadata)
- **LLM**: OpenAI GPT-4o-mini
- **Embeddings**: OpenAI text-embedding-3-small

## Project Structure

```
backend/
├── src/
│   ├── api/
│   │   ├── models/          # Pydantic request/response models
│   │   └── routes/          # API endpoints
│   │       ├── health.py    # Health check endpoint
│   │       └── chat.py      # Chat endpoint (RAG)
│   ├── services/
│   │   ├── embeddings.py    # OpenAI embedding generation
│   │   ├── qdrant_service.py # Vector search
│   │   ├── postgres_service.py # Metadata queries
│   │   ├── llm_service.py   # LLM generation
│   │   └── rag_pipeline.py  # RAG orchestration
│   ├── ingestion/
│   │   ├── textbook_parser.py # Markdown parsing
│   │   ├── chunker.py       # Text chunking
│   │   └── indexer.py       # Batch indexing
│   ├── config.py            # Settings management
│   └── main.py              # FastAPI app
├── scripts/
│   ├── init_database.py     # Create Postgres schema
│   └── ingest_textbook.py   # Index textbook content
└── tests/                   # Test suite

```

## Setup

### 1. Environment Variables

Create `.env` file with:

```bash
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_api_key
QDRANT_COLLECTION_NAME=textbook_chunks

# Neon Postgres
DATABASE_URL=postgresql://user:password@host/database

# OpenAI
OPENAI_API_KEY=sk-...
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o-mini

# App Settings
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=["http://localhost:3000"]
```

### 2. Install Dependencies

```bash
cd backend
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Initialize Database

```bash
python scripts/init_database.py
```

This creates the `textbook_chunks` table and indexes in Postgres.

### 4. Index Textbook Content

```bash
python scripts/ingest_textbook.py ../docs
```

This will:
1. Parse all Markdown files from Docusaurus `docs/` directory
2. Chunk content using heading-based strategy (300-500 tokens)
3. Generate embeddings using OpenAI
4. Store vectors in Qdrant
5. Store metadata in Postgres

### 5. Run Server

```bash
# Development mode with auto-reload
uvicorn src.main:app --reload --port 8000

# Production mode
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## API Endpoints

### Health Check

```bash
GET /v1/health
```

Response:
```json
{
  "status": "healthy",
  "qdrant": "connected",
  "postgres": "connected",
  "openai": "connected"
}
```

### Chat (RAG)

```bash
POST /v1/chat
```

Request:
```json
{
  "message": "What is inverse kinematics?",
  "highlighted_text": null,
  "session_id": "session-123",
  "conversation_history": [
    {"role": "user", "content": "Previous question"},
    {"role": "assistant", "content": "Previous answer"}
  ]
}
```

Response:
```json
{
  "response": "Inverse kinematics is the process of determining joint angles...",
  "sources": [
    {
      "module_name": "Kinematics",
      "chapter_name": "Inverse Kinematics",
      "slug": "kinematics/inverse-kinematics",
      "url": "/docs/kinematics/inverse-kinematics",
      "preview": "Inverse kinematics (IK) refers to..."
    }
  ],
  "session_id": "session-123",
  "retrieval_count": 3,
  "processing_time_ms": 850
}
```

## RAG Pipeline

The RAG pipeline follows these steps:

1. **Embed Query**: Convert user question to vector (1536-dim)
2. **Search Qdrant**: Find top-5 similar chunks (cosine similarity >0.6)
3. **Retrieve Metadata**: Join with Postgres to get full context
4. **Generate Response**: Call GPT-4o-mini with context and safety checks
5. **Extract Sources**: Deduplicate and format source references

## Safety & Ethics

The LLM service includes:
- **Harmful Query Detection**: Blocks queries about weaponization, harm, etc.
- **Factual Grounding**: Only answers based on provided context
- **Source Attribution**: All responses cite textbook sources
- **Redirection**: Points users to "Ethics & Safety" module when appropriate

## Testing

```bash
# Run tests
pytest

# With coverage
pytest --cov=src --cov-report=html
```

## Deployment

### Docker

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Environment Variables for Production

- Set `ENVIRONMENT=production`
- Use production database URLs
- Configure CORS for your domain
- Enable rate limiting
- Add monitoring (e.g., Sentry, DataDog)

## Monitoring

Health check endpoint provides status for all services:
- Qdrant connection
- Postgres connection
- OpenAI API availability

Monitor these metrics:
- Response time (p50, p95, p99)
- Error rates
- Token usage (OpenAI costs)
- Vector search latency

## Troubleshooting

### "This topic is not covered" responses

- Check similarity threshold (default 0.6)
- Verify textbook content is indexed
- Inspect Qdrant collection size

### Slow responses

- Check OpenAI API latency
- Verify network to Qdrant/Postgres
- Consider caching frequent queries

### Import errors

- Ensure you're in the `backend` directory
- Activate virtual environment
- Run imports with `python -m src.main` instead of direct execution

## License

Part of the Physical-AI Textbook project.
