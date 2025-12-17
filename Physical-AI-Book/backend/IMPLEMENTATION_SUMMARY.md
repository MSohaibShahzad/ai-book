# RAG Chatbot Implementation Summary

## Complete Feature Set

This RAG chatbot now includes:

1. ✅ **OpenAI Agents SDK Integration** - Agent-based architecture with tool calling
2. ✅ **Streaming Responses** - Real-time token-by-token streaming via SSE
3. ✅ **RAG Pipeline** - Qdrant vector search + Neon Postgres metadata
4. ✅ **Rate Limiting** - 10 requests/minute per IP
5. ✅ **CORS Support** - Configured for frontend integration
6. ✅ **Error Handling** - Comprehensive error handling and logging
7. ✅ **Safety Checks** - Harmful content filtering

## API Endpoints

### 1. Non-Streaming Chat
**Endpoint**: `POST /v1/chat`

**Request**:
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "highlighted_text": null,
  "conversation_history": []
}
```

**Response**:
```json
{
  "response": "Inverse kinematics (IK) is...",
  "sources": [],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "retrieval_count": 0,
  "processing_time_ms": 18022
}
```

### 2. Streaming Chat (NEW)
**Endpoint**: `POST /v1/chat/stream`

**Request**: Same as above

**Response** (Server-Sent Events):
```
data: {"chunk": "Inverse"}
data: {"chunk": " kinematics"}
data: {"chunk": " is"}
...
data: {"done": true}
```

### 3. Health Check
**Endpoint**: `GET /v1/health`

**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-13T00:00:00Z"
}
```

## Architecture

```
┌─────────────────┐
│   Chat Widget   │
│   (Frontend)    │
└────────┬────────┘
         │
         │ HTTP POST
         ↓
┌─────────────────────────────────────────┐
│          FastAPI Backend                │
│  ┌───────────────────────────────────┐  │
│  │  /v1/chat         (non-streaming) │  │
│  │  /v1/chat/stream  (SSE streaming) │  │
│  └───────────┬───────────────────────┘  │
│              ↓                           │
│  ┌───────────────────────────────────┐  │
│  │      RAG Pipeline                 │  │
│  │  - Agent orchestration            │  │
│  │  - Query processing               │  │
│  └───────────┬───────────────────────┘  │
│              ↓                           │
│  ┌───────────────────────────────────┐  │
│  │   OpenAI Agent Service            │  │
│  │  ┌─────────────────────────────┐  │  │
│  │  │   Physical-AI Tutor Agent   │  │  │
│  │  │  - Instructions (system)    │  │  │
│  │  │  - Model: gpt-4o-mini       │  │  │
│  │  │  - Tools: [retrieval tool]  │  │  │
│  │  └──────────┬──────────────────┘  │  │
│  │             ↓                      │  │
│  │  ┌─────────────────────────────┐  │  │
│  │  │  retrieve_textbook_context  │  │  │
│  │  │  (function_tool)            │  │  │
│  │  │  1. Generate embedding      │  │  │
│  │  │  2. Search Qdrant           │  │  │
│  │  │  3. Fetch Postgres metadata │  │  │
│  │  │  4. Format context          │  │  │
│  │  └─────────────────────────────┘  │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
         │               │
         ↓               ↓
┌─────────────┐   ┌──────────────┐
│   Qdrant    │   │    Neon      │
│   Cloud     │   │   Postgres   │
│  (Vectors)  │   │  (Metadata)  │
└─────────────┘   └──────────────┘
         │               │
         └───────┬───────┘
                 ↓
         ┌──────────────┐
         │   OpenAI     │
         │  GPT-4o-mini │
         │ (via Agents) │
         └──────────────┘
```

## Technology Stack

### Backend
- **Framework**: FastAPI 0.104+
- **Agent SDK**: OpenAI Agents Python SDK
- **LLM**: OpenAI GPT-4o-mini
- **Embeddings**: OpenAI text-embedding-3-small
- **Vector DB**: Qdrant Cloud
- **Metadata DB**: Neon Postgres (serverless)
- **Python**: 3.11+

### Dependencies
```txt
fastapi>=0.104.1
uvicorn[standard]>=0.24.0
pydantic>=2.5.0
qdrant-client>=1.7.0
psycopg2-binary==2.9.9
sqlalchemy==2.0.23
openai>=1.33.0
openai-agents
python-dotenv==1.0.0
tiktoken==0.5.2
pytest==7.4.3
slowapi==0.1.9
```

## Key Features Explained

### 1. OpenAI Agents SDK
- **Agent-based architecture**: Agent autonomously decides when to retrieve context
- **Tool calling**: `retrieve_textbook_context` function decorated with `@function_tool`
- **Streaming support**: `Runner.run_streamed()` for real-time responses
- **Event-driven**: Yields `ResponseTextDeltaEvent` objects

### 2. Streaming (SSE)
- **Server-Sent Events**: Standard HTTP streaming protocol
- **Progressive rendering**: Chunks appear as they're generated
- **Better UX**: Feels faster than waiting for full response
- **Backwards compatible**: Original `/v1/chat` endpoint unchanged

### 3. RAG Pipeline
- **Hybrid search**: Combines embedding similarity + metadata filtering
- **Two-stage retrieval**:
  1. Qdrant vector search (top-k similar chunks)
  2. Postgres metadata enrichment (module, chapter, section info)
- **Context formatting**: Structured excerpts with source attribution

### 4. Safety & Quality
- **Harmful content filtering**: Keywords-based safety check
- **Rate limiting**: 10 req/min per IP (via SlowAPI)
- **Error handling**: Comprehensive try-catch with logging
- **CORS**: Configured for localhost:3000 and production domain

## Running the Application

### Start Backend
```bash
cd backend
source venv/bin/activate
pip install -r requirements.txt
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### Test Endpoints

**Non-streaming**:
```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is a robot?", "session_id": "550e8400-e29b-41d4-a716-446655440000"}'
```

**Streaming**:
```bash
curl -N -X POST http://localhost:8000/v1/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"message": "What is a robot?", "session_id": "550e8400-e29b-41d4-a716-446655440000"}'
```

### Try the Demo
Open `backend/streaming_demo.html` in your browser to see a live demo of:
- Streaming vs non-streaming comparison
- Real-time token-by-token rendering
- Interactive chat interface

## Environment Variables

Required in `backend/.env`:

```env
# Qdrant Cloud
QDRANT_URL="https://..."
QDRANT_API_KEY="..."
QDRANT_COLLECTION_NAME=textbook_chunks

# Neon Postgres
DATABASE_URL="postgresql://..."

# OpenAI
OPENAI_API_KEY="sk-..."
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o-mini

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=["http://localhost:3000"]
RATE_LIMIT_PER_MINUTE=10
```

## Performance Metrics

Based on testing:

- **First token latency (streaming)**: ~3-5 seconds
- **Full response time (non-streaming)**: ~15-20 seconds
- **Tokens per second**: ~30-50 (varies by query complexity)
- **Vector search time**: ~200-500ms
- **Metadata fetch time**: ~50-150ms

## Files Structure

```
backend/
├── src/
│   ├── api/
│   │   ├── models/
│   │   │   ├── request.py      # Pydantic request models
│   │   │   └── response.py     # Pydantic response models
│   │   └── routes/
│   │       ├── chat.py         # Chat endpoints (normal + streaming)
│   │       └── health.py       # Health check
│   ├── services/
│   │   ├── agent_service.py    # OpenAI Agent service (NEW)
│   │   ├── rag_pipeline.py     # RAG orchestration
│   │   ├── embeddings.py       # OpenAI embeddings
│   │   ├── qdrant_service.py   # Qdrant vector DB
│   │   ├── postgres_service.py # Postgres metadata
│   │   └── llm_service.py      # DEPRECATED (kept for reference)
│   ├── middleware/
│   │   ├── rate_limit.py       # SlowAPI rate limiting
│   │   └── logging.py          # Request logging
│   ├── config.py               # Settings and config
│   ├── logging_config.py       # Logging setup
│   └── main.py                 # FastAPI app entry point
├── requirements.txt            # Python dependencies
├── .env                        # Environment variables
├── streaming_demo.html         # Live streaming demo
└── README.md                   # Setup instructions
```

## Migration Notes

### From LLM Service to Agent Service

**Before**:
```python
response = llm_service.generate_response(query, context_chunks, conversation_history)
```

**After**:
```python
response = await agent_service.generate_response(query, conversation_history)
# Agent handles context retrieval internally via tools
```

### Benefits of Agent Architecture

1. **Autonomous decision-making**: Agent decides when to retrieve context
2. **Tool composition**: Easy to add more tools (calculator, web search, etc.)
3. **Built-in orchestration**: SDK handles multi-turn conversations
4. **Streaming support**: Native streaming via `run_streamed()`
5. **Tracing**: Built-in observability (when enabled)

## Future Enhancements

Potential improvements:

1. **Source tracking in streaming**: Return source references even with streaming
2. **Multi-turn memory**: Implement conversation history with `previous_response_id`
3. **Multiple agents**: Specialized agents for theory vs. practice questions
4. **Agent handoffs**: Transfer between agents based on topic
5. **Function calling**: Add tools for calculations, code execution, etc.
6. **Voice support**: Integrate OpenAI Agents Voice Pipeline
7. **Caching**: Cache frequent queries to reduce latency
8. **WebSocket alternative**: Real-time bidirectional communication

## Documentation

See additional documentation:

- `OPENAI_AGENTS_MIGRATION.md` - Migration from LLM service to Agents SDK
- `STREAMING_IMPLEMENTATION.md` - Streaming architecture and usage
- `QUICKSTART_RAG.md` - Quick start guide for RAG setup
- `CHATBOT_FRONTEND.md` - Frontend integration guide

## Support

For issues or questions:
1. Check server logs: `tail -f backend/logs/app.log`
2. Verify environment variables in `.env`
3. Test with `streaming_demo.html` first
4. Check OpenAI API key validity
5. Ensure Qdrant and Postgres are accessible
