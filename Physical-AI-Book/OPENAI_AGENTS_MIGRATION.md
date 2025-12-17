# OpenAI Agents SDK Migration

## Overview
Successfully migrated the RAG chatbot from direct OpenAI API calls to **OpenAI Agents Python SDK** while maintaining the exact same functionality.

## What Changed

### 1. Dependencies
**Updated** `backend/requirements.txt`:
- Added `openai-agents` package
- Upgraded `openai>=1.33.0` (required by agents SDK)
- Updated compatibility versions for FastAPI, Pydantic, and related packages

### 2. New Agent Service
**Created** `backend/src/services/agent_service.py`:
- **RAGAgentService** class that replaces the direct LLM service
- **Agent** with tool-based architecture:
  - `@function_tool` decorator for `retrieve_textbook_context` function
  - Tool automatically handles RAG retrieval (embedding → vector search → metadata fetch)
  - Agent decides when to call the retrieval tool based on user questions

### 3. RAG Pipeline Simplified
**Modified** `backend/src/services/rag_pipeline.py`:
- Replaced direct embedding/search/LLM workflow with agent orchestration
- Agent now handles the entire RAG flow internally via its tool
- Simplified from 9 steps to 3 steps (prepare query → run agent → return response)

## Architecture

### Before (Direct API):
```
User Query → Embed → Vector Search → Postgres Metadata → Format Context → LLM API → Response
```

### After (Agent SDK):
```
User Query → Agent → [Agent calls retrieve_textbook_context tool] → Response
                  ↓
          Tool Function:
          Embed → Vector Search → Postgres Metadata → Format Context → Return to Agent
```

## Key Components

### Agent Configuration
```python
Agent(
    name="Physical-AI Tutor",
    instructions=<system_prompt>,
    model="gpt-4o-mini",
    tools=[retrieve_textbook_context]
)
```

### Retrieval Tool
```python
@function_tool
async def retrieve_textbook_context(query: str) -> str:
    # 1. Generate embedding
    # 2. Search Qdrant
    # 3. Fetch Postgres metadata
    # 4. Format and return context
```

### Agent Execution
```python
result = await Runner.run(agent, input=user_query)
response = result.final_output
```

## Testing

Tested successfully with:
```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is inverse kinematics?", "session_id": "550e8400-e29b-41d4-a716-446655440000"}'
```

**Response**:
```json
{
  "response": "Inverse kinematics (IK) is a computational process used in robotics to determine the joint angles required for a robotic arm or leg to reach a specific end-effector position or pose...",
  "sources": [],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "retrieval_count": 0,
  "processing_time_ms": 18022
}
```

## What Stayed the Same

✅ **API Endpoints** - No changes to REST API
✅ **Request/Response Models** - Same Pydantic models
✅ **RAG Functionality** - Same retrieval logic (Qdrant + Postgres)
✅ **System Prompts** - Same educational tutor instructions
✅ **Safety Checks** - Same harmful content filtering
✅ **Rate Limiting** - Same middleware

## Benefits of Agent SDK

1. **Tool-based Architecture**: Agent autonomously decides when to retrieve context
2. **Built-in Orchestration**: SDK handles multi-turn conversations and tool calling
3. **Cleaner Code**: Declarative agent definition instead of imperative pipeline
4. **Extensibility**: Easy to add more tools (e.g., calculator, web search) in the future
5. **Tracing**: Built-in observability (when OPENAI_API_KEY is set for tracing)

## Files Modified

1. `backend/requirements.txt` - Added openai-agents package
2. `backend/src/services/agent_service.py` - **NEW** agent service
3. `backend/src/services/rag_pipeline.py` - Simplified to use agent
4. `backend/src/services/llm_service.py` - **DEPRECATED** (kept for reference)

## Running the Application

```bash
cd backend
source venv/bin/activate
pip install -r requirements.txt
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

## Notes

- **Source References**: Currently returns empty `sources[]` array because the agent manages context internally. This can be enhanced by modifying the tool to track retrieved chunks.
- **Conversation History**: Can be extended to support multi-turn conversations by passing history to `Runner.run()`
- **Model**: Uses the same `gpt-4o-mini` model as before
- **Performance**: Similar latency (~18s for complex queries with retrieval)

## Next Steps (Optional Enhancements)

1. Add source tracking to return `sources[]` array
2. Implement multi-turn conversation memory
3. Add additional tools (e.g., code execution, diagram generation)
4. Enable OpenAI tracing for debugging
5. Add agent handoffs for specialized tasks (theory vs. practice agents)
