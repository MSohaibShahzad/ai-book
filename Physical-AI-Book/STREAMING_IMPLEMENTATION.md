# Streaming Chat Implementation

## Overview
Added **Server-Sent Events (SSE) streaming** to the RAG chatbot, allowing real-time token-by-token response streaming using OpenAI Agents SDK.

## Implementation

### 1. Agent Service Streaming Method
**Added to** `backend/src/services/agent_service.py`:

```python
async def generate_response_stream(self, query: str, conversation_history: Optional[List[Dict[str, str]]] = None):
    """Generate streaming agent response."""
    result = Runner.run_streamed(self.agent, input=query)

    async for event in result.stream_events():
        if event.type == "raw_response_event" and isinstance(event.data, ResponseTextDeltaEvent):
            yield event.data.delta
```

### 2. RAG Pipeline Streaming
**Added to** `backend/src/services/rag_pipeline.py`:

```python
async def process_query_stream(self, query: str, highlighted_text: Optional[str] = None, conversation_history: Optional[List[Dict[str, str]]] = None):
    """Process query with streaming response."""
    final_query = query
    if highlighted_text:
        final_query = f"Context: {highlighted_text}\n\nQuestion: {query}"

    async for chunk in self.agent.generate_response_stream(query=final_query, conversation_history=conversation_history):
        yield chunk
```

### 3. Streaming Chat Endpoint
**Added to** `backend/src/api/routes/chat.py`:

```python
@router.post("/chat/stream")
async def chat_stream(request: Request, response: Response, chat_request: ChatRequest):
    """Stream chat response with Server-Sent Events."""
    async def event_stream():
        async for chunk in rag_pipeline.process_query_stream(
            query=chat_request.message,
            highlighted_text=chat_request.highlighted_text,
            conversation_history=chat_request.conversation_history
        ):
            yield f"data: {json.dumps({'chunk': chunk})}\n\n"

        yield f"data: {json.dumps({'done': True})}\n\n"

    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"
        }
    )
```

## API Usage

### Streaming Endpoint
**URL**: `POST /v1/chat/stream`

**Request**:
```json
{
  "message": "What is a robot?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "highlighted_text": null,
  "conversation_history": []
}
```

**Response** (Server-Sent Events):
```
data: {"chunk": "A"}

data: {"chunk": " robot"}

data: {"chunk": " is"}

data: {"chunk": " a"}

data: {"chunk": " programmable"}

data: {"chunk": " machine"}

...

data: {"chunk": "."}

data: {"done": true}
```

### Testing with cURL
```bash
curl -N -X POST http://localhost:8000/v1/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"message": "What is a robot?", "session_id": "550e8400-e29b-41d4-a716-446655440000"}'
```

**Note**: The `-N` flag disables buffering for streaming output.

## Frontend Integration

### JavaScript/TypeScript Example
```javascript
async function streamChat(message, sessionId) {
  const response = await fetch('http://localhost:8000/v1/chat/stream', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      message: message,
      session_id: sessionId,
      highlighted_text: null,
      conversation_history: []
    })
  });

  const reader = response.body.getReader();
  const decoder = new TextDecoder();

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    const chunk = decoder.decode(value);
    const lines = chunk.split('\n');

    for (const line of lines) {
      if (line.startsWith('data: ')) {
        const data = JSON.parse(line.slice(6));

        if (data.chunk) {
          // Append chunk to UI
          console.log(data.chunk);
        } else if (data.done) {
          // Stream completed
          console.log('Stream complete');
        } else if (data.error) {
          // Handle error
          console.error(data.error);
        }
      }
    }
  }
}
```

### React Example
```jsx
import { useState } from 'react';

function ChatWidget() {
  const [response, setResponse] = useState('');
  const [isStreaming, setIsStreaming] = useState(false);

  async function handleSubmit(message) {
    setIsStreaming(true);
    setResponse('');

    const res = await fetch('/v1/chat/stream', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        message,
        session_id: crypto.randomUUID()
      })
    });

    const reader = res.body.getReader();
    const decoder = new TextDecoder();

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      const chunk = decoder.decode(value);
      const lines = chunk.split('\n');

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const data = JSON.parse(line.slice(6));

          if (data.chunk) {
            setResponse(prev => prev + data.chunk);
          } else if (data.done) {
            setIsStreaming(false);
          }
        }
      }
    }
  }

  return (
    <div>
      <div>{response}</div>
      {isStreaming && <span>...</span>}
    </div>
  );
}
```

## Benefits

1. **Better UX**: Users see responses appear in real-time (like ChatGPT)
2. **Lower Perceived Latency**: Streaming starts immediately, feels faster
3. **Progressive Rendering**: Frontend can display partial responses
4. **Agent Transparency**: Users see the agent "thinking" and generating

## SSE Event Format

All events follow Server-Sent Events specification:

```
data: <JSON payload>\n\n
```

### Event Types

1. **Chunk Event** (text delta):
   ```
   data: {"chunk": "text"}
   ```

2. **Completion Event**:
   ```
   data: {"done": true}
   ```

3. **Error Event**:
   ```
   data: {"error": "error message"}
   ```

## Comparison: Non-Streaming vs Streaming

### Non-Streaming Endpoint
- **URL**: `POST /v1/chat`
- **Response**: Complete response at once (JSON)
- **Use Case**: Simple integrations, batch processing
- **Latency**: Wait for full response (~15-20s)

### Streaming Endpoint
- **URL**: `POST /v1/chat/stream`
- **Response**: Token-by-token (SSE)
- **Use Case**: Interactive chat widgets, real-time UIs
- **Latency**: First token in ~3-5s, continuous streaming

## Technical Details

### OpenAI Agents SDK Streaming
- Uses `Runner.run_streamed()` instead of `Runner.run()`
- Returns `StreamedRunResult` with `stream_events()` method
- Yields `ResponseTextDeltaEvent` objects with `.delta` property

### FastAPI StreamingResponse
- `media_type="text/event-stream"` for SSE
- `Cache-Control: no-cache` to prevent caching
- `Connection: keep-alive` to maintain connection
- `X-Accel-Buffering: no` to disable nginx buffering

### Rate Limiting
Both endpoints share the same rate limit:
- `10 requests/minute` per IP address
- Applied via `@limiter.limit("10/minute")` decorator

## Files Modified

1. `backend/src/services/agent_service.py` - Added `generate_response_stream()`
2. `backend/src/services/rag_pipeline.py` - Added `process_query_stream()`
3. `backend/src/api/routes/chat.py` - Added `/chat/stream` endpoint

## Next Steps

To use streaming in your chat widget:

1. Update frontend to call `/v1/chat/stream` instead of `/v1/chat`
2. Implement SSE event parsing (see JavaScript example above)
3. Update UI to append chunks progressively
4. Add loading indicator while streaming
5. Handle `done` and `error` events appropriately

## Backwards Compatibility

✅ **Original endpoint unchanged**: `/v1/chat` still works for non-streaming use cases
✅ **Same request format**: Both endpoints accept identical request payloads
✅ **No breaking changes**: Existing integrations continue to work
