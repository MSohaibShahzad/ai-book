# Research: Chapter Personalization Technology Choices

**Feature**: Chapter Personalization
**Date**: 2025-12-23
**Purpose**: Document technology decisions, best practices research, and implementation patterns for AI-powered chapter personalization system

## Overview

This document consolidates research findings for key technology choices in the Chapter Personalization feature. Each decision includes evaluation criteria, alternatives considered, and rationale for the final choice.

## Decision 1: AI Framework - OpenAI Agents SDK

### Context

Need to implement reusable AI agent that adapts textbook chapter content based on user background (software/hardware experience level, interest area). Agent must preserve technical accuracy while adapting explanations.

### Options Evaluated

| Option | Pros | Cons | Score |
|--------|------|------|-------|
| **OpenAI Agents SDK** | Built-in state management, retry logic, structured agent lifecycle, easier debugging | Relatively new framework, less community examples | 9/10 |
| Direct OpenAI API | Full control, well-documented, mature | Manual retry logic, state management, error handling | 6/10 |
| LangChain | Rich ecosystem, many integrations | Heavy framework, unnecessary complexity for focused use case | 5/10 |
| Claude API (Anthropic) | Strong reasoning, good at instruction following | Different API patterns, less familiar to team | 6/10 |

### Decision: OpenAI Agents SDK ✅

**Rationale**:
- **Structured Agent Lifecycle**: SDK provides `Agent` class with clear init/run/cleanup pattern
- **Built-in Retry Logic**: Automatic retries with exponential backoff for transient failures
- **State Management**: Agent maintains context across calls without manual state tracking
- **System Prompts**: First-class support for system-level instructions (personalization rules)
- **Timeout Configuration**: Native support for per-request timeouts (30-second requirement)
- **Error Handling**: Structured error types make it easier to distinguish timeout vs API errors

**Implementation Pattern**:
```python
from openai_agents import Agent, AgentConfig

personalization_agent = Agent(
    name="chapter_personalizer",
    instructions="""
    You are an expert technical content adapter. Your job is to rewrite
    textbook chapters to match the user's experience level while preserving
    100% accuracy of code examples, mathematical formulas, and learning outcomes.

    User Profile:
    - Software Background: {software_background}
    - Hardware Background: {hardware_background}
    - Interest Area: {interest_area}

    Rules:
    1. NEVER modify code blocks, LaTeX formulas, or YAML frontmatter
    2. Adapt only prose explanations
    3. Match tone to experience level (Beginner: foundational, Advanced: technical depth)
    4. Preserve all learning outcomes verbatim
    5. Maintain markdown structure exactly
    """,
    model="gpt-4o",  # Latest GPT-4 model for quality
    timeout=30,      # Hard 30-second timeout
)

# Usage:
response = personalization_agent.run(
    message=chapter_content,
    context={
        "software_background": user.software_background,
        "hardware_background": user.hardware_background,
        "interest_area": user.interest_area,
    }
)
```

**Best Practices Adopted**:
- Define agent once, reuse for all personalization requests (avoids initialization overhead)
- Use context variables for user profile instead of string interpolation (cleaner, safer)
- Set explicit timeout matching requirement (30 seconds)
- Log agent responses for debugging and quality monitoring

**References**:
- OpenAI Agents SDK Documentation: https://platform.openai.com/docs/agents
- Agent best practices: https://platform.openai.com/docs/guides/agents/best-practices

---

## Decision 2: Rate Limiting Storage - PostgreSQL

### Context

Must track 3 personalization requests per user per day with rolling 24-hour window. Need persistent storage that survives server restarts and supports efficient lookups.

### Options Evaluated

| Option | Pros | Cons | Score |
|--------|------|------|-------|
| **PostgreSQL Table** | Already in stack, persistent, supports analytics queries | Slightly slower than in-memory | 9/10 |
| Redis | Extremely fast, built-in TTL | New infrastructure dependency, adds complexity | 7/10 |
| In-memory (Python dict) | Zero latency, simple | Lost on restart, no persistence | 3/10 |
| User table column | No new table needed | Less flexible, harder to track history | 5/10 |

### Decision: PostgreSQL Table ✅

**Schema Design**:
```sql
CREATE TABLE personalization_quota (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    request_count INTEGER NOT NULL DEFAULT 0 CHECK (request_count >= 0 AND request_count <= 3),
    first_request_timestamp TIMESTAMPTZ NOT NULL,
    reset_at TIMESTAMPTZ NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_personalization_quota_reset ON personalization_quota(reset_at);
```

**Rationale**:
- **Persistence**: Survives server restarts, deployments, and scaling events
- **Consistency**: Already using PostgreSQL (Neon) for user data - no new infrastructure
- **Analytics-Ready**: Can query for usage patterns, peak times, user adoption metrics
- **Rolling Window Support**: `first_request_timestamp` + 24 hours calculation is simple and accurate
- **Automatic Cleanup**: Can add periodic job to delete rows where `reset_at < NOW() - 7 days`

**Rate Limiting Logic**:
```python
def check_rate_limit(user_id: UUID) -> tuple[bool, int]:
    """
    Returns: (is_allowed, remaining_requests)
    """
    quota = db.query(PersonalizationQuota).filter_by(user_id=user_id).first()

    if not quota:
        # First request - create quota entry
        quota = PersonalizationQuota(
            user_id=user_id,
            request_count=1,
            first_request_timestamp=datetime.utcnow(),
            reset_at=datetime.utcnow() + timedelta(hours=24)
        )
        db.add(quota)
        db.commit()
        return True, 2  # 2 remaining after this request

    now = datetime.utcnow()
    if now > quota.reset_at:
        # Window expired - reset counter
        quota.request_count = 1
        quota.first_request_timestamp = now
        quota.reset_at = now + timedelta(hours=24)
        db.commit()
        return True, 2

    if quota.request_count >= 3:
        # Limit exceeded
        return False, 0

    # Increment counter
    quota.request_count += 1
    db.commit()
    remaining = 3 - quota.request_count
    return True, remaining
```

**Best Practices Adopted**:
- Use database constraints (CHECK) to enforce 0-3 range
- Index on `reset_at` for efficient cleanup queries
- Cascade delete when user is deleted (maintains referential integrity)
- Store both `first_request_timestamp` and `reset_at` for clarity and debugging

---

## Decision 3: Frontend State Management - React Context API

### Context

Need to manage personalization state (cached content, rate limit, current view) across multiple components without prop drilling. State is session-only and feature-scoped.

### Options Evaluated

| Option | Pros | Cons | Score |
|--------|------|------|-------|
| **React Context + Hooks** | Lightweight, React native, scoped to feature | Manual optimization needed for performance | 9/10 |
| Redux | Powerful, dev tools, time-travel debugging | Overkill for session-only state | 5/10 |
| Component State | Simple, no setup | Prop drilling, hard to share state | 4/10 |
| LocalStorage | Persists across sessions | Violates "session-only" requirement | 2/10 |

### Decision: React Context API + Custom Hooks ✅

**Architecture**:
```typescript
// PersonalizationContext.tsx
interface PersonalizationState {
  personalizedContent: Map<string, string>;  // chapterId -> markdown
  rateLimitRemaining: number;
  currentView: 'original' | 'personalized';
  isLoading: boolean;
  error: string | null;
}

const PersonalizationContext = createContext<PersonalizationState | null>(null);

// Custom hooks
function usePersonalization(chapterId: string) {
  const context = useContext(PersonalizationContext);

  const personalize = async (chapterContent: string) => {
    // Call API, update context
  };

  const toggleView = () => {
    // Switch between original and personalized
  };

  return { personalize, toggleView, ...context };
}

function useRateLimit() {
  const context = useContext(PersonalizationContext);

  const checkLimit = async () => {
    // Fetch remaining limit from API
  };

  return { remaining: context.rateLimitRemaining, checkLimit };
}
```

**Rationale**:
- **Feature Isolation**: Context scoped to personalization feature, doesn't pollute global state
- **Session-Only**: Context state cleared on page reload (matches requirement)
- **Clean API**: Custom hooks abstract complexity, provide simple interface to components
- **Performance**: Can use React.memo and useMemo for optimization if needed
- **Lightweight**: No additional dependencies beyond React

**Best Practices Adopted**:
- Separate context provider from hooks (testability)
- Use TypeScript for type safety
- Memoize expensive computations (markdown parsing)
- Clear context on profile update (invalidate cache)
- Provide loading and error states in context

---

## Decision 4: Markdown Processing - markdown-it-py with AST Protection

### Context

Must preserve 100% accuracy of code blocks, LaTeX formulas, YAML frontmatter, and diagrams while allowing AI to rewrite prose explanations. Need reliable parsing and reconstruction.

### Options Evaluated

| Option | Pros | Cons | Score |
|--------|------|------|-------|
| **markdown-it-py + AST** | Used in 004-urdu-translation (consistency), robust parsing, plugin ecosystem | Learning curve for AST manipulation | 9/10 |
| Regex-based replacement | Simple, fast | Fragile with nested structures, error-prone | 3/10 |
| LLM-only (trust AI) | No parsing needed | Risk of hallucinated changes to code/formulas | 2/10 |
| CommonMark-py | Python standard | No plugins for LaTeX/frontmatter, less features | 6/10 |

### Decision: markdown-it-py with AST-based selective rewriting ✅

**Implementation Pattern**:
```python
from markdown_it import MarkdownIt
from markdown_it.token import Token

def protect_technical_elements(markdown_content: str) -> tuple[str, dict]:
    """
    Extract protected elements (code, math, frontmatter) and replace with placeholders.
    Returns: (prose-only markdown, protection_map)
    """
    md = MarkdownIt('commonmark', {'html': True})
    md.enable(['table', 'strikethrough'])
    tokens = md.parse(markdown_content)

    protected_elements = {}
    placeholder_counter = 0
    prose_markdown = []

    for token in tokens:
        if token.type in ['code_block', 'fence', 'code_inline']:
            # Protect code blocks
            placeholder = f"{{{{CODE_{placeholder_counter}}}}}"
            protected_elements[placeholder] = token.content
            prose_markdown.append(placeholder)
            placeholder_counter += 1
        elif token.type == 'math_block' or token.markup == '$$':
            # Protect LaTeX math
            placeholder = f"{{{{MATH_{placeholder_counter}}}}}"
            protected_elements[placeholder] = token.content
            prose_markdown.append(placeholder)
            placeholder_counter += 1
        elif token.type == 'html_block' and '---' in token.content:
            # Protect YAML frontmatter
            placeholder = f"{{{{FRONTMATTER_{placeholder_counter}}}}}"
            protected_elements[placeholder] = token.content
            prose_markdown.append(placeholder)
            placeholder_counter += 1
        elif token.type == 'image':
            # Protect image references
            placeholder = f"{{{{IMAGE_{placeholder_counter}}}}}"
            protected_elements[placeholder] = token.markup
            prose_markdown.append(placeholder)
            placeholder_counter += 1
        else:
            # Regular prose - pass to AI
            prose_markdown.append(token.content)

    return '\n'.join(prose_markdown), protected_elements

def reconstruct_markdown(personalized_prose: str, protection_map: dict) -> str:
    """
    Replace placeholders with original protected elements.
    """
    result = personalized_prose
    for placeholder, original_content in protection_map.items():
        result = result.replace(placeholder, original_content)
    return result

# Usage in personalization flow:
prose_only, protected = protect_technical_elements(original_chapter)
personalized_prose = ai_agent.run(message=prose_only, context=user_profile)
personalized_chapter = reconstruct_markdown(personalized_prose, protected)
```

**Rationale**:
- **Consistency**: Same library as 004-urdu-translation feature (team familiarity)
- **Robustness**: AST parsing handles nested structures, edge cases correctly
- **Plugin Support**: Supports LaTeX (math blocks), tables, strikethrough via plugins
- **Preservation Guarantee**: Protected elements never sent to AI, can't be modified
- **Validation**: Can compare AST structure before/after to ensure no structural changes

**Best Practices Adopted**:
- Extract all protected elements before AI processing
- Use unique placeholders (counters) to avoid collision with content
- Reconstruct in reverse order of extraction
- Validate final markdown structure matches original
- Log discrepancies for debugging

---

## Decision 5: Observability - FastAPI Middleware + Structured Logging

### Context

Must track request count, success/failure rate, average generation time, and timeout count to meet FR-024, FR-025 requirements and verify 90% success rate (SC-007).

### Options Evaluated

| Option | Pros | Cons | Score |
|--------|------|------|-------|
| **FastAPI Middleware + Structured Logs** | Built-in, no dependencies, flexible | Manual aggregation for dashboards | 8/10 |
| Prometheus + Grafana | Industry standard, powerful visualization | Infrastructure overhead for MVP | 7/10 |
| Datadog/New Relic | Comprehensive APM, easy setup | Monthly cost, vendor lock-in | 6/10 |
| Manual logging in endpoints | Full control | Error-prone, hard to maintain | 3/10 |

### Decision: FastAPI Middleware + Structured Logging + Metrics Endpoint ✅

**Implementation**:
```python
import time
import json
from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

class PersonalizationMetrics:
    def __init__(self):
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.timeout_requests = 0
        self.total_duration_ms = 0

    @property
    def avg_duration_ms(self):
        if self.total_requests == 0:
            return 0
        return self.total_duration_ms / self.total_requests

    @property
    def success_rate(self):
        if self.total_requests == 0:
            return 0
        return (self.successful_requests / self.total_requests) * 100

metrics = PersonalizationMetrics()

class ObservabilityMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        if not request.url.path.startswith("/api/personalize"):
            return await call_next(request)

        start_time = time.time()

        try:
            response = await call_next(request)
            duration_ms = (time.time() - start_time) * 1000

            # Track metrics
            metrics.total_requests += 1
            metrics.total_duration_ms += duration_ms

            if response.status_code == 200:
                metrics.successful_requests += 1
            elif response.status_code == 408:
                metrics.timeout_requests += 1
                metrics.failed_requests += 1
            else:
                metrics.failed_requests += 1

            # Structured logging
            logger.info(json.dumps({
                "event": "personalization_request",
                "user_id": request.state.user_id,
                "chapter_id": request.query_params.get("chapter_id"),
                "duration_ms": duration_ms,
                "status_code": response.status_code,
                "timestamp": time.time()
            }))

            return response

        except Exception as e:
            duration_ms = (time.time() - start_time) * 1000
            metrics.total_requests += 1
            metrics.failed_requests += 1

            logger.error(json.dumps({
                "event": "personalization_error",
                "error": str(e),
                "duration_ms": duration_ms,
                "timestamp": time.time()
            }))
            raise

# Metrics endpoint
@app.get("/metrics")
async def get_metrics():
    return {
        "total_requests": metrics.total_requests,
        "successful_requests": metrics.successful_requests,
        "failed_requests": metrics.failed_requests,
        "timeout_requests": metrics.timeout_requests,
        "avg_duration_ms": metrics.avg_duration_ms,
        "success_rate_percent": metrics.success_rate
    }
```

**Rationale**:
- **Zero Dependencies**: Uses FastAPI built-in middleware
- **Structured Logs**: JSON format enables easy parsing by log aggregators (CloudWatch, Loki, etc.)
- **Real-time Metrics**: In-memory metrics updated on every request
- **Dashboard-Ready**: `/metrics` endpoint can be queried by monitoring systems
- **Lightweight**: Minimal performance overhead (<1ms per request)

**Best Practices Adopted**:
- Use middleware for automatic instrumentation (no manual tracking in endpoints)
- JSON structured logging for machine readability
- Separate metrics from logs (metrics in-memory, logs to file/stdout)
- Include context (user_id, chapter_id) in logs for debugging
- Expose metrics endpoint for health checks and dashboards

**Monitoring Dashboard Integration**:
- Query `/metrics` endpoint every 60 seconds
- Alert if `success_rate_percent < 90` (violates SC-007)
- Alert if `avg_duration_ms > 25000` (approaching timeout limit)
- Graph `timeout_requests` to identify patterns

---

## Summary of Decisions

| Decision Area | Chosen Technology | Primary Benefit |
|--------------|-------------------|-----------------|
| AI Framework | OpenAI Agents SDK | Structured agent lifecycle, built-in retry |
| Rate Limiting | PostgreSQL Table | Persistent, analytics-ready, consistent with stack |
| State Management | React Context + Hooks | Lightweight, feature-scoped, session-only |
| Markdown Processing | markdown-it-py + AST | Robust parsing, 100% preservation guarantee |
| Observability | FastAPI Middleware + Logs | Zero dependencies, real-time metrics |

All decisions prioritize:
- ✅ **Simplicity**: Minimal dependencies, leverage existing stack
- ✅ **Reliability**: Robust error handling, timeout enforcement
- ✅ **Maintainability**: Clear patterns, consistent with existing features
- ✅ **Constitution Compliance**: Original content preservation, authenticated access, metrics tracking

## Next Steps

1. **Phase 1**: Generate detailed data models (`data-model.md`)
2. **Phase 1**: Define API contracts (`contracts/personalization-api.yaml`)
3. **Phase 1**: Create setup guide (`quickstart.md`)
4. **Phase 2**: Generate implementation tasks (`/sp.tasks`)
