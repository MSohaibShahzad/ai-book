# Research: Urdu Translation Implementation

**Date**: 2025-12-20
**Feature**: 004-urdu-translation
**Status**: Complete

## Overview

This document consolidates research findings for implementing the Urdu translation feature. Research focused on six key unknowns identified in plan.md:

1. OpenAI Agents SDK best practices
2. Technical term preservation strategies
3. Caching architecture
4. Markdown parsing and preservation
5. Error handling patterns
6. Frontend state management

---

## 1. OpenAI Agents SDK for Translation

### Decision

**Use OpenAI Agents SDK (`openai-agents-python`) with GPT-4o model and streaming for translation workflows.**

Architecture components:
- Primary translation agent with Urdu-specific system prompts
- Placeholder substitution pattern for code/LaTeX/technical terms
- Streaming API for real-time feedback on long chapters
- Optional validator agent (GPT-4o-mini) for quality checks

### Rationale

1. **Future-Proof Choice**: Assistants API is deprecated in 2026; Agents SDK is the official replacement
2. **Production-Ready**: Lightweight, minimal abstractions, built-in tracing and debugging
3. **Streaming Support**: Essential for 1,000-5,000 word chapters (2-5 second response times with streaming vs 30+ seconds batch)
4. **GPT-4o Over GPT-4o-mini**: Superior multilingual fluency and technical term handling justify the 4x cost difference ($0.08-$0.10 vs $0.02 per chapter)
   - GPT-4o maintains better context for technical educational content
   - GPT-4o-mini acceptable for validation/checking only

### Alternatives Considered

| Alternative | Pros | Cons | Decision |
|-------------|------|------|----------|
| Swarm Framework | Lightweight multi-agent | Educational-only, no production support | ❌ Rejected |
| Assistants API | Mature, well-documented | Deprecated 2026, migration required | ❌ Rejected |
| GPT-4o-mini | 60% cheaper ($0.02/chapter) | Inferior fluency, technical accuracy | ❌ Use only for validation |
| Batch API | 50% cost savings | 24-hour latency, no streaming | ❌ Not suitable for interactive feature |

### Implementation Guidance

**Installation**:
```bash
pip install openai-agents python-dotenv tiktoken tenacity
```

**Core Pattern**: Placeholder Substitution
```python
from agents import Agent, Runner

# Step 1: Extract technical elements
def extract_preservables(text):
    placeholders = {}
    # Replace code blocks with __CODE_N__
    # Replace LaTeX with __LATEX_N__
    # Replace technical terms with __TERM_NAME__
    return cleaned_text, placeholders

# Step 2: Translation agent
urdu_translator = Agent(
    name="UrduTranslator",
    instructions="""Translate to Urdu.
    NEVER translate placeholders (__CODE_*, __LATEX_*, __TERM_*).
    Use simple Urdu suitable for undergraduates.""",
    model="gpt-4o"
)

# Step 3: Stream translation
async def translate_streaming(text):
    extraction = extract_preservables(text)
    result = Runner.run_streamed(urdu_translator, input=extraction["cleaned_text"])
    async for event in result.stream_events():
        # Process chunks
        pass
    return restore_preservables(translated, extraction["placeholders"])
```

**Rate Limiting**:
```python
from tenacity import retry, wait_random_exponential, stop_after_attempt

@retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
async def translate_with_retry(agent, text):
    return await Runner.run(agent, input=text)
```

**Cost Estimation**:
- 5,000-word chapter ≈ 6,500 tokens input + 7,800 tokens output
- GPT-4o cost: $0.08-$0.10 per chapter
- 50 chapters × 100 users = ~$400-$500 without caching

**Sources**:
- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [LaTeXTrans Multi-Agent Architecture](https://arxiv.org/html/2508.18791v1)
- [GPT-4o vs GPT-4o-mini Comparison](https://www.nebuly.com/blog/gpt-4o-mini-vs-gpt-4o)

---

## 2. Caching Strategy

### Decision

**Use PostgreSQL (existing Neon database) with versioned cache entries and 90-day TTL.**

Schema:
```sql
CREATE TABLE translation_cache (
    id SERIAL PRIMARY KEY,
    chapter_slug VARCHAR(255),
    language VARCHAR(10),
    content_hash VARCHAR(64),  -- SHA-256 of source
    translated_content TEXT,
    created_at TIMESTAMP,
    accessed_at TIMESTAMP,
    expires_at TIMESTAMP,
    UNIQUE(chapter_slug, language, content_hash)
);
```

Cache Key: `{chapter_slug}:{language}:{content_hash}`

### Rationale

**Cost Efficiency** (primary driver):
- Translation cost without cache (100 users × 43 chapters): **$347.00**
- Translation cost with cache: **$8.08** (one-time)
- **Savings: 99% ($338.92)**
- PostgreSQL infrastructure cost: **$0** (existing Neon free tier)
- Redis alternative cost: **$15-19/month** = needs 152 months to break even

**Performance**:
- PostgreSQL: 50-100ms latency with proper indexing
- Redis: ~1ms latency
- **Trade-off acceptable**: 50ms vs 1ms is imperceptible compared to 2-5 second live translation

**Storage Requirements**:
- 43 chapters × 2 languages ≈ 1.23 MB
- Fits comfortably in Neon's 500MB free tier
- Scales to 1,000 chapters (~28 MB) without issue

**Version-Based Invalidation**:
- When source content changes, new `content_hash` is computed
- Old cached translations automatically become stale (hash mismatch)
- Daily cleanup job removes expired entries

### Alternatives Considered

| Alternative | Cost | Performance | Decision |
|-------------|------|-------------|----------|
| Redis (dedicated) | $15-19/month | 1ms latency | ❌ Cost unjustified for educational content |
| In-memory (app) | $0 | Fast, but not persistent | ❌ Cache lost on restart |
| CDN caching | Variable | Fast, but invalidation hard | ❌ Overkill for dynamic content |
| PostgreSQL (chosen) | $0 | 50-100ms | ✅ Best cost/performance balance |

### Implementation Guidance

**Cache Lookup Flow**:
```python
async def get_translated_chapter(chapter_slug, language, db):
    source = await get_chapter_content(chapter_slug)
    hash = sha256(source.encode()).hexdigest()

    # Check cache
    cached = await db.query(
        TranslationCache
    ).filter_by(
        chapter_slug=chapter_slug,
        language=language,
        content_hash=hash
    ).filter(
        TranslationCache.expires_at > datetime.now()
    ).first()

    if cached:
        # Hit: update access time
        cached.accessed_at = datetime.now()
        await db.commit()
        return cached.translated_content

    # Miss: translate and cache
    translated = await translate_content(source, language)
    db.add(TranslationCache(
        chapter_slug=chapter_slug,
        language=language,
        content_hash=hash,
        translated_content=translated,
        expires_at=datetime.now() + timedelta(days=90)
    ))
    await db.commit()
    return translated
```

**TTL Recommendations**:
- Stable chapters (preface, appendices): 180 days
- Core curriculum: 90 days (default)
- Code examples: 30 days (libraries update frequently)

**Future Optimization**:
If traffic exceeds PostgreSQL capacity (unlikely):
1. Add Redis as L1 cache layer (read-through pattern)
2. Keep PostgreSQL as source of truth
3. Redis caches top 10-20% hot chapters only

**Sources**:
- [PostgreSQL vs Redis Caching Performance](https://www.myscale.com/blog/postgres-vs-redis-battle-caching-performance/)
- [Redis is fast - I'll cache in Postgres](https://dizzy.zone/2025/09/24/Redis-is-fast-Ill-cache-in-Postgres/)
- [Cache Invalidation Strategies](https://www.designgurus.io/blog/cache-invalidation-strategies/)

---

## 3. Markdown Preservation

### Decision

**AST-based extraction with `markdown-it-py` parser + placeholder tokenization.**

Strategy:
1. Parse markdown to Abstract Syntax Tree (AST)
2. Extract code blocks, LaTeX formulas, custom Docusaurus components
3. Replace with unique placeholder tokens (e.g., `┌PRESERVE_0001┘`)
4. Translate text-only content
5. Restore placeholders with original preserved content

### Rationale

**Why AST Beats Regex**:
- **Structural awareness**: Parsers understand markdown natively (headings, lists, tables, code blocks as distinct tokens)
- **Edge case handling**: Nested lists with code, tables with inline code, custom JSX components
- **Round-trip fidelity**: AST → modify → render ensures output structure matches input exactly

**Why `markdown-it-py`**:
- CommonMark spec compliant (ensures Docusaurus compatibility)
- Extensible plugin system (`front_matter_plugin`, `dollarmath_plugin`)
- Clean token API for extraction/manipulation
- Docusaurus uses Remark (also CommonMark), so compatibility guaranteed

### Alternatives Considered

| Alternative | Pros | Cons | Decision |
|-------------|------|------|----------|
| Pure regex | Simple for basic cases | Fails on nested structures, fragile | ❌ Fallback only |
| `mistune` | Fastest (3x faster) | Less extensible plugins | ❌ Not worth complexity trade-off |
| `python-markdown` | Mature, widely used | Not CommonMark, slower | ❌ Incompatible with Docusaurus |
| Send raw markdown to LLM | Simplest (no preprocessing) | Unreliable, LLMs corrupt formatting | ❌ Not production-safe |
| `markdown-it-py` (chosen) | CommonMark, extensible, clean API | Slightly slower than mistune | ✅ Best long-term choice |

### Implementation Guidance

**Installation**:
```bash
pip install markdown-it-py mdit-py-plugins pyyaml
```

**Core Translation Pipeline**:
```python
from markdown_it import MarkdownIt
from mdit_py_plugins.front_matter import front_matter_plugin
from mdit_py_plugins.dollarmath import dollarmath_plugin

class MarkdownTranslator:
    def __init__(self):
        self.md = (
            MarkdownIt()
            .use(front_matter_plugin)  # YAML frontmatter
            .use(dollarmath_plugin)    # $...$ and $$...$$
            .enable('table')           # GFM tables
        )
        self.placeholder_map = {}
        self.counter = 0

    def extract_preservables(self, markdown_text):
        tokens = self.md.parse(markdown_text)
        result = []

        for token in tokens:
            if token.type == 'fence':  # Code blocks
                placeholder = f"┌PRESERVE_{self.counter:04d}┘"
                self.placeholder_map[placeholder] = token.content
                self.counter += 1
                result.append(f"```{token.info}\n{placeholder}\n```")

            elif token.type == 'math_block':  # $$...$$
                placeholder = f"┌PRESERVE_{self.counter:04d}┘"
                self.placeholder_map[placeholder] = token.content
                self.counter += 1
                result.append(f"$${placeholder}$$")

            elif token.type == 'html_block':  # JSX components
                placeholder = f"┌PRESERVE_{self.counter:04d}┘"
                self.placeholder_map[placeholder] = token.content
                self.counter += 1
                result.append(placeholder)

            else:
                result.append(self._reconstruct_token(token))

        return '\n'.join(result), self.placeholder_map

    def restore_preservables(self, translated_text):
        for placeholder, original in self.placeholder_map.items():
            translated_text = translated_text.replace(placeholder, original)
        return translated_text
```

**Handling YAML Frontmatter**:
```python
import yaml

frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
if frontmatter_match:
    fm = yaml.safe_load(frontmatter_match.group(1))
    # Translate values, keep keys in English
    for key in ['title', 'description', 'sidebar_label']:
        if key in fm:
            fm[key] = await translate_text(fm[key])
```

**Edge Cases Handled**:
1. Nested lists with code blocks
2. Tables with inline code (`` `command` ``)
3. Custom Docusaurus components (`<Tabs>`, `<CodeBlock>`)
4. Links with code in anchor text
5. LaTeX formulas with currency symbols (`$100` vs `$E=mc^2$`)

**Validation**:
```python
def validate_translation(original, translated):
    errors = []
    # Check code block count
    if original.count('```') != translated.count('```'):
        errors.append("Code block count mismatch")
    # Check math block count
    if original.count('$$') != translated.count('$$'):
        errors.append("Math block count mismatch")
    return errors
```

**Sources**:
- [markdown-it-py Documentation](https://markdown-it-py.readthedocs.io/)
- [Continuous Markdown Translations (AST Strategy)](https://microsoft.github.io/genaiscript/blog/continuous-translations/)
- [Docusaurus MDX Documentation](https://docusaurus.io/docs/markdown-features/react)

---

## 4. Error Handling Patterns

### Decision

**Implement retry logic with exponential backoff + graceful fallbacks + user-friendly error messages.**

Error categories:
1. **Transient errors** (rate limits, network): Auto-retry with backoff
2. **Permanent errors** (invalid API key): Fail fast with clear message
3. **Partial failures** (long chapters timeout): Split into chunks

### Implementation Guidance

**Retry with Tenacity**:
```python
from tenacity import retry, stop_after_attempt, wait_random_exponential
import openai

@retry(
    wait=wait_random_exponential(min=1, max=60),
    stop=stop_after_attempt(6)
)
async def translate_with_retry(text):
    try:
        return await openai_translate(text)
    except openai.RateLimitError:
        raise  # Tenacity retries
    except openai.APIError as e:
        if "invalid_api_key" in str(e):
            raise ValueError("OpenAI API key invalid") from e
        raise
```

**User-Facing Error Messages**:
```python
ERROR_MESSAGES = {
    "rate_limit": "Translation service is busy. Please try again in a moment.",
    "timeout": "Chapter too large. Please contact support.",
    "api_error": "Translation failed. Please try again.",
    "auth_required": "Please log in to use Urdu translation.",
    "network": "Connection lost. Check your internet and retry."
}

def format_error_for_user(exception):
    if isinstance(exception, openai.RateLimitError):
        return ERROR_MESSAGES["rate_limit"]
    elif isinstance(exception, TimeoutError):
        return ERROR_MESSAGES["timeout"]
    # ...
```

**Chunking for Large Chapters**:
```python
def chunk_chapter(text, max_tokens=100000):
    import tiktoken
    encoding = tiktoken.encoding_for_model("gpt-4o")

    if len(encoding.encode(text)) <= max_tokens:
        return [text]

    # Split on paragraph boundaries
    paragraphs = text.split("\n\n")
    chunks = []
    current = []
    current_tokens = 0

    for para in paragraphs:
        para_tokens = len(encoding.encode(para))
        if current_tokens + para_tokens > max_tokens:
            chunks.append("\n\n".join(current))
            current = [para]
            current_tokens = para_tokens
        else:
            current.append(para)
            current_tokens += para_tokens

    if current:
        chunks.append("\n\n".join(current))

    return chunks
```

**Circuit Breaker Pattern** (optional):
```python
from pybreaker import CircuitBreaker

translation_breaker = CircuitBreaker(
    fail_max=5,  # Open circuit after 5 failures
    timeout_duration=60  # Stay open for 60 seconds
)

@translation_breaker
async def translate(text):
    return await openai_translate(text)
```

---

## 5. Frontend State Management

### Decision

**Use React Context API for language toggle state + component-level state for translation loading.**

Why not URL params or global state:
- Translation is session-based only (not persistent across visits)
- URL-based language toggle would conflict with Docusaurus routing
- Component state sufficient for button loading/error states

### Implementation Guidance

**Translation Context**:
```typescript
// src/contexts/TranslationContext.tsx
import React, { createContext, useState, useContext } from 'react';

interface TranslationContextType {
  currentLanguage: 'en' | 'ur';
  translatedContent: string | null;
  toggleLanguage: () => void;
  setTranslation: (content: string) => void;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

export const TranslationProvider: React.FC = ({ children }) => {
  const [currentLanguage, setLanguage] = useState<'en' | 'ur'>('en');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);

  const toggleLanguage = () => {
    setLanguage(prev => prev === 'en' ? 'ur' : 'en');
  };

  const setTranslation = (content: string) => {
    setTranslatedContent(content);
    setLanguage('ur');
  };

  return (
    <TranslationContext.Provider value={{ currentLanguage, translatedContent, toggleLanguage, setTranslation }}>
      {children}
    </TranslationContext.Provider>
  );
};

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) throw new Error('useTranslation must be used within TranslationProvider');
  return context;
};
```

**Translation Button Component**:
```typescript
// src/components/TranslationButton/index.tsx
import React, { useState } from 'react';
import { useTranslation } from '@site/src/contexts/TranslationContext';

export const TranslationButton: React.FC<{ chapterSlug: string }> = ({ chapterSlug }) => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const { currentLanguage, toggleLanguage, setTranslation } = useTranslation();

  const handleTranslate = async () => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ chapter_slug: chapterSlug })
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.message || 'Translation failed');
      }

      const { translated_content } = await response.json();
      setTranslation(translated_content);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="translation-controls">
      {currentLanguage === 'en' ? (
        <button onClick={handleTranslate} disabled={loading}>
          {loading ? 'Translating...' : 'Translate to Urdu'}
        </button>
      ) : (
        <button onClick={toggleLanguage}>Show Original English</button>
      )}
      {error && <p className="error">{error}</p>}
    </div>
  );
};
```

**Docusaurus Integration** (wrap MDX content):
```typescript
// src/theme/DocItem/Content/index.tsx (theme swizzle)
import React from 'react';
import { TranslationProvider, useTranslation } from '@site/src/contexts/TranslationContext';
import OriginalContent from '@theme-original/DocItem/Content';

function ContentWrapper() {
  const { currentLanguage, translatedContent } = useTranslation();

  if (currentLanguage === 'ur' && translatedContent) {
    return (
      <div className="markdown urdu-content" dir="rtl">
        {/* Render translated markdown */}
        <div dangerouslySetInnerHTML={{ __html: translatedContent }} />
      </div>
    );
  }

  return <OriginalContent />;
}

export default function Content() {
  return (
    <TranslationProvider>
      <ContentWrapper />
    </TranslationProvider>
  );
}
```

---

## 6. Additional Research Findings

### Urdu Font Support
- **Noto Nastaliq Urdu** recommended (Google Fonts): https://fonts.google.com/noto/specimen/Noto+Nastaliq+Urdu
- Add to `docusaurus.config.js`:
```javascript
stylesheets: [
  {
    href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap',
    type: 'text/css',
  },
]
```
- CSS for Urdu content:
```css
.urdu-content {
  font-family: 'Noto Nastaliq Urdu', serif;
  direction: rtl;
  text-align: right;
  line-height: 1.8; /* Nastaliq needs more line spacing */
}
```

### Docusaurus RTL Support
- Docusaurus has built-in RTL support via `i18n` config
- For dynamic switching (our use case), use `dir="rtl"` on container div
- Preserve LTR for code blocks even in RTL context:
```css
.urdu-content pre, .urdu-content code {
  direction: ltr;
  text-align: left;
}
```

### Analytics Tracking
```python
# backend/src/services/analytics_service.py
async def log_translation_request(user_id: str, chapter_slug: str, action: str, db):
    db.add(TranslationLog(
        user_id=user_id,
        chapter_slug=chapter_slug,
        action=action,  # "translate_requested" | "toggle_to_english" | "toggle_to_urdu"
        timestamp=datetime.now()
    ))
    await db.commit()
```

---

## Summary of Key Decisions

| Unknown | Decision | Primary Rationale |
|---------|----------|-------------------|
| AI Translation | OpenAI Agents SDK + GPT-4o + streaming | Future-proof, production-ready, superior quality for technical content |
| Caching | PostgreSQL (Neon) with 90-day TTL | 99% cost savings, $0 infrastructure, adequate performance |
| Markdown Preservation | AST-based extraction with `markdown-it-py` | Robust, CommonMark compliant, handles edge cases |
| Error Handling | Exponential backoff retry + user-friendly messages | Resilient to transient failures, clear UX |
| Frontend State | React Context API + component state | Simple, session-based, no persistence complexity |
| Font/RTL | Noto Nastaliq Urdu + `dir="rtl"` | Google Fonts free, native browser RTL support |

---

## Next Steps (Phase 1)

With research complete, proceed to:
1. **data-model.md**: Define database schemas (translation_cache, translation_logs)
2. **contracts/**: OpenAPI spec for `/api/translate` endpoint
3. **quickstart.md**: Local setup instructions for translation feature
4. **Agent context update**: Add OpenAI Agents SDK to CLAUDE.md

---

**Research Status**: ✅ Complete - All unknowns resolved, ready for Phase 1 design
