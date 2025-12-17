# Textbook Ingestion Guide

This guide explains how to index the Physical-AI textbook into the RAG system.

## Prerequisites

Before running ingestion, you must have:

1. ✅ **Qdrant Cloud account** - Sign up at https://cloud.qdrant.io (free tier available)
2. ✅ **Neon Postgres database** - Sign up at https://neon.tech (free tier available)
3. ✅ **OpenAI API key** - Get from https://platform.openai.com/api-keys

## Step 1: Configure Environment Variables

Edit `backend/.env` with your actual credentials:

```bash
# Qdrant Cloud
QDRANT_URL=https://abc123-example.us-east.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your_actual_qdrant_api_key_here
QDRANT_COLLECTION_NAME=textbook_chunks

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-abc123.us-east-2.aws.neon.tech/neondb?sslmode=require

# OpenAI
OPENAI_API_KEY=sk-proj-abc123...
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o-mini
```

### Getting Qdrant Credentials

1. Go to https://cloud.qdrant.io
2. Create a free cluster (choose any region)
3. Copy the cluster URL and API key
4. Paste into `QDRANT_URL` and `QDRANT_API_KEY`

### Getting Neon Postgres Credentials

1. Go to https://neon.tech
2. Create a new project
3. Copy the connection string from the dashboard
4. Paste into `DATABASE_URL`

### Getting OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Create a new API key
3. Paste into `OPENAI_API_KEY`

## Step 2: Initialize Database Schema

Run the database initialization script:

```bash
cd backend
source venv/bin/activate  # Activate virtual environment
python scripts/init_database.py
```

Expected output:
```
Creating textbook_chunks table...
Creating indexes...
✅ Database schema initialized successfully
```

## Step 3: Run Pre-flight Check

Before ingestion, verify all services are accessible:

```bash
python scripts/preflight_check.py
```

Expected output:
```
✅ All checks passed! Ready for ingestion.
```

If any checks fail, fix the issues before proceeding.

## Step 4: Preview Ingestion (Dry Run)

Preview what will be ingested without actually storing data:

```bash
python scripts/ingest_textbook.py --textbook-path ../textbook/docs --dry-run
```

This will:
- Parse all Markdown files
- Extract frontmatter (module, chapter, slug)
- Chunk content (300-500 tokens per chunk)
- Show statistics without storing

Expected output:
```
🔍 DRY RUN MODE - No data will be stored

📄 Parsing files...
  Found 25 markdown files

✂️  Chunking content...
  Generated 487 chunks
  Average chunk size: 412 tokens

Dry run complete - no data stored
```

## Step 5: Run Full Ingestion

If dry run looks good, run the full ingestion:

```bash
python scripts/ingest_textbook.py --textbook-path ../textbook/docs
```

This will:
1. Parse all textbook Markdown files
2. Chunk content intelligently (heading-based, 300-500 tokens)
3. Generate embeddings using OpenAI (text-embedding-3-small)
4. Store vectors in Qdrant Cloud
5. Store metadata in Neon Postgres

**Duration**: ~5-10 minutes for full textbook (depends on size)

Expected output:
```
📊 Checking database connections...
  ✅ Qdrant Cloud connected
  ✅ Postgres connected

🗂️  Setting up Qdrant collection...
  ✅ Collection ready

🗂️  Setting up Postgres tables...
  ✅ Tables ready

📄 Parsing textbook files...
  Processing: 00-preface/index.md
  Processing: 01-foundations-ros2/introduction.md
  ...
  Found 25 files

✂️  Chunking content...
  Generated 487 chunks
  Average tokens: 412

🔢 Generating embeddings...
  Progress: [████████████████] 100%
  Embedded 487 chunks

💾 Storing in Qdrant...
  Inserted 487 vectors

💾 Storing metadata in Postgres...
  Inserted 487 chunks

✅ Ingestion complete!
  Total files: 25
  Total chunks: 487
  Total tokens: ~200,000
```

## Step 6: Verify Ingestion

Check that data was stored correctly:

```bash
python scripts/test_setup.py
```

Expected output:
```
✓ Postgres connected (487 chunks indexed)
✓ Qdrant connected (487 vectors stored)
✓ RAG pipeline working
```

## Cost Estimate

### One-time Ingestion Costs (OpenAI)

For a textbook with ~100,000 words (~130,000 tokens):

- **Embeddings**: ~$0.02 (text-embedding-3-small: $0.00002 per 1K tokens)
- **Total**: < $0.05 for full textbook indexing

### Ongoing Query Costs

Per student query:
- **Embedding**: ~$0.00002 (query embedding)
- **LLM**: ~$0.002 (GPT-4o-mini response generation)
- **Total per query**: ~$0.002

Example: 1000 student queries = ~$2.00

### Free Tier Limits

- **Qdrant Cloud**: 1GB storage (holds ~1M vectors) - sufficient
- **Neon Postgres**: 0.5GB storage - sufficient
- **OpenAI**: Pay-as-you-go (no free tier, but very affordable)

## Troubleshooting

### Error: "Connection refused" (Qdrant)

- Verify QDRANT_URL is correct (includes https:// and port :6333)
- Check Qdrant API key is valid
- Ensure cluster is running (not paused)

### Error: "Connection timeout" (Postgres)

- Verify DATABASE_URL format: `postgresql://user:password@host/db?sslmode=require`
- Check Neon project is active
- Ensure sslmode=require is included

### Error: "Invalid API key" (OpenAI)

- Verify API key starts with `sk-proj-` or `sk-`
- Check key has not expired
- Ensure billing is set up on OpenAI account

### Error: "Embedding dimension mismatch"

- Delete existing Qdrant collection and re-create
- Or use a different collection name in .env

### Low chunk count

- Verify Markdown files have YAML frontmatter with `module`, `chapter`, `slug`
- Check file permissions (should be readable)
- Review dry-run output for parsing errors

## Re-indexing

To re-index the textbook (e.g., after content updates):

```bash
# Option 1: Drop and recreate collection
python -c "from src.services.qdrant_service import qdrant_service; qdrant_service.client.delete_collection('textbook_chunks')"

# Option 2: Use a different collection name
# Edit QDRANT_COLLECTION_NAME in .env to a new name (e.g., textbook_chunks_v2)

# Then re-run ingestion
python scripts/ingest_textbook.py --textbook-path ../textbook/docs
```

## Next Steps

After successful ingestion:

1. **Start the backend**: `uvicorn src.main:app --reload`
2. **Start the frontend**: `cd ../textbook && npm start`
3. **Test the chatbot**: Open http://localhost:3000 and click the chat button
4. **Try sample queries**:
   - "What is inverse kinematics?"
   - "Explain ROS 2 architecture"
   - "How do VLAs work?"

---

**Need help?** See `backend/README.md` or `QUICKSTART_RAG.md`
