# Quickstart Guide: RAG Chatbot for Physical-AI Textbook

**Feature**: 002-rag-chatbot
**Purpose**: Step-by-step setup for local development and deployment

---

## Prerequisites

### Required Software
- **Python**: 3.11 or higher
- **Node.js**: 18.x or higher (for frontend widget)
- **npm**: 9.x or higher
- **Git**: For version control
- **Docker** (optional): For containerized deployment

### Required Accounts & API Keys
1. **Qdrant Cloud**: Free account at https://cloud.qdrant.io
   - Create cluster → Note API URL and API key
2. **Neon Postgres**: Free account at https://neon.tech
   - Create database → Note connection string
3. **OpenAI API**: Account at https://platform.openai.com
   - Generate API key → Note key

---

## Backend Setup

### 1. Clone Repository
```bash
cd /path/to/project
git checkout 002-rag-chatbot
```

### 2. Create Python Virtual Environment
```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Expected `requirements.txt`**:
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
qdrant-client==1.7.0
psycopg2-binary==2.9.9
sqlalchemy==2.0.23
openai==1.3.8
python-dotenv==1.0.0
tiktoken==0.5.2
pyyaml==6.0.1
pytest==7.4.3
httpx==0.25.1
pytest-asyncio==0.21.1
```

### 4. Configure Environment Variables
Create `backend/.env` file:
```env
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_chunks

# Neon Postgres
DATABASE_URL=postgresql://user:password@host/dbname?sslmode=require

# OpenAI
OPENAI_API_KEY=your-openai-api-key
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o-mini

# Application Settings
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=["http://localhost:3000","https://physical-ai-textbook.dev"]
RATE_LIMIT_PER_MINUTE=10
```

### 5. Initialize Database Schema
```bash
# Create tables
python scripts/init_database.py

# Verify connection
python scripts/check_database.py
```

**Expected `scripts/init_database.py`**:
```python
import psycopg2
from dotenv import load_dotenv
import os

load_dotenv()

conn = psycopg2.connect(os.getenv('DATABASE_URL'))
cur = conn.cursor()

# Create textbook_chunks table
cur.execute("""
CREATE TABLE IF NOT EXISTS textbook_chunks (
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
    CONSTRAINT unique_chunk UNIQUE (file_path, chunk_index)
);

CREATE INDEX IF NOT EXISTS idx_qdrant_vector_id ON textbook_chunks (qdrant_vector_id);
CREATE INDEX IF NOT EXISTS idx_module_chapter ON textbook_chunks (module_name, chapter_name);
CREATE INDEX IF NOT EXISTS idx_slug ON textbook_chunks (slug);
""")

conn.commit()
cur.close()
conn.close()
print("✅ Database schema initialized")
```

### 6. Ingest Textbook Content
```bash
# Dry run to check chunking
python scripts/ingest_textbook.py --textbook-path ../textbook/docs --dry-run

# Full ingestion (may take 1-2 minutes)
python scripts/ingest_textbook.py --textbook-path ../textbook/docs

# Expected output:
# Processing 10 modules...
# Chunking 50 chapters...
# Generated 847 chunks
# Embedding chunks... [====================] 100%
# Storing vectors in Qdrant... Done
# Storing metadata in Postgres... Done
# ✅ Ingestion complete: 847 chunks indexed
```

### 7. Start Backend Server
```bash
uvicorn src.main:app --reload --port 8000

# Expected output:
# INFO:     Uvicorn running on http://127.0.0.1:8000
# INFO:     Application startup complete
```

### 8. Test Backend API
```bash
# Health check
curl http://localhost:8000/v1/health

# Expected response:
# {"status":"healthy","services":{"qdrant":true,"postgres":true,"openai":true},"version":"1.0.0"}

# Test chat endpoint
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS 2?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "conversation_history": []
  }'
```

---

## Frontend Setup

### 1. Navigate to Frontend Directory
```bash
cd ../frontend
```

### 2. Install Dependencies
```bash
npm install
```

**Expected `package.json` dependencies**:
```json
{
  "dependencies": {
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "markdown-to-jsx": "^7.3.2"
  },
  "devDependencies": {
    "webpack": "^5.89.0",
    "webpack-cli": "^5.1.4",
    "babel-loader": "^9.1.3",
    "@babel/preset-react": "^7.23.3",
    "css-loader": "^6.8.1",
    "style-loader": "^3.3.3",
    "typescript": "^5.3.2",
    "@types/react": "^18.2.42",
    "ts-loader": "^9.5.1"
  }
}
```

### 3. Configure Environment Variables
Create `frontend/.env`:
```env
REACT_APP_API_URL=http://localhost:8000/v1
```

### 4. Build Widget
```bash
npm run build

# Expected output:
# Compiled successfully!
# File sizes after gzip:
#   65.2 KB  build/widget.js
```

### 5. Test Widget Locally
```bash
npm start

# Opens http://localhost:3000 with widget demo
```

---

## Docusaurus Integration

### 1. Copy Widget Bundle
```bash
# Copy built widget to Docusaurus static assets
cp frontend/build/widget.js ../textbook/static/chat-widget.js
```

### 2. Update Docusaurus Config
Edit `/textbook/docusaurus.config.js`:
```javascript
module.exports = {
  // ... existing config
  scripts: [
    {
      src: '/chat-widget.js',
      async: true,
      defer: true
    }
  ]
};
```

### 3. Test Integration
```bash
cd ../textbook
npm start

# Visit http://localhost:3000
# Widget should appear as floating button in bottom-right corner
```

---

## Deployment

### Backend Deployment (Render.com Example)

1. **Create `render.yaml`**:
```yaml
services:
  - type: web
    name: rag-chatbot-api
    runtime: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: DATABASE_URL
        sync: false
      - key: OPENAI_API_KEY
        sync: false
      - key: ENVIRONMENT
        value: production
```

2. **Deploy**:
```bash
# Connect to Render.com
render deploy

# Note the deployed URL: https://rag-chatbot-api.onrender.com
```

### Frontend Deployment (CDN)

1. **Build Production Bundle**:
```bash
cd frontend
NODE_ENV=production npm run build
```

2. **Upload to CDN**:
```bash
# Example: Upload to S3 + CloudFront
aws s3 cp build/widget.js s3://your-cdn-bucket/widget-v1.0.0.js
aws cloudfront create-invalidation --distribution-id YOUR_DIST_ID --paths "/widget-v1.0.0.js"
```

3. **Update Docusaurus Config**:
```javascript
scripts: [
  {
    src: 'https://cdn.example.com/widget-v1.0.0.js',
    async: true
  }
]
```

---

## Verification Checklist

### Backend
- [ ] `/health` returns `{"status": "healthy"}`
- [ ] Qdrant collection `textbook_chunks` exists with 500-1000 vectors
- [ ] Postgres table `textbook_chunks` has 500-1000 rows
- [ ] `/chat` endpoint returns answer with sources for "What is ROS 2?"
- [ ] Highlighted text query works: `highlighted_text` parameter is used in response

### Frontend
- [ ] Widget loads without console errors
- [ ] Floating button appears in bottom-right corner
- [ ] Click button opens chat interface
- [ ] Typing a message sends request to backend
- [ ] Assistant response displays with Markdown formatting
- [ ] Source references are clickable and navigate to textbook pages
- [ ] Page refresh clears conversation history
- [ ] Text selection populates `highlighted_text` in request

### Integration
- [ ] Widget works on all Docusaurus pages
- [ ] No CSS conflicts with textbook styles
- [ ] Widget bundle size < 100KB gzipped
- [ ] Response time < 3 seconds for 95% of queries

---

## Troubleshooting

### Backend Issues

**Error: "Qdrant connection failed"**
```bash
# Check Qdrant Cloud status
curl -X GET $QDRANT_URL/collections

# Verify API key
echo $QDRANT_API_KEY
```

**Error: "Database connection failed"**
```bash
# Test Postgres connection
psql $DATABASE_URL -c "SELECT 1;"

# Check SSL mode
# Neon requires sslmode=require
```

**Error: "OpenAI rate limit exceeded"**
```python
# Reduce concurrent requests in ingestion script
# Add exponential backoff in llm_service.py
```

### Frontend Issues

**Widget not loading**
```bash
# Check browser console for errors
# Verify script src URL is correct
# Check CORS headers in backend response
```

**Text selection not working**
```javascript
// Verify getSelection() API is supported
console.log(window.getSelection());
```

---

## Development Workflow

### 1. Make Changes
```bash
# Backend
cd backend
source venv/bin/activate
# Edit files in src/
uvicorn src.main:app --reload

# Frontend
cd frontend
# Edit files in src/
npm start
```

### 2. Run Tests
```bash
# Backend
pytest tests/ -v

# Frontend
npm test
```

### 3. Rebuild & Deploy
```bash
# Backend
git add .
git commit -m "feat: add highlight context priority"
git push origin 002-rag-chatbot

# Trigger deployment via CI/CD or manual deploy

# Frontend
npm run build
# Upload to CDN
```

---

## Next Steps

1. **Implement Backend**: Use `/sp.tasks` to generate task breakdown
2. **Build Frontend Widget**: Follow React best practices
3. **Write Tests**: Contract tests for API, integration tests for RAG pipeline
4. **Monitor Performance**: Add logging for retrieval latency and LLM response time
5. **Iterate on Chunking**: Test with real textbook content, adjust chunk size if needed

---

## Support

- **Documentation**: See `plan.md`, `data-model.md`, and `contracts/openapi.yaml`
- **Issues**: Track bugs and features in project management system
- **Constitution**: Refer to `.specify/memory/constitution.md` for RAG Chatbot Principles (VII-XIII)
