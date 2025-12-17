#!/usr/bin/env python3
"""
Test script to verify RAG Chatbot setup.
Checks connections to all services and validates configuration.
"""
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.config import settings
import asyncio


async def test_config():
    """Test configuration loading."""
    print("=" * 60)
    print("1. Testing Configuration")
    print("=" * 60)

    try:
        print(f"✓ Environment: {settings.environment}")
        print(f"✓ Qdrant URL: {settings.qdrant_url}")
        print(f"✓ Qdrant Collection: {settings.qdrant_collection_name}")
        print(f"✓ Database URL: {'***' if settings.database_url else 'NOT SET'}")
        print(f"✓ OpenAI API Key: {'***' + settings.openai_api_key[-4:] if settings.openai_api_key else 'NOT SET'}")
        print(f"✓ Embedding Model: {settings.embedding_model}")
        print(f"✓ LLM Model: {settings.llm_model}")
        print(f"✓ CORS Origins: {settings.get_cors_origins()}")
        return True
    except Exception as e:
        print(f"✗ Configuration error: {e}")
        return False


async def test_postgres():
    """Test Postgres connection."""
    print("\n" + "=" * 60)
    print("2. Testing Postgres Connection")
    print("=" * 60)

    try:
        import psycopg2
        conn = psycopg2.connect(settings.database_url)
        cur = conn.cursor()

        # Check if table exists
        cur.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'textbook_chunks'
            );
        """)
        table_exists = cur.fetchone()[0]

        if table_exists:
            cur.execute("SELECT COUNT(*) FROM textbook_chunks;")
            count = cur.fetchone()[0]
            print(f"✓ Connected to Postgres")
            print(f"✓ Table 'textbook_chunks' exists")
            print(f"✓ Chunks indexed: {count}")
        else:
            print("✓ Connected to Postgres")
            print("⚠ Table 'textbook_chunks' does not exist - run init_database.py")

        cur.close()
        conn.close()
        return True

    except Exception as e:
        print(f"✗ Postgres connection failed: {e}")
        return False


async def test_qdrant():
    """Test Qdrant connection."""
    print("\n" + "=" * 60)
    print("3. Testing Qdrant Connection")
    print("=" * 60)

    try:
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Check collection
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        print(f"✓ Connected to Qdrant")
        print(f"✓ Available collections: {collection_names}")

        if settings.qdrant_collection_name in collection_names:
            collection_info = client.get_collection(settings.qdrant_collection_name)
            print(f"✓ Collection '{settings.qdrant_collection_name}' exists")
            print(f"✓ Vectors count: {collection_info.vectors_count}")
        else:
            print(f"⚠ Collection '{settings.qdrant_collection_name}' does not exist - run ingest_textbook.py")

        return True

    except Exception as e:
        print(f"✗ Qdrant connection failed: {e}")
        return False


async def test_openai():
    """Test OpenAI API."""
    print("\n" + "=" * 60)
    print("4. Testing OpenAI API")
    print("=" * 60)

    try:
        from openai import OpenAI

        client = OpenAI(api_key=settings.openai_api_key)

        # Test embedding generation
        response = client.embeddings.create(
            model=settings.embedding_model,
            input="test"
        )

        print(f"✓ OpenAI API accessible")
        print(f"✓ Embedding model: {settings.embedding_model}")
        print(f"✓ Embedding dimension: {len(response.data[0].embedding)}")

        # Test LLM
        chat_response = client.chat.completions.create(
            model=settings.llm_model,
            messages=[{"role": "user", "content": "Say 'test'"}],
            max_tokens=10
        )

        print(f"✓ LLM model: {settings.llm_model}")
        print(f"✓ LLM test response: {chat_response.choices[0].message.content}")

        return True

    except Exception as e:
        print(f"✗ OpenAI API test failed: {e}")
        return False


async def test_full_rag_pipeline():
    """Test full RAG pipeline with sample query."""
    print("\n" + "=" * 60)
    print("5. Testing Full RAG Pipeline")
    print("=" * 60)

    try:
        from src.services.rag_pipeline import rag_pipeline

        # Sample query
        query = "What is robotics?"
        print(f"Query: {query}")

        response, sources, processing_time = await rag_pipeline.process_query(query)

        print(f"✓ RAG pipeline executed successfully")
        print(f"✓ Response length: {len(response)} chars")
        print(f"✓ Sources found: {len(sources)}")
        print(f"✓ Processing time: {processing_time}ms")

        print(f"\nResponse preview:")
        print(f"{response[:200]}...")

        if sources:
            print(f"\nSource preview:")
            print(f"  - {sources[0]['module_name']} → {sources[0]['chapter_name']}")

        return True

    except Exception as e:
        print(f"✗ RAG pipeline test failed: {e}")
        print(f"  This is expected if textbook content is not yet indexed")
        return False


async def main():
    """Run all tests."""
    print("\n🧪 RAG Chatbot Setup Test\n")

    results = []

    results.append(await test_config())
    results.append(await test_postgres())
    results.append(await test_qdrant())
    results.append(await test_openai())

    # Only test RAG if all services are working
    if all(results):
        results.append(await test_full_rag_pipeline())

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    print(f"Tests passed: {passed}/{total}")

    if passed == total:
        print("✅ All tests passed! System is ready.")
        return 0
    else:
        print("⚠️ Some tests failed. Check configuration and connections.")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
