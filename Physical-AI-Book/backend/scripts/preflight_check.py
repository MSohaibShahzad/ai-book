#!/usr/bin/env python3
"""
Pre-flight check before running ingestion.
Verifies all services are properly configured and accessible.
"""
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.config import settings


def check_env_variables():
    """Check if environment variables are properly set."""
    print("\n" + "=" * 60)
    print("1. Checking Environment Variables")
    print("=" * 60)

    required_vars = {
        "QDRANT_URL": settings.qdrant_url,
        "QDRANT_API_KEY": settings.qdrant_api_key,
        "DATABASE_URL": settings.database_url,
        "OPENAI_API_KEY": settings.openai_api_key,
    }

    all_set = True

    for var_name, var_value in required_vars.items():
        if not var_value or "your-" in var_value or "password" in var_value:
            print(f"  ❌ {var_name}: NOT SET (using placeholder)")
            all_set = False
        else:
            masked = var_value[:10] + "..." if len(var_value) > 10 else "***"
            print(f"  ✅ {var_name}: {masked}")

    return all_set


def check_qdrant():
    """Check Qdrant connection."""
    print("\n" + "=" * 60)
    print("2. Checking Qdrant Cloud Connection")
    print("=" * 60)

    try:
        from src.services.qdrant_service import qdrant_service

        if qdrant_service.health_check():
            print("  ✅ Qdrant Cloud connected")

            # Check collection
            collections = qdrant_service.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if settings.qdrant_collection_name in collection_names:
                collection_info = qdrant_service.client.get_collection(settings.qdrant_collection_name)
                print(f"  ✅ Collection '{settings.qdrant_collection_name}' exists")
                print(f"     Vectors: {collection_info.vectors_count}")
            else:
                print(f"  ⚠️  Collection '{settings.qdrant_collection_name}' does not exist (will be created)")

            return True
        else:
            print("  ❌ Qdrant Cloud connection failed")
            return False

    except Exception as e:
        print(f"  ❌ Qdrant error: {e}")
        return False


def check_postgres():
    """Check Postgres connection."""
    print("\n" + "=" * 60)
    print("3. Checking Neon Postgres Connection")
    print("=" * 60)

    try:
        from src.services.postgres_service import postgres_service

        if postgres_service.health_check():
            print("  ✅ Postgres connected")

            # Check if table exists
            import psycopg2
            conn = psycopg2.connect(settings.database_url)
            cur = conn.cursor()

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
                print(f"  ✅ Table 'textbook_chunks' exists")
                print(f"     Chunks: {count}")
            else:
                print("  ⚠️  Table 'textbook_chunks' does not exist")
                print("     Run: python scripts/init_database.py")

            cur.close()
            conn.close()
            return True

        else:
            print("  ❌ Postgres connection failed")
            return False

    except Exception as e:
        print(f"  ❌ Postgres error: {e}")
        return False


def check_openai():
    """Check OpenAI API."""
    print("\n" + "=" * 60)
    print("4. Checking OpenAI API")
    print("=" * 60)

    try:
        from openai import OpenAI

        client = OpenAI(api_key=settings.openai_api_key)

        # Test embedding
        response = client.embeddings.create(
            model=settings.embedding_model,
            input="test"
        )

        print(f"  ✅ OpenAI API accessible")
        print(f"     Embedding model: {settings.embedding_model}")
        print(f"     LLM model: {settings.llm_model}")

        return True

    except Exception as e:
        print(f"  ❌ OpenAI API error: {e}")
        return False


def check_textbook_docs():
    """Check if textbook docs exist."""
    print("\n" + "=" * 60)
    print("5. Checking Textbook Documentation")
    print("=" * 60)

    from pathlib import Path

    textbook_path = Path("../textbook/docs")

    if not textbook_path.exists():
        print(f"  ❌ Textbook docs not found at: {textbook_path.absolute()}")
        return False

    # Count markdown files
    md_files = list(textbook_path.rglob("*.md"))

    print(f"  ✅ Textbook docs found")
    print(f"     Path: {textbook_path.absolute()}")
    print(f"     Markdown files: {len(md_files)}")

    return True


def main():
    """Run all checks."""
    print("\n🔍 RAG Chatbot - Pre-flight Check\n")

    checks = [
        check_env_variables(),
        check_qdrant(),
        check_postgres(),
        check_openai(),
        check_textbook_docs(),
    ]

    passed = sum(checks)
    total = len(checks)

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"Checks passed: {passed}/{total}\n")

    if passed == total:
        print("✅ All checks passed! Ready for ingestion.")
        print("\nTo run ingestion:")
        print("  python scripts/ingest_textbook.py --textbook-path ../textbook/docs")
        print("\nTo preview without storing:")
        print("  python scripts/ingest_textbook.py --textbook-path ../textbook/docs --dry-run")
        return 0
    else:
        print("⚠️  Some checks failed. Please fix the issues above before running ingestion.")
        print("\nSetup instructions: See backend/README.md or QUICKSTART_RAG.md")
        return 1


if __name__ == "__main__":
    sys.exit(main())
