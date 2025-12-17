#!/usr/bin/env python3
"""
CLI script for ingesting textbook content into RAG system.
Usage: python scripts/ingest_textbook.py --textbook-path ../textbook/docs [--dry-run]
"""
import argparse
import sys
from pathlib import Path

# Add parent directory to path to import src modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.ingestion.indexer import TextbookIndexer
from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service
from src.config import settings


def main():
    parser = argparse.ArgumentParser(description="Ingest textbook content into RAG system")
    parser.add_argument(
        "--textbook-path",
        type=str,
        required=True,
        help="Path to textbook docs directory"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and chunk without storing in databases"
    )
    args = parser.parse_args()

    textbook_path = Path(args.textbook_path)
    if not textbook_path.exists():
        print(f"❌ Error: Textbook path does not exist: {textbook_path}")
        sys.exit(1)

    print("=" * 60)
    print("RAG Chatbot - Textbook Ingestion")
    print("=" * 60)
    print(f"Textbook path: {textbook_path}")
    print(f"Environment: {settings.environment}")
    print(f"Qdrant collection: {settings.qdrant_collection_name}")
    print(f"Embedding model: {settings.embedding_model}")
    print("=" * 60)

    if args.dry_run:
        print("\n🔍 DRY RUN MODE - No data will be stored\n")

    # Initialize services
    if not args.dry_run:
        print("\n📊 Checking database connections...")

        # Check Qdrant
        if qdrant_service.health_check():
            print("  ✅ Qdrant Cloud connected")
        else:
            print("  ❌ Qdrant Cloud connection failed")
            sys.exit(1)

        # Check Postgres
        if postgres_service.health_check():
            print("  ✅ Postgres connected")
        else:
            print("  ❌ Postgres connection failed")
            sys.exit(1)

        # Create collection if needed
        print("\n🗂️  Setting up Qdrant collection...")
        qdrant_service.create_collection(vector_size=1536)
        print("  ✅ Collection ready")

        # Create Postgres tables if needed
        print("\n🗂️  Setting up Postgres tables...")
        postgres_service.create_tables()
        print("  ✅ Tables ready")

    # Initialize indexer
    indexer = TextbookIndexer(textbook_path)

    # Index textbook
    print("\n📚 Starting textbook ingestion...\n")

    if args.dry_run:
        # Just parse and chunk without storing
        pages = indexer.parser.parse_all()
        print(f"✅ Parsed {len(pages)} pages")

        total_chunks = 0
        for page in pages:
            chunks = indexer.chunker.chunk_page(page.content)
            total_chunks += len(chunks)
            print(f"  {page.module_name} / {page.chapter_name}: {len(chunks)} chunks")

        print(f"\n✅ DRY RUN COMPLETE: {total_chunks} total chunks would be indexed")
    else:
        # Full indexing
        stats = indexer.index_all(verbose=True)

        print("\n" + "=" * 60)
        print("✅ INGESTION COMPLETE")
        print("=" * 60)
        print(f"Pages processed: {stats['pages_processed']}")
        print(f"Total chunks indexed: {stats['total_chunks']}")
        print(f"Status: {stats['status']}")
        print("=" * 60)


if __name__ == "__main__":
    main()
