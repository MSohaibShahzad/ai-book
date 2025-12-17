#!/usr/bin/env python3
"""
Database initialization script for RAG Chatbot.
Creates textbook_chunks table with required indexes in Postgres.
"""
import sys
import os
import psycopg2
from psycopg2 import sql

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.config import settings


def create_schema():
    """Create textbook_chunks table and indexes."""

    create_table_sql = """
    CREATE TABLE IF NOT EXISTS textbook_chunks (
        id SERIAL PRIMARY KEY,
        qdrant_vector_id UUID NOT NULL UNIQUE,
        chunk_text TEXT NOT NULL,
        module_name VARCHAR(255) NOT NULL,
        chapter_name VARCHAR(255) NOT NULL,
        section_heading VARCHAR(500),
        file_path TEXT NOT NULL,
        slug VARCHAR(500) NOT NULL,
        chunk_index INTEGER NOT NULL,
        token_count INTEGER NOT NULL,
        created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
    );
    """

    create_indexes_sql = [
        "CREATE INDEX IF NOT EXISTS idx_qdrant_vector_id ON textbook_chunks(qdrant_vector_id);",
        "CREATE INDEX IF NOT EXISTS idx_module_name ON textbook_chunks(module_name);",
        "CREATE INDEX IF NOT EXISTS idx_chapter_name ON textbook_chunks(chapter_name);",
        "CREATE INDEX IF NOT EXISTS idx_slug ON textbook_chunks(slug);",
    ]

    try:
        # Connect to database
        conn = psycopg2.connect(settings.database_url)
        cur = conn.cursor()

        print("Creating textbook_chunks table...")
        cur.execute(create_table_sql)

        print("Creating indexes...")
        for idx_sql in create_indexes_sql:
            cur.execute(idx_sql)

        conn.commit()
        print("✅ Database schema initialized successfully")

        # Verify table creation
        cur.execute("""
            SELECT column_name, data_type
            FROM information_schema.columns
            WHERE table_name = 'textbook_chunks'
            ORDER BY ordinal_position;
        """)
        columns = cur.fetchall()
        print(f"\nTable 'textbook_chunks' has {len(columns)} columns:")
        for col_name, col_type in columns:
            print(f"  - {col_name}: {col_type}")

        cur.close()
        conn.close()

    except psycopg2.Error as e:
        print(f"❌ Database error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    print("Initializing database schema for RAG Chatbot...")
    print(f"Database URL: {settings.database_url.split('@')[1] if '@' in settings.database_url else 'masked'}")
    create_schema()
