"""
Database migration runner for translation feature.

Feature: 004-urdu-translation
Usage: python migrations/run_migrations.py
"""

import os
import psycopg2
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable not set")


def run_migration(cursor, migration_file):
    """Execute a single migration SQL file."""
    with open(migration_file, 'r') as f:
        sql = f.read()

    print(f"Running migration: {os.path.basename(migration_file)}")
    cursor.execute(sql)
    print(f"✓ Completed: {os.path.basename(migration_file)}")


def main():
    """Run all migration scripts in order."""
    migrations_dir = os.path.dirname(os.path.abspath(__file__))
    migration_files = [
        os.path.join(migrations_dir, "001_create_translation_cache.sql"),
        os.path.join(migrations_dir, "002_create_translation_log.sql"),
    ]

    try:
        # Connect to database
        print(f"Connecting to database...")
        conn = psycopg2.connect(DATABASE_URL)
        cursor = conn.cursor()

        # Run migrations
        print(f"\nRunning {len(migration_files)} migrations...\n")
        for migration_file in migration_files:
            if os.path.exists(migration_file):
                run_migration(cursor, migration_file)
            else:
                print(f"⚠ Warning: Migration file not found: {migration_file}")

        # Commit changes
        conn.commit()
        print(f"\n✓ All migrations completed successfully")

        # Verify tables were created
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_name IN ('translation_cache', 'translation_log')
            ORDER BY table_name;
        """)
        tables = cursor.fetchall()
        print(f"\nCreated tables:")
        for table in tables:
            print(f"  - {table[0]}")

        cursor.close()
        conn.close()

    except Exception as e:
        print(f"\n✗ Migration failed: {e}")
        raise


if __name__ == "__main__":
    main()
