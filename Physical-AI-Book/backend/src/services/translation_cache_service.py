"""
Translation cache service for PostgreSQL storage.

Feature: 004-urdu-translation
Implements: Cache lookup, storage, and TTL management
"""

import hashlib
import psycopg2
from datetime import datetime, timedelta
from typing import Optional, Dict
import os
from dotenv import load_dotenv

load_dotenv()


class TranslationCacheService:
    """
    Service for managing translation cache in PostgreSQL.

    Implements:
    - Version-based cache invalidation via content_hash
    - TTL-based expiration (90 days default)
    - Cache hit/miss tracking
    """

    def __init__(self):
        """Initialize cache service with database connection."""
        self.database_url = os.getenv("DATABASE_URL")
        if not self.database_url:
            raise ValueError("DATABASE_URL environment variable not set")

        self.default_ttl_days = 90  # Default TTL for cached translations

    def _get_connection(self):
        """Get database connection."""
        return psycopg2.connect(self.database_url)

    def compute_content_hash(self, content: str) -> str:
        """
        Compute SHA-256 hash of content for versioning.

        Args:
            content: Source markdown content

        Returns:
            64-character hex hash
        """
        return hashlib.sha256(content.encode('utf-8')).hexdigest()

    async def get_cached_translation(
        self,
        chapter_slug: str,
        language: str,
        content_hash: str
    ) -> Optional[Dict]:
        """
        Retrieve cached translation if available and not expired.

        Args:
            chapter_slug: Chapter identifier
            language: Target language code ("ur")
            content_hash: SHA-256 hash of source content

        Returns:
            Dictionary with translated_content and cached_at, or None if cache miss
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            # Query for matching cache entry that hasn't expired
            cursor.execute("""
                SELECT translated_content, created_at, accessed_at
                FROM translation_cache
                WHERE chapter_slug = %s
                  AND language = %s
                  AND content_hash = %s
                  AND expires_at > CURRENT_TIMESTAMP
                LIMIT 1
            """, (chapter_slug, language, content_hash))

            result = cursor.fetchone()

            if result:
                # Cache hit - update accessed_at timestamp
                cursor.execute("""
                    UPDATE translation_cache
                    SET accessed_at = CURRENT_TIMESTAMP
                    WHERE chapter_slug = %s
                      AND language = %s
                      AND content_hash = %s
                """, (chapter_slug, language, content_hash))
                conn.commit()

                translated_content, created_at, accessed_at = result
                cursor.close()
                conn.close()

                print(f"[CACHE HIT] {chapter_slug} ({language})")
                return {
                    "translated_content": translated_content,
                    "cached_at": created_at,
                    "from_cache": True
                }

            # Cache miss
            cursor.close()
            conn.close()
            print(f"[CACHE MISS] {chapter_slug} ({language})")
            return None

        except Exception as e:
            print(f"[ERROR] Cache lookup failed: {e}")
            return None

    async def store_translation(
        self,
        chapter_slug: str,
        language: str,
        content_hash: str,
        translated_content: str,
        ttl_days: Optional[int] = None
    ) -> bool:
        """
        Store translated content in cache.

        Args:
            chapter_slug: Chapter identifier
            language: Target language code
            content_hash: SHA-256 hash of source content
            translated_content: Translated markdown content
            ttl_days: Time-to-live in days (defaults to 90)

        Returns:
            True if stored successfully, False otherwise
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            ttl = ttl_days or self.default_ttl_days
            expires_at = datetime.now() + timedelta(days=ttl)

            # Insert or update cache entry
            cursor.execute("""
                INSERT INTO translation_cache (
                    chapter_slug,
                    language,
                    content_hash,
                    translated_content,
                    created_at,
                    accessed_at,
                    expires_at
                )
                VALUES (%s, %s, %s, %s, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP, %s)
                ON CONFLICT (chapter_slug, language, content_hash)
                DO UPDATE SET
                    translated_content = EXCLUDED.translated_content,
                    accessed_at = CURRENT_TIMESTAMP,
                    expires_at = EXCLUDED.expires_at
            """, (chapter_slug, language, content_hash, translated_content, expires_at))

            conn.commit()
            cursor.close()
            conn.close()

            print(f"[CACHE STORED] {chapter_slug} ({language}) - TTL: {ttl} days")
            return True

        except Exception as e:
            print(f"[ERROR] Cache storage failed: {e}")
            return False

    async def invalidate_cache(
        self,
        chapter_slug: str,
        language: Optional[str] = None
    ) -> int:
        """
        Manually invalidate cached translations for a chapter.

        Args:
            chapter_slug: Chapter identifier
            language: Optional language filter (invalidates all languages if None)

        Returns:
            Number of cache entries deleted
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            if language:
                cursor.execute("""
                    DELETE FROM translation_cache
                    WHERE chapter_slug = %s AND language = %s
                """, (chapter_slug, language))
            else:
                cursor.execute("""
                    DELETE FROM translation_cache
                    WHERE chapter_slug = %s
                """, (chapter_slug,))

            deleted_count = cursor.rowcount
            conn.commit()
            cursor.close()
            conn.close()

            print(f"[CACHE INVALIDATED] {chapter_slug} - {deleted_count} entries deleted")
            return deleted_count

        except Exception as e:
            print(f"[ERROR] Cache invalidation failed: {e}")
            return 0

    async def cleanup_expired_entries(self) -> int:
        """
        Remove expired cache entries (TTL exceeded).

        Returns:
            Number of expired entries deleted
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            cursor.execute("""
                DELETE FROM translation_cache
                WHERE expires_at < CURRENT_TIMESTAMP
            """)

            deleted_count = cursor.rowcount
            conn.commit()
            cursor.close()
            conn.close()

            print(f"[CLEANUP] Removed {deleted_count} expired cache entries")
            return deleted_count

        except Exception as e:
            print(f"[ERROR] Cache cleanup failed: {e}")
            return 0

    async def get_cache_stats(self) -> Dict:
        """
        Get cache statistics for monitoring.

        Returns:
            Dictionary with cache metrics
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            # Total cached translations
            cursor.execute("SELECT COUNT(*) FROM translation_cache")
            total_cached = cursor.fetchone()[0]

            # Active (non-expired) translations
            cursor.execute("""
                SELECT COUNT(*) FROM translation_cache
                WHERE expires_at > CURRENT_TIMESTAMP
            """)
            active_cached = cursor.fetchone()[0]

            # Expired translations
            cursor.execute("""
                SELECT COUNT(*) FROM translation_cache
                WHERE expires_at <= CURRENT_TIMESTAMP
            """)
            expired_cached = cursor.fetchone()[0]

            # Most accessed chapters
            cursor.execute("""
                SELECT chapter_slug, language, COUNT(*) as access_count
                FROM translation_cache
                WHERE expires_at > CURRENT_TIMESTAMP
                GROUP BY chapter_slug, language
                ORDER BY access_count DESC
                LIMIT 10
            """)
            popular_chapters = cursor.fetchall()

            cursor.close()
            conn.close()

            return {
                "total_cached": total_cached,
                "active_cached": active_cached,
                "expired_cached": expired_cached,
                "popular_chapters": [
                    {"chapter": ch[0], "language": ch[1], "accesses": ch[2]}
                    for ch in popular_chapters
                ]
            }

        except Exception as e:
            print(f"[ERROR] Cache stats failed: {e}")
            return {
                "total_cached": 0,
                "active_cached": 0,
                "expired_cached": 0,
                "popular_chapters": []
            }
