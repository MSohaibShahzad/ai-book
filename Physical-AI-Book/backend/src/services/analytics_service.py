"""
Analytics service for translation feature.

Feature: 004-urdu-translation
Phase 7: Analytics tracking
"""

import psycopg2
from datetime import datetime
from typing import Dict, List, Optional
import os
from dotenv import load_dotenv

load_dotenv()


class AnalyticsService:
    """
    Service for translation analytics and usage tracking.

    Provides insights into:
    - Most translated chapters
    - User engagement with translations
    - Language toggle patterns
    - Translation success/failure rates
    """

    def __init__(self):
        """Initialize analytics service with database connection."""
        self.database_url = os.getenv("DATABASE_URL")
        if not self.database_url:
            raise ValueError("DATABASE_URL environment variable not set")

    def _get_connection(self):
        """Get database connection."""
        return psycopg2.connect(self.database_url)

    async def log_translation_action(
        self,
        user_id: int,
        chapter_slug: str,
        action: str
    ) -> bool:
        """
        Log translation action to database.

        Args:
            user_id: User ID from auth system
            chapter_slug: Chapter identifier
            action: Action type (translate_requested, toggle_to_english, etc.)

        Returns:
            True if logged successfully, False otherwise
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            cursor.execute("""
                INSERT INTO translation_log (user_id, chapter_slug, action, timestamp)
                VALUES (%s, %s, %s, CURRENT_TIMESTAMP)
            """, (user_id, chapter_slug, action))

            conn.commit()
            cursor.close()
            conn.close()

            return True

        except Exception as e:
            print(f"[ERROR] Failed to log action: {e}")
            return False

    async def get_popular_chapters(self, limit: int = 10) -> List[Dict]:
        """
        Get most translated chapters.

        Args:
            limit: Number of results to return

        Returns:
            List of chapters with translation counts
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            cursor.execute("""
                SELECT
                    chapter_slug,
                    COUNT(*) as translation_count,
                    COUNT(DISTINCT user_id) as unique_users
                FROM translation_log
                WHERE action = 'translate_requested'
                GROUP BY chapter_slug
                ORDER BY translation_count DESC
                LIMIT %s
            """, (limit,))

            results = cursor.fetchall()
            cursor.close()
            conn.close()

            return [
                {
                    "chapter_slug": row[0],
                    "translation_count": row[1],
                    "unique_users": row[2]
                }
                for row in results
            ]

        except Exception as e:
            print(f"[ERROR] Failed to get popular chapters: {e}")
            return []

    async def get_user_translation_history(
        self,
        user_id: int,
        limit: int = 20
    ) -> List[Dict]:
        """
        Get translation history for a specific user.

        Args:
            user_id: User ID
            limit: Number of results to return

        Returns:
            List of translation actions
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            cursor.execute("""
                SELECT chapter_slug, action, timestamp
                FROM translation_log
                WHERE user_id = %s
                ORDER BY timestamp DESC
                LIMIT %s
            """, (user_id, limit))

            results = cursor.fetchall()
            cursor.close()
            conn.close()

            return [
                {
                    "chapter_slug": row[0],
                    "action": row[1],
                    "timestamp": row[2].isoformat()
                }
                for row in results
            ]

        except Exception as e:
            print(f"[ERROR] Failed to get user history: {e}")
            return []

    async def get_translation_stats(self) -> Dict:
        """
        Get overall translation statistics.

        Returns:
            Dictionary with various metrics
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()

            # Total translations requested
            cursor.execute("""
                SELECT COUNT(*) FROM translation_log
                WHERE action = 'translate_requested'
            """)
            total_translations = cursor.fetchone()[0]

            # Unique users who translated
            cursor.execute("""
                SELECT COUNT(DISTINCT user_id) FROM translation_log
                WHERE action = 'translate_requested'
            """)
            unique_users = cursor.fetchone()[0]

            # Total toggle actions
            cursor.execute("""
                SELECT COUNT(*) FROM translation_log
                WHERE action IN ('toggle_to_english', 'toggle_to_urdu')
            """)
            total_toggles = cursor.fetchone()[0]

            # Failed translations
            cursor.execute("""
                SELECT COUNT(*) FROM translation_log
                WHERE action = 'translation_failed'
            """)
            failed_translations = cursor.fetchone()[0]

            # Success rate
            success_rate = (
                ((total_translations - failed_translations) / total_translations * 100)
                if total_translations > 0 else 0
            )

            cursor.close()
            conn.close()

            return {
                "total_translations": total_translations,
                "unique_users": unique_users,
                "total_toggles": total_toggles,
                "failed_translations": failed_translations,
                "success_rate": round(success_rate, 2)
            }

        except Exception as e:
            print(f"[ERROR] Failed to get translation stats: {e}")
            return {
                "total_translations": 0,
                "unique_users": 0,
                "total_toggles": 0,
                "failed_translations": 0,
                "success_rate": 0
            }
