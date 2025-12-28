"""
Database connection pool management for Postgres.
Provides thread-safe connection pooling using psycopg2.pool.
"""
import psycopg2
from psycopg2 import pool
from contextlib import contextmanager
import logging

from src.config import settings

logger = logging.getLogger(__name__)


class DatabasePool:
    """Singleton connection pool for Postgres."""

    _instance = None
    _pool = None

    def __new__(cls):
        """Ensure singleton instance."""
        if cls._instance is None:
            cls._instance = super(DatabasePool, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize connection pool if not already created."""
        if self._pool is None:
            self._initialize_pool()

    def _initialize_pool(self):
        """Create connection pool with min/max connections."""
        try:
            self._pool = psycopg2.pool.ThreadedConnectionPool(
                minconn=2,  # Minimum connections
                maxconn=10,  # Maximum connections
                dsn=settings.database_url,
                connect_timeout=10,
            )
            logger.info("Database connection pool initialized (min=2, max=10)")
        except psycopg2.Error as e:
            logger.error(f"Failed to create connection pool: {e}")
            raise

    @contextmanager
    def get_connection(self):
        """
        Get a connection from the pool.

        Usage:
            with db_pool.get_connection() as conn:
                cur = conn.cursor()
                cur.execute("SELECT ...")
                results = cur.fetchall()
                cur.close()

        Yields:
            psycopg2.connection: Database connection
        """
        conn = None
        try:
            conn = self._pool.getconn()
            if conn is None:
                raise Exception("Failed to get connection from pool")

            yield conn

            # Commit if no errors
            conn.commit()

        except Exception as e:
            # Rollback on error
            if conn:
                conn.rollback()
            logger.error(f"Database operation failed: {e}")
            raise

        finally:
            # Return connection to pool
            if conn:
                self._pool.putconn(conn)

    @contextmanager
    def get_cursor(self):
        """
        Get a cursor from a pooled connection.

        Convenience method that handles both connection and cursor.

        Usage:
            with db_pool.get_cursor() as cur:
                cur.execute("SELECT ...")
                results = cur.fetchall()

        Yields:
            psycopg2.cursor: Database cursor
        """
        with self.get_connection() as conn:
            cur = conn.cursor()
            try:
                yield cur
            finally:
                cur.close()

    def close_all(self):
        """Close all connections in pool. Call on shutdown."""
        if self._pool:
            self._pool.closeall()
            logger.info("Database connection pool closed")


# Global database pool instance
db_pool = DatabasePool()


# Cleanup on module unload (optional, for graceful shutdown)
import atexit
atexit.register(db_pool.close_all)
