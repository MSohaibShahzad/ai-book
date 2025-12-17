"""
PostgreSQL service for textbook chunk metadata.
Uses SQLAlchemy for database operations with Neon Postgres.
"""
from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, UniqueConstraint, Index
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
from typing import List, Optional, Dict
import asyncio
import uuid

from src.config import settings


Base = declarative_base()


class TextbookChunk(Base):
    """
    SQLAlchemy model for textbook_chunks table.
    Stores metadata for each textbook chunk with references to Qdrant vectors.
    """
    __tablename__ = "textbook_chunks"

    id = Column(Integer, primary_key=True, autoincrement=True)
    qdrant_vector_id = Column(UUID(as_uuid=True), unique=True, nullable=False)
    chunk_text = Column(Text, nullable=False)
    module_name = Column(String(255), nullable=False)
    chapter_name = Column(String(255), nullable=False)
    section_heading = Column(String(500), nullable=True)
    file_path = Column(String(500), nullable=False)
    slug = Column(String(500), nullable=False)
    chunk_index = Column(Integer, nullable=False)
    token_count = Column(Integer, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)

    __table_args__ = (
        UniqueConstraint('file_path', 'chunk_index', name='unique_chunk'),
        Index('idx_qdrant_vector_id', 'qdrant_vector_id'),
        Index('idx_module_chapter', 'module_name', 'chapter_name'),
        Index('idx_slug', 'slug'),
    )


class PostgresService:
    """Service for PostgreSQL database operations."""

    def __init__(self):
        """Initialize database connection and create tables if needed."""
        self.engine = create_engine(
            settings.database_url,
            pool_size=5,  # Number of connections to maintain
            max_overflow=10,  # Max connections beyond pool_size
            pool_timeout=30,  # Seconds to wait for connection
            pool_recycle=3600,  # Recycle connections after 1 hour
            pool_pre_ping=True  # Verify connections before using
        )
        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)

    def create_tables(self):
        """Create all tables if they don't exist."""
        Base.metadata.create_all(bind=self.engine)

    def health_check(self) -> bool:
        """Check if database connection is healthy."""
        try:
            from sqlalchemy import text
            with self.engine.connect() as conn:
                conn.execute(text("SELECT 1"))
            return True
        except Exception:
            return False

    def get_chunk_by_vector_id(self, vector_id: uuid.UUID) -> Optional[TextbookChunk]:
        """Retrieve chunk metadata by Qdrant vector ID."""
        session = self.SessionLocal()
        try:
            return session.query(TextbookChunk).filter(
                TextbookChunk.qdrant_vector_id == vector_id
            ).first()
        finally:
            session.close()

    async def get_chunks_by_vector_ids(self, vector_ids: List[uuid.UUID]) -> List[Dict]:
        """
        Async wrapper for retrieving multiple chunks by Qdrant vector IDs.
        Returns list of dictionaries instead of SQLAlchemy objects.
        """
        loop = asyncio.get_event_loop()
        chunks = await loop.run_in_executor(None, self._get_chunks_by_vector_ids_sync, vector_ids)
        
        # Convert SQLAlchemy objects to dictionaries
        return [
            {
                "qdrant_vector_id": str(chunk.qdrant_vector_id),
                "chunk_text": chunk.chunk_text,
                "module_name": chunk.module_name,
                "chapter_name": chunk.chapter_name,
                "section_heading": chunk.section_heading,
                "file_path": chunk.file_path,
                "slug": chunk.slug,
                "chunk_index": chunk.chunk_index,
            }
            for chunk in chunks
        ]
    
    def _get_chunks_by_vector_ids_sync(self, vector_ids: List[uuid.UUID]) -> List[TextbookChunk]:
        """Synchronous version for executor."""
        session = self.SessionLocal()
        try:
            return session.query(TextbookChunk).filter(
                TextbookChunk.qdrant_vector_id.in_(vector_ids)
            ).all()
        finally:
            session.close()

    def insert_chunk(
        self,
        qdrant_vector_id: uuid.UUID,
        chunk_text: str,
        module_name: str,
        chapter_name: str,
        section_heading: Optional[str],
        file_path: str,
        slug: str,
        chunk_index: int,
        token_count: int
    ) -> TextbookChunk:
        """Insert a new textbook chunk."""
        session = self.SessionLocal()
        try:
            chunk = TextbookChunk(
                qdrant_vector_id=qdrant_vector_id,
                chunk_text=chunk_text,
                module_name=module_name,
                chapter_name=chapter_name,
                section_heading=section_heading,
                file_path=file_path,
                slug=slug,
                chunk_index=chunk_index,
                token_count=token_count
            )
            session.add(chunk)
            session.commit()
            session.refresh(chunk)
            return chunk
        finally:
            session.close()

    def bulk_insert_chunks(self, chunks: List[dict]) -> int:
        """Bulk insert multiple chunks. Returns number of inserted chunks."""
        session = self.SessionLocal()
        try:
            chunk_objects = [TextbookChunk(**chunk_data) for chunk_data in chunks]
            session.bulk_save_objects(chunk_objects)
            session.commit()
            return len(chunk_objects)
        finally:
            session.close()


# Global instance
postgres_service = PostgresService()
