"""
Indexer for storing textbook chunks in Qdrant and Postgres.
Orchestrates parsing, chunking, embedding, and storage.
"""
from pathlib import Path
from typing import List
import uuid

from src.ingestion.textbook_parser import TextbookParser, TextbookPage
from src.ingestion.chunker import TextbookChunker
from src.services.embeddings import embedding_service
from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service


class TextbookIndexer:
    """Orchestrates textbook indexing pipeline."""

    def __init__(self, textbook_root: Path):
        """
        Initialize indexer.

        Args:
            textbook_root: Path to textbook/docs/ directory
        """
        self.parser = TextbookParser(textbook_root)
        self.chunker = TextbookChunker(
            target_chunk_size=400,
            overlap_tokens=50,
            min_chunk_size=100,
            max_chunk_size=600
        )

    def index_page(self, page: TextbookPage) -> int:
        """
        Index a single textbook page.

        Args:
            page: TextbookPage object

        Returns:
            Number of chunks indexed
        """
        # Chunk the page content
        chunks = self.chunker.chunk_page(page.content)

        if not chunks:
            return 0

        # Generate embeddings for all chunks
        chunk_texts = [chunk.text for chunk in chunks]
        embeddings = embedding_service.generate_embeddings_batch(chunk_texts)

        # Prepare data for bulk insertion
        qdrant_vectors = []
        postgres_chunks = []

        for chunk, embedding in zip(chunks, embeddings):
            vector_id = uuid.uuid4()

            # Qdrant payload
            payload = {
                "module_name": page.module_name,
                "chapter_name": page.chapter_name,
                "slug": page.slug,
                "section_heading": chunk.section_heading,
            }

            qdrant_vectors.append((vector_id, embedding, payload))

            # Postgres metadata
            postgres_chunks.append({
                "qdrant_vector_id": vector_id,
                "chunk_text": chunk.text,
                "module_name": page.module_name,
                "chapter_name": page.chapter_name,
                "section_heading": chunk.section_heading,
                "file_path": page.file_path,
                "slug": page.slug,
                "chunk_index": chunk.chunk_index,
                "token_count": chunk.token_count,
            })

        # Bulk insert into Qdrant
        qdrant_service.bulk_insert_vectors(qdrant_vectors)

        # Bulk insert into Postgres
        postgres_service.bulk_insert_chunks(postgres_chunks)

        return len(chunks)

    def index_all(self, verbose: bool = True) -> dict:
        """
        Index all textbook pages.

        Args:
            verbose: Print progress messages

        Returns:
            Statistics dictionary
        """
        # Parse all pages
        if verbose:
            print("Parsing textbook files...")
        pages = self.parser.parse_all()

        if verbose:
            print(f"Found {len(pages)} pages")

        # Index each page
        total_chunks = 0
        for i, page in enumerate(pages):
            if verbose:
                print(f"[{i+1}/{len(pages)}] Indexing: {page.module_name} / {page.chapter_name}")

            chunk_count = self.index_page(page)
            total_chunks += chunk_count

            if verbose:
                print(f"  → {chunk_count} chunks indexed")

        return {
            "pages_processed": len(pages),
            "total_chunks": total_chunks,
            "status": "completed"
        }
