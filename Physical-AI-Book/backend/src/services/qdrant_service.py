"""
Qdrant service for vector similarity search.
Uses Qdrant Cloud for storing and retrieving textbook chunk embeddings.
"""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, ScoredPoint
from typing import List, Optional
import uuid

from src.config import settings


class QdrantService:
    """Service for Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client connection."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name

    def create_collection(self, vector_size: int = 1536):
        """
        Create Qdrant collection for textbook chunks.

        Args:
            vector_size: Embedding dimension (1536 for text-embedding-3-small)
        """
        try:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
        except Exception as e:
            # Collection might already exist
            if "already exists" not in str(e).lower():
                raise

    def health_check(self) -> bool:
        """Check if Qdrant connection is healthy."""
        try:
            self.client.get_collections()
            return True
        except Exception:
            return False

    def insert_vector(
        self,
        vector_id: uuid.UUID,
        embedding: List[float],
        payload: dict
    ) -> None:
        """
        Insert a single vector into Qdrant.

        Args:
            vector_id: UUID for the vector point
            embedding: 1536-dimensional embedding vector
            payload: Metadata (module_name, chapter_name, etc.)
        """
        point = PointStruct(
            id=str(vector_id),
            vector=embedding,
            payload=payload
        )
        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

    def bulk_insert_vectors(
        self,
        vectors: List[tuple[uuid.UUID, List[float], dict]]
    ) -> int:
        """
        Bulk insert multiple vectors.

        Args:
            vectors: List of (vector_id, embedding, payload) tuples

        Returns:
            Number of vectors inserted
        """
        points = [
            PointStruct(
                id=str(vector_id),
                vector=embedding,
                payload=payload
            )
            for vector_id, embedding, payload in vectors
        ]
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        return len(points)

    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: Optional[float] = None
    ) -> List[ScoredPoint]:
        """
        Search for similar vectors using cosine similarity.

        Args:
            query_vector: Query embedding (1536-dim)
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of ScoredPoint objects with id, score, and payload
        """
        search_params = {
            "collection_name": self.collection_name,
            "query": query_vector,
            "limit": limit,
        }
        if score_threshold is not None:
            search_params["score_threshold"] = score_threshold

        # Use query_points for newer Qdrant client versions
        result = self.client.query_points(**search_params)
        return result.points

    def get_collection_info(self) -> dict:
        """Get collection statistics."""
        try:
            collection = self.client.get_collection(self.collection_name)
            return {
                "vectors_count": collection.vectors_count,
                "points_count": collection.points_count,
                "status": collection.status,
            }
        except Exception as e:
            return {"error": str(e)}


# Global instance
qdrant_service = QdrantService()
