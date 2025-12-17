"""
Embedding service using OpenAI text-embedding-3-small.
Generates 1536-dimensional embeddings for textbook chunks.
"""
from openai import OpenAI
from typing import List
import time

from src.config import settings


class EmbeddingService:
    """Service for generating text embeddings using OpenAI."""

    def __init__(self):
        """Initialize OpenAI client."""
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.embedding_model

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text (textbook chunk)

        Returns:
            1536-dimensional embedding vector
        """
        response = self.client.embeddings.create(
            model=self.model,
            input=text
        )
        return response.data[0].embedding

    def generate_embeddings_batch(
        self,
        texts: List[str],
        batch_size: int = 100,
        delay_seconds: float = 0.1
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts with batching and rate limiting.

        Args:
            texts: List of input texts
            batch_size: Number of texts per API call (max 100)
            delay_seconds: Delay between batches to avoid rate limits

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            # Generate embeddings for batch
            response = self.client.embeddings.create(
                model=self.model,
                input=batch
            )

            # Extract embeddings in order
            batch_embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(batch_embeddings)

            # Rate limiting delay
            if i + batch_size < len(texts):
                time.sleep(delay_seconds)

        return all_embeddings

    def health_check(self) -> bool:
        """Check if OpenAI API is accessible."""
        try:
            self.generate_embedding("test")
            return True
        except Exception:
            return False


# Global instance
embedding_service = EmbeddingService()
