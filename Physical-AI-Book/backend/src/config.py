"""
Configuration management for RAG Chatbot backend.
Loads environment variables and provides settings via Pydantic BaseSettings.
"""
from pydantic_settings import BaseSettings
from typing import List
import os


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook_chunks"

    # Neon Postgres
    database_url: str

    # OpenAI
    openai_api_key: str
    embedding_model: str = "text-embedding-3-small"
    llm_model: str = "gpt-4o-mini"

    # Application Settings
    environment: str = "development"
    log_level: str = "INFO"
    cors_origins: str = '["http://localhost:3000", "https://ai-book-green.vercel.app", "https://ai-book-ki61.vercel.app"]'
    rate_limit_per_minute: int = 10

    # Better-Auth Integration
    better_auth_secret: str 
    better_auth_url: str 
    node_env: str = "production"

    # Retrieval Settings
    max_retrieval_chunks: int = 5
    chunk_overlap_tokens: int = 50

    # LLM Settings
    llm_temperature: float = 0.7
    llm_max_tokens: int = 1000

    class Config:
        env_file = ".env"
        case_sensitive = False

    def get_cors_origins(self) -> List[str]:
        """Parse CORS origins from JSON string."""
        import json
        try:
            origins = json.loads(self.cors_origins)
            # Add wildcard support for all Vercel preview deployments
            if not any(origin.endswith('*') for origin in origins):
                origins.append("https://*.vercel.app")
            return origins
        except json.JSONDecodeError:
            return [
                "http://localhost:3000",
                "https://ai-book-green.vercel.app",
                "https://ai-book-ki61.vercel.app",
                "https://*.vercel.app"
            ]


# Global settings instance
settings = Settings()
