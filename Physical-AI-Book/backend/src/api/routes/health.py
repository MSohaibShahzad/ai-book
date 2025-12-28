"""
Health check endpoint for RAG Chatbot API.
Returns status of all dependent services.
"""
from fastapi import APIRouter
from datetime import datetime

from src.api.models.response import HealthResponse
from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service
from src.services.embeddings import embedding_service

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse with service status
    """
    # Check all services
    qdrant_healthy = qdrant_service.health_check()
    postgres_healthy = postgres_service.health_check()
    openai_healthy = embedding_service.health_check()

    # Determine overall status
    services = {
        "qdrant": qdrant_healthy,
        "postgres": postgres_healthy,
        "openai": openai_healthy
    }

    if all(services.values()):
        status = "healthy"
    elif any(services.values()):
        status = "degraded"
    else:
        status = "unhealthy"

    return HealthResponse(
        status=status,
        services=services,
        version="1.0.0",
        timestamp=datetime.utcnow().isoformat() + "Z"
    )
