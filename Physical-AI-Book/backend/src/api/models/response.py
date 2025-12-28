"""
Response models for RAG Chatbot API.
Based on data-model.md and contracts/openapi.yaml.
"""
from pydantic import BaseModel, Field
from typing import List


class SourceReference(BaseModel):
    """Source reference for retrieved textbook passages."""
    module_name: str = Field(..., min_length=1)
    chapter_name: str = Field(..., min_length=1)
    slug: str = Field(..., min_length=1)
    chunk_text_preview: str = Field(..., max_length=200)

    class Config:
        json_schema_extra = {
            "example": {
                "module_name": "Robot Manipulation",
                "chapter_name": "Chapter 3: Inverse Kinematics",
                "slug": "/docs/robot-manipulation/inverse-kinematics",
                "chunk_text_preview": "Inverse kinematics (IK) solves the problem of determining joint angles..."
            }
        }


class ChatResponse(BaseModel):
    """
    Response model for POST /chat endpoint.

    Returns:
    - response: Markdown-formatted answer with source attribution
    - sources: List of 0-5 source references
    - session_id: Echo of request session_id
    - retrieval_count: Number of chunks retrieved
    - processing_time_ms: Total processing time
    """
    response: str = Field(..., min_length=1)
    sources: List[SourceReference] = Field(..., min_length=0, max_length=5)
    session_id: str = Field(..., min_length=36, max_length=36)
    retrieval_count: int = Field(..., ge=0)
    processing_time_ms: float = Field(..., ge=0)

    class Config:
        json_schema_extra = {
            "example": {
                "response": "Inverse kinematics (IK) solves the problem...\n\nSource: Module 2, Chapter 3",
                "sources": [
                    {
                        "module_name": "Robot Manipulation",
                        "chapter_name": "Chapter 3: Inverse Kinematics",
                        "slug": "/docs/robot-manipulation/inverse-kinematics",
                        "chunk_text_preview": "Inverse kinematics (IK) solves..."
                    }
                ],
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "retrieval_count": 3,
                "processing_time_ms": 847.3
            }
        }


class HealthResponse(BaseModel):
    """Response model for GET /health endpoint."""
    status: str = Field(..., pattern="^(healthy|degraded|unhealthy)$")
    services: dict = Field(..., min_length=3)
    version: str = Field(default="1.0.0")
    timestamp: str

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "services": {
                    "qdrant": True,
                    "postgres": True,
                    "openai": True
                },
                "version": "1.0.0",
                "timestamp": "2025-12-10T23:59:59Z"
            }
        }


class ErrorResponse(BaseModel):
    """Error response model for all error cases."""
    error: str = Field(..., min_length=1)
    detail: str = Field(default="")
    status_code: int = Field(..., ge=100, le=599)

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Validation error",
                "detail": "message: Field required",
                "status_code": 400
            }
        }
