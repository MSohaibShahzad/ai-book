"""
Request models for RAG Chatbot API.
Based on data-model.md and contracts/openapi.yaml.
"""
from pydantic import BaseModel, Field
from typing import List, Optional


class ConversationMessage(BaseModel):
    """Individual message in conversation history."""
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str = Field(..., min_length=1)


class ChatRequest(BaseModel):
    """
    Request model for POST /chat endpoint.

    Validates:
    - message: 1-2000 characters
    - highlighted_text: optional, max 5000 characters
    - session_id: UUID format (36 characters)
    - conversation_history: optional, max 10 messages
    """
    message: str = Field(..., min_length=1, max_length=2000)
    highlighted_text: Optional[str] = Field(None, max_length=5000)
    session_id: str = Field(..., min_length=36, max_length=36)
    conversation_history: Optional[List[ConversationMessage]] = Field(None, max_length=10)

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is inverse kinematics?",
                "highlighted_text": None,
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "conversation_history": []
            }
        }
