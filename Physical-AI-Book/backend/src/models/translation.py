"""
Pydantic models for Urdu translation feature.

Feature: 004-urdu-translation
Models: TranslationCache, TranslationLog, TranslationAction enum
"""

from pydantic import BaseModel, Field
from datetime import datetime
from enum import Enum
from typing import Optional


class TranslationAction(str, Enum):
    """Translation action types for analytics logging."""
    TRANSLATE_REQUESTED = "translate_requested"
    TOGGLE_TO_ENGLISH = "toggle_to_english"
    TOGGLE_TO_URDU = "toggle_to_urdu"
    TRANSLATION_FAILED = "translation_failed"


class TranslationCache(BaseModel):
    """
    Model for cached translations.

    Attributes:
        id: Unique cache entry identifier
        chapter_slug: Chapter identifier (e.g., "01-what-is-ros2")
        language: Target language code ("ur" for Urdu)
        content_hash: SHA-256 hash of source content for versioning
        translated_content: Full markdown content in target language
        created_at: When translation was first cached
        accessed_at: Last time cached translation was retrieved
        expires_at: When cached translation expires (TTL-based)
    """
    id: int
    chapter_slug: str = Field(..., max_length=255)
    language: str = Field(..., max_length=10)
    content_hash: str = Field(..., min_length=64, max_length=64)
    translated_content: str
    created_at: datetime
    accessed_at: datetime
    expires_at: datetime

    class Config:
        from_attributes = True


class TranslationCacheCreate(BaseModel):
    """Model for creating new cache entries."""
    chapter_slug: str = Field(..., max_length=255)
    language: str = Field(default="ur", max_length=10)
    content_hash: str = Field(..., min_length=64, max_length=64)
    translated_content: str
    expires_at: datetime


class TranslationLog(BaseModel):
    """
    Model for translation analytics logging.

    Attributes:
        id: Unique log entry identifier
        user_id: User who performed the action (from auth system - TEXT type)
        chapter_slug: Chapter identifier
        action: Type of action (translate, toggle, etc.)
        timestamp: When action occurred
    """
    id: int
    user_id: str  # Changed from int to str to match database schema
    chapter_slug: str = Field(..., max_length=255)
    action: TranslationAction
    timestamp: datetime

    class Config:
        from_attributes = True


class TranslationLogCreate(BaseModel):
    """Model for creating new log entries."""
    user_id: str  # Changed from int to str
    chapter_slug: str = Field(..., max_length=255)
    action: TranslationAction


class TranslateRequest(BaseModel):
    """
    Request model for translation endpoint.

    Validates chapter_slug format and target language.
    """
    chapter_slug: str = Field(
        ...,
        pattern=r'^[a-z0-9][a-z0-9/-]*[a-z0-9]$',
        description="Chapter identifier (e.g., 'what-is-ros2', 'preface/index', 'vision-language-action/voice-to-action-pipelines')",
        examples=["what-is-ros2", "llms-for-cognitive-planning", "preface/index", "vision-language-action/voice-to-action-pipelines"]
    )
    target_language: str = Field(
        default="ur",
        pattern=r'^ur$',
        description="Target language code (currently only Urdu supported)"
    )


class TranslateResponse(BaseModel):
    """
    Response model for successful translation.

    Attributes:
        status: "success" or "error"
        translated_content: Full markdown content in Urdu
        from_cache: Whether translation was served from cache
        cached_at: When translation was originally cached (null if not from cache)
        message: Additional information (optional)
    """
    status: str = Field(default="success")
    translated_content: str
    from_cache: bool = Field(default=False)
    cached_at: Optional[datetime] = None
    message: Optional[str] = None


class LogActionRequest(BaseModel):
    """
    Request model for analytics logging endpoint.

    Validates chapter_slug and action type.
    """
    chapter_slug: str = Field(
        ...,
        pattern=r'^[a-z0-9][a-z0-9/-]*[a-z0-9]$',
        description="Chapter identifier"
    )
    action: TranslationAction


class LogActionResponse(BaseModel):
    """Response model for analytics logging."""
    logged: bool = Field(default=True)


class ErrorResponse(BaseModel):
    """
    Error response model for all translation endpoints.

    Provides user-friendly error messages per constitution principle XX.
    """
    status: str = Field(default="error")
    message: str
    retry_after: Optional[int] = None
