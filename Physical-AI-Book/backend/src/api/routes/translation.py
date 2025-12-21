"""
Translation API routes.

Feature: 004-urdu-translation
Endpoints: POST /api/translate, POST /api/translate/log, DELETE /api/translate/cache
"""

from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.responses import JSONResponse
from typing import Dict
import psycopg2
from datetime import datetime
import os

from ...models.translation import (
    TranslateRequest,
    TranslateResponse,
    LogActionRequest,
    LogActionResponse,
    ErrorResponse,
    TranslationAction
)
from ...services.translation_service import TranslationService
from ...middleware.jwt_auth import get_current_user

router = APIRouter(prefix="/api/translate", tags=["translation"])

# Initialize translation service with correct path to textbook docs
translation_service = TranslationService(textbook_docs_path="../textbook/docs")


@router.post(
    "",
    response_model=TranslateResponse,
    responses={
        400: {"model": ErrorResponse},
        401: {"model": ErrorResponse},
        429: {"model": ErrorResponse},
        500: {"model": ErrorResponse}
    }
)
async def translate_chapter(
    request: TranslateRequest,
    current_user = Depends(get_current_user)
):
    """
    Translate a chapter to Urdu.

    Authentication required. Checks cache first, translates if cache miss.

    Args:
        request: Translation request with chapter_slug
        current_user: Authenticated user from JWT token

    Returns:
        TranslateResponse with translated content

    Raises:
        HTTPException: For various error conditions
    """
    # Check authentication
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required. Please log in to use translation features."
        )

    try:
        # Translate chapter (checks cache internally)
        result = await translation_service.translate_chapter(
            chapter_slug=request.chapter_slug,
            target_language=request.target_language
        )

        # Log translation request
        await log_translation_action(
            user_id=current_user.id,
            chapter_slug=request.chapter_slug,
            action=TranslationAction.TRANSLATE_REQUESTED
        )

        return TranslateResponse(
            status="success",
            translated_content=result["translated_content"],
            from_cache=result["from_cache"],
            cached_at=result.get("cached_at"),
            message="Translation generated successfully" if not result["from_cache"]
                    else "Translation served from cache"
        )

    except FileNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Chapter not found: {request.chapter_slug}"
        )

    except Exception as e:
        # Log translation failure
        try:
            await log_translation_action(
                user_id=current_user.id,
                chapter_slug=request.chapter_slug,
                action=TranslationAction.TRANSLATION_FAILED
            )
        except:
            pass  # Don't fail on logging error

        print(f"[ERROR] Translation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Translation failed. Please try again."
        )


@router.post("/log", response_model=LogActionResponse)
async def log_translation_request(
    request: LogActionRequest,
    current_user = Depends(get_current_user)
):
    """
    Log translation action for analytics.

    Authentication required.

    Args:
        request: Log action request with chapter_slug and action
        current_user: Authenticated user from JWT token

    Returns:
        LogActionResponse confirming action was logged
    """
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    try:
        logged = await log_translation_action(
            user_id=current_user.id,
            chapter_slug=request.chapter_slug,
            action=request.action
        )

        return LogActionResponse(logged=logged)

    except Exception as e:
        print(f"[ERROR] Logging failed: {e}")
        # Don't fail the request if logging fails
        return LogActionResponse(logged=False)


@router.delete("/cache/{chapter_slug}")
async def invalidate_chapter_cache(
    chapter_slug: str,
    language: str = "ur",
    current_user = Depends(get_current_user)
):
    """
    Invalidate cached translation for a chapter (admin only).

    Authentication required. Admin-only endpoint for cache management.

    Args:
        chapter_slug: Chapter identifier
        language: Language code (default: "ur")
        current_user: Authenticated user from JWT token

    Returns:
        JSON with invalidation status
    """
    # TODO: Add admin role check
    # For now, allow any authenticated user
    # if not current_user.get("is_admin"):
    #     raise HTTPException(
    #         status_code=status.HTTP_403_FORBIDDEN,
    #         detail="Admin access required"
    #     )

    try:
        invalidated_count = await translation_service.invalidate_chapter_cache(
            chapter_slug=chapter_slug,
            language=language
        )

        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content={
                "status": "success",
                "message": f"Cache invalidated for chapter: {chapter_slug}",
                "invalidated_count": invalidated_count
            }
        )

    except Exception as e:
        print(f"[ERROR] Cache invalidation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Cache invalidation failed"
        )


@router.get("/stats")
async def get_translation_stats(
    current_user = Depends(get_current_user)
):
    """
    Get translation and cache statistics (admin only).

    Authentication required.

    Args:
        current_user: Authenticated user from JWT token

    Returns:
        JSON with cache statistics
    """
    try:
        stats = await translation_service.get_translation_stats()
        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content=stats
        )

    except Exception as e:
        print(f"[ERROR] Stats retrieval failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve statistics"
        )


# Helper function for logging
async def log_translation_action(
    user_id: int,
    chapter_slug: str,
    action: TranslationAction
) -> bool:
    """
    Log translation action to database.

    Args:
        user_id: User ID from auth system
        chapter_slug: Chapter identifier
        action: Action type enum

    Returns:
        True if logged successfully, False otherwise
    """
    try:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            print("[WARNING] DATABASE_URL not set, skipping logging")
            return False

        conn = psycopg2.connect(database_url)
        cursor = conn.cursor()

        cursor.execute("""
            INSERT INTO translation_log (user_id, chapter_slug, action, timestamp)
            VALUES (%s, %s, %s, CURRENT_TIMESTAMP)
        """, (user_id, chapter_slug, action.value))

        conn.commit()
        cursor.close()
        conn.close()

        return True

    except Exception as e:
        print(f"[ERROR] Failed to log action: {e}")
        return False
