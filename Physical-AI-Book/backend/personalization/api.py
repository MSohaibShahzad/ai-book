"""
Personalization API Endpoints

FastAPI routes for chapter personalization, quota management, and cache invalidation.
"""

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from typing import Optional, Dict, Union
import time
import logging
import asyncio

from .agent import personalize_chapter_async
from .rate_limiter import check_rate_limit, increment_quota, get_quota_status
from .metrics import metrics

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/personalization", tags=["personalization"])


# ============================================================================
# Request/Response Models
# ============================================================================

class UserProfile(BaseModel):
    """User profile for personalization"""
    softwareBackground: str = Field(..., description="Beginner|Intermediate|Advanced|Expert")
    hardwareBackground: str = Field(..., description="None|Beginner|Intermediate|Advanced")
    interestArea: str = Field(..., description="AI|Robotics|Computer Vision|Motion Control|General")


class PersonalizationRequest(BaseModel):
    """Request body for POST /api/personalize"""
    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'foundations-ros2/nodes-topics')")
    chapter_content: str = Field(..., description="Original chapter markdown content")
    user_profile: UserProfile = Field(..., description="User's background profile")


class PersonalizationResponse(BaseModel):
    """Response body for POST /api/personalize"""
    personalized_content: str = Field(..., description="Personalized chapter markdown")
    remaining_limit: int = Field(..., ge=0, le=3, description="Remaining personalization requests (0-3)")
    generation_time_ms: float = Field(..., description="Generation time in milliseconds")
    reset_at: str = Field(..., description="ISO 8601 timestamp when quota resets")


class QuotaStatus(BaseModel):
    """Response body for GET /api/personalization/quota"""
    remaining_requests: int = Field(..., ge=0, le=3, description="Remaining requests in current window")
    total_requests: int = Field(3, description="Total allowed requests per day")
    reset_at: str = Field(..., description="ISO 8601 timestamp when quota resets")
    hours_until_reset: float = Field(..., description="Hours until quota resets")


class ErrorResponse(BaseModel):
    """Error response body"""
    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict] = Field(None, description="Additional error context")
    retry_allowed: Optional[bool] = Field(None, description="Whether retry is allowed")
    remaining_limit: Optional[int] = Field(None, description="Remaining quota (if applicable)")


# ============================================================================
# Authentication Dependency
# ============================================================================

async def get_current_user_id(request: Request) -> str:
    """
    Extract user ID from Better-Auth session.

    Returns user ID as string (Better-Auth uses string IDs, not UUIDs).
    Raises HTTPException 401 if not authenticated.
    """
    # First check if user was set by JWT middleware (from Authorization header)
    user = getattr(request.state, "user", None)

    if user:
        # User authenticated via JWT (Authorization header)
        if not user.id:
            logger.error(f"User object missing ID: {user}")
            raise HTTPException(
                status_code=401,
                detail={
                    "error": "unauthorized",
                    "message": "Invalid user ID in session",
                    "login_url": "/signin"
                }
            )
        return str(user.id)  # Return as string (Better-Auth uses string IDs)

    # If not found, check for Better-Auth session cookie (fallback)
    session_cookie = request.cookies.get("better-auth.session_token") or request.cookies.get("better-auth-session")

    if not session_cookie:
        raise HTTPException(
            status_code=401,
            detail={
                "error": "unauthorized",
                "message": "Authentication required to personalize chapters. Please sign in.",
                "login_url": "/signin"
            }
        )

    # If we have a session cookie but no user in state, something is wrong
    raise HTTPException(
        status_code=401,
        detail={
            "error": "unauthorized",
            "message": "Invalid or expired session",
            "login_url": "/signin"
        }
    )


# ============================================================================
# Endpoints
# ============================================================================

@router.post("/personalize", response_model=PersonalizationResponse, status_code=200)
async def personalize_chapter_endpoint(
    request: PersonalizationRequest,
    user_id: str = Depends(get_current_user_id)
):
    """
    Generate personalized chapter content.

    Rate limited to 3 requests per user per day.
    Hard timeout at 30 seconds.

    Errors:
    - 400: Incomplete profile or validation error
    - 401: Unauthorized (not authenticated)
    - 408: Timeout (exceeded 30 seconds)
    - 429: Rate limit exceeded
    - 500: Generation failed
    """
    start_time = time.time()

    try:
        logger.info(f"Personalization request for user {user_id}, chapter {request.chapter_id}")

        # Step 1: Validate user profile
        if not all([
            request.user_profile.softwareBackground,
            request.user_profile.hardwareBackground,
            request.user_profile.interestArea
        ]):
            missing_fields = []
            if not request.user_profile.softwareBackground:
                missing_fields.append("softwareBackground")
            if not request.user_profile.hardwareBackground:
                missing_fields.append("hardwareBackground")
            if not request.user_profile.interestArea:
                missing_fields.append("interestArea")

            raise HTTPException(
                status_code=400,
                detail={
                    "error": "incomplete_profile",
                    "message": "Complete your profile to enable personalization",
                    "details": {"missing_fields": missing_fields},
                    "profile_url": "/profile/settings"
                }
            )

        # Step 2: Check rate limit
        is_allowed, remaining = check_rate_limit(user_id)

        if not is_allowed:
            quota_status = get_quota_status(user_id)
            raise HTTPException(
                status_code=429,
                detail={
                    "error": "rate_limit_exceeded",
                    "message": "Daily personalization limit exceeded. Try again after reset.",
                    "remaining_limit": 0,
                    "reset_at": quota_status["reset_at"],
                    "retry_after_seconds": int(quota_status["hours_until_reset"] * 3600)
                }
            )

        # Step 3: Personalize chapter with timeout
        try:
            personalized_content = await personalize_chapter_async(
                chapter_content=request.chapter_content,
                user_profile=request.user_profile.dict(),
                timeout_seconds=30  # Hard timeout
            )

        except asyncio.TimeoutError:
            # Timeout - still decrement quota
            duration_ms = (time.time() - start_time) * 1000
            metrics.record_failure(duration_ms, is_timeout=True)
            remaining_after_timeout = increment_quota(user_id)

            logger.error(f"Timeout for user {user_id}, chapter {request.chapter_id}")

            raise HTTPException(
                status_code=408,
                detail={
                    "error": "timeout",
                    "message": "Personalization timed out after 30 seconds. Please try again.",
                    "retry_allowed": True,
                    "remaining_limit": remaining_after_timeout
                }
            )

        except TimeoutError as e:
            # Timeout - still decrement quota
            duration_ms = (time.time() - start_time) * 1000
            metrics.record_failure(duration_ms, is_timeout=True)
            remaining_after_timeout = increment_quota(user_id)

            logger.error(f"Timeout for user {user_id}: {e}")

            raise HTTPException(
                status_code=408,
                detail={
                    "error": "timeout",
                    "message": str(e),
                    "retry_allowed": True,
                    "remaining_limit": remaining_after_timeout
                }
            )

        except Exception as e:
            # Generation failed - still decrement quota
            duration_ms = (time.time() - start_time) * 1000
            metrics.record_failure(duration_ms, is_timeout=False)
            remaining_after_error = increment_quota(user_id)

            logger.error(f"Generation failed for user {user_id}: {e}")

            raise HTTPException(
                status_code=500,
                detail={
                    "error": "generation_failed",
                    "message": "Unable to generate personalized content. Please try again.",
                    "retry_allowed": True,
                    "remaining_limit": remaining_after_error
                }
            )

        # Step 4: Success - increment quota and record metrics
        remaining_after_success = increment_quota(user_id)
        duration_ms = (time.time() - start_time) * 1000
        metrics.record_success(duration_ms)

        # Get quota status for reset_at timestamp
        quota_status = get_quota_status(user_id)

        logger.info(f"Personalization successful for user {user_id}, took {duration_ms:.0f}ms")

        return PersonalizationResponse(
            personalized_content=personalized_content,
            remaining_limit=remaining_after_success,
            generation_time_ms=duration_ms,
            reset_at=quota_status["reset_at"]
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        # Unexpected error
        duration_ms = (time.time() - start_time) * 1000
        metrics.record_failure(duration_ms, is_timeout=False)

        logger.error(f"Unexpected error for user {user_id}: {e}")

        raise HTTPException(
            status_code=500,
            detail={
                "error": "internal_error",
                "message": "An unexpected error occurred. Please try again.",
                "retry_allowed": True
            }
        )


@router.get("/quota", response_model=QuotaStatus, status_code=200)
async def get_quota_status_endpoint(user_id: str = Depends(get_current_user_id)):
    """
    Get user's personalization quota status.

    Returns remaining requests, reset timestamp, and hours until reset.
    """
    try:
        logger.info(f"Quota status request for user {user_id}")

        quota_status = get_quota_status(user_id)

        return QuotaStatus(**quota_status)

    except Exception as e:
        logger.error(f"Error getting quota status for user {user_id}: {e}")

        raise HTTPException(
            status_code=500,
            detail={
                "error": "internal_error",
                "message": "Unable to fetch quota status. Please try again."
            }
        )


@router.delete("/cache", status_code=204)
async def invalidate_cache_endpoint(user_id: str = Depends(get_current_user_id)):
    """
    Invalidate personalized content cache (called on profile update).

    Returns 204 No Content on success.
    Cache invalidation happens client-side in React Context.
    """
    try:
        logger.info(f"Cache invalidation request for user {user_id}")

        # Nothing to do on backend - cache is client-side only
        # This endpoint exists for consistency and potential future backend caching

        return JSONResponse(content=None, status_code=204)

    except Exception as e:
        logger.error(f"Error invalidating cache for user {user_id}: {e}")

        raise HTTPException(
            status_code=500,
            detail={
                "error": "internal_error",
                "message": "Unable to invalidate cache. Please try again."
            }
        )


# ============================================================================
# Metrics Endpoint (Public, No Authentication)
# ============================================================================

@router.get("/metrics", status_code=200)
async def get_metrics_endpoint():
    """
    Get personalization metrics for monitoring.

    Public endpoint (no authentication required).
    Used by monitoring dashboards to verify 90% success rate (SC-007).
    """
    try:
        return metrics.to_dict()

    except Exception as e:
        logger.error(f"Error getting metrics: {e}")
        return {
            "total_requests": 0,
            "successful_requests": 0,
            "failed_requests": 0,
            "timeout_requests": 0,
            "avg_duration_ms": 0.0,
            "success_rate_percent": 0.0,
            "last_updated": None
        }
