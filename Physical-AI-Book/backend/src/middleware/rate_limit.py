"""
Rate limiting middleware using slowapi.
Limits requests per session_id to prevent abuse.
"""
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request
from src.config import settings


def get_session_id(request: Request) -> str:
    """
    Extract session_id from request for rate limiting.
    Falls back to IP address if no session_id provided.
    """
    # Try to get session_id from request body (for POST requests)
    if hasattr(request, "_json"):
        body = request._json
        if body and isinstance(body, dict) and "session_id" in body:
            return body["session_id"]

    # Fall back to IP address
    return get_remote_address(request)


# Create limiter instance
limiter = Limiter(
    key_func=get_session_id,
    default_limits=[f"{settings.rate_limit_per_minute}/minute"],
    headers_enabled=True,  # Add X-RateLimit headers to responses
)


# Export exception handler
rate_limit_exceeded_handler = _rate_limit_exceeded_handler
