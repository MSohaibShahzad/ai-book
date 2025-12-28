"""
JWT Authentication Middleware for FastAPI

This middleware verifies JWT tokens issued by the auth-service.
It does NOT handle authentication - only token verification.
"""
import logging
import jwt
from typing import Optional, Dict, Any
from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)

# Import settings to get JWT_SECRET
from ..config import settings

JWT_SECRET = settings.jwt_secret or settings.better_auth_secret
JWT_ALGORITHM = "HS256"


class AuthUser:
    """User data extracted from JWT token"""

    def __init__(self, data: Dict[str, Any]):
        self.id: str = data.get("userId", "")
        self.name: str = data.get("name", "")
        self.email: str = data.get("email", "")
        self.email_verified: bool = data.get("emailVerified", False)
        self.software_background: str = data.get("softwareBackground", "Beginner")
        self.hardware_background: str = data.get("hardwareBackground", "None")
        self.interest_area: str = data.get("interestArea", "AI")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "email": self.email,
            "email_verified": self.email_verified,
            "software_background": self.software_background,
            "hardware_background": self.hardware_background,
            "interest_area": self.interest_area,
        }


def verify_jwt_token(token: str) -> Optional[AuthUser]:
    """
    Verify JWT token and extract user data.

    Args:
        token: JWT token string (without "Bearer " prefix)

    Returns:
        AuthUser object if valid, None otherwise
    """
    try:
        logger.debug(f"[JWT Verify] Using secret: {JWT_SECRET[:10]}...")
        logger.debug(f"[JWT Verify] Algorithm: {JWT_ALGORITHM}")

        # Verify and decode JWT
        payload = jwt.decode(
            token,
            JWT_SECRET,
            algorithms=[JWT_ALGORITHM],
            issuer="auth-service",
            audience="api-service",
        )

        logger.info(f"[JWT Verify] Token verified successfully. User: {payload.get('email')}")
        return AuthUser(payload)

    except jwt.ExpiredSignatureError:
        logger.warning("JWT token has expired")
        return None
    except jwt.InvalidTokenError as e:
        logger.warning(f"Invalid JWT token: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error verifying JWT: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return None


class JWTAuthMiddleware(BaseHTTPMiddleware):
    """
    Middleware to verify JWT tokens and attach user to request state.

    Extracts JWT from Authorization header (Bearer token) and verifies it.
    Attaches user data to request.state.user if valid.
    """

    async def dispatch(self, request: Request, call_next):
        # Extract Authorization header
        auth_header = request.headers.get("authorization", "")

        logger.info(f"[JWT Middleware] Processing request to {request.url.path}")
        logger.info(f"[JWT Middleware] Authorization header present: {bool(auth_header)}")

        user = None
        if auth_header.startswith("Bearer "):
            token = auth_header[7:]  # Remove "Bearer " prefix
            logger.info(f"[JWT Middleware] Token extracted: {token[:20]}...")
            user = verify_jwt_token(token)
            if user:
                logger.info(f"[JWT Middleware] User authenticated: {user.email}")
            else:
                logger.warning("[JWT Middleware] Token verification failed")
        else:
            logger.warning(f"[JWT Middleware] No Bearer token found. Header: {auth_header[:50] if auth_header else 'empty'}")

        # Attach user to request state
        request.state.user = user

        # Continue processing
        response = await call_next(request)
        return response


def get_current_user(request: Request) -> Optional[AuthUser]:
    """
    Helper to get authenticated user from request.

    Usage in route handlers:
    ```python
    from src.middleware.jwt_auth import get_current_user

    @router.post("/chat")
    async def chat(request: Request):
        user = get_current_user(request)
        if user:
            print(f"User: {user.name} ({user.email})")
        else:
            # Anonymous user
            pass
    ```
    """
    return getattr(request.state, "user", None)


def get_user_context_for_llm(user: Optional[AuthUser]) -> str:
    """
    Generate LLM context string based on user's background.

    Args:
        user: Authenticated user or None

    Returns:
        Context string to prepend to system prompt
    """
    if not user:
        return "The user is not logged in. Provide general responses suitable for all levels."

    context_parts = [
        f"The user's name is {user.name}.",
        f"Software programming background: {user.software_background}.",
        f"Hardware/robotics background: {user.hardware_background}.",
        f"Primary interest area: {user.interest_area}.",
    ]

    # Personalization hints
    if user.software_background == "Beginner":
        context_parts.append(
            "Explain code concepts clearly with examples. Avoid assuming prior knowledge."
        )
    elif user.software_background == "Expert":
        context_parts.append(
            "You can use advanced programming concepts. Be concise and technical."
        )

    if user.hardware_background == "None":
        context_parts.append(
            "Explain hardware concepts from the ground up. Use analogies."
        )
    elif user.hardware_background == "Advanced":
        context_parts.append(
            "Assume familiarity with robotics hardware and electronics."
        )

    if user.interest_area == "AI":
        context_parts.append("Emphasize AI and machine learning aspects of the topic.")
    elif user.interest_area == "Robotics":
        context_parts.append(
            "Emphasize mechanical and control systems aspects of the topic."
        )
    elif user.interest_area == "Computer Vision":
        context_parts.append("Emphasize vision and perception aspects of the topic.")
    elif user.interest_area == "Motion Control":
        context_parts.append("Emphasize motion planning and control aspects of the topic.")

    return " ".join(context_parts)
