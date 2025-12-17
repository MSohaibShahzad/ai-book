"""
Authentication middleware for FastAPI to integrate with Better-Auth server.

This middleware validates user sessions by communicating with the Node.js Better-Auth server
and attaches user information to the request state for use in route handlers.
"""
import logging
import httpx
from typing import Optional, Dict, Any
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)

AUTH_SERVER_URL = "http://localhost:3001"


class AuthUser:
    """User data from Better-Auth session"""

    def __init__(self, data: Dict[str, Any]):
        self.id: str = data.get("id", "")
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


async def get_session_from_auth_server(cookie_header: str) -> Optional[AuthUser]:
    """
    Validate session with Better-Auth server and return user data.

    Args:
        cookie_header: Cookie header string from the request

    Returns:
        AuthUser object if session is valid, None otherwise
    """
    try:
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{AUTH_SERVER_URL}/api/validate-session",
                headers={"Cookie": cookie_header},
                timeout=5.0,
            )

            if response.status_code != 200:
                logger.warning(f"Session validation failed: {response.status_code}")
                return None

            data = response.json()

            if not data.get("user"):
                return None

            return AuthUser(data["user"])

    except httpx.RequestError as e:
        logger.error(f"Error connecting to auth server: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error validating session: {e}")
        return None


async def auth_middleware(request: Request, call_next):
    """
    FastAPI middleware to validate user sessions.

    Attaches user data to request.state.user if session is valid.
    """
    # Get cookie header
    cookie_header = request.headers.get("cookie", "")

    # Validate session
    user = None
    if cookie_header:
        user = await get_session_from_auth_server(cookie_header)

    # Attach user to request state
    request.state.user = user

    # Continue processing request
    response = await call_next(request)
    return response


def get_current_user(request: Request) -> Optional[AuthUser]:
    """
    Helper function to get current authenticated user from request.

    Usage in route handlers:
    ```python
    from src.middleware.auth_middleware import get_current_user

    @router.post("/chat")
    async def chat(request: Request):
        user = get_current_user(request)
        if user:
            # User is authenticated
            print(f"User {user.name} ({user.email})")
            print(f"Background: {user.software_background}, {user.hardware_background}")
        else:
            # User is not authenticated
            pass
    ```
    """
    return getattr(request.state, "user", None)


def require_auth(request: Request) -> AuthUser:
    """
    Dependency for routes that require authentication.

    Usage:
    ```python
    from fastapi import Depends
    from src.middleware.auth_middleware import require_auth, AuthUser

    @router.post("/protected")
    async def protected_route(user: AuthUser = Depends(require_auth)):
        return {"message": f"Hello {user.name}"}
    ```

    Raises:
        HTTPException: 401 Unauthorized if user is not authenticated
    """
    user = get_current_user(request)
    if not user:
        raise HTTPException(status_code=401, detail="Authentication required")
    return user


def get_user_context_for_llm(user: Optional[AuthUser]) -> str:
    """
    Generate context string for LLM based on user's background.

    This context is prepended to the chatbot system prompt to personalize responses.

    Args:
        user: Authenticated user or None

    Returns:
        Context string for LLM prompt
    """
    if not user:
        return "The user is not logged in. Provide general responses suitable for all levels."

    context_parts = [
        f"The user's name is {user.name}.",
        f"Software programming background: {user.software_background}.",
        f"Hardware/robotics background: {user.hardware_background}.",
        f"Primary interest area: {user.interest_area}.",
    ]

    # Add personalization hints based on background
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
