"""
Auth proxy routes for Better-Auth server.

This module forwards authentication requests to the Better-Auth Node.js server,
allowing the frontend to communicate with a single backend URL.
"""
import httpx
from fastapi import APIRouter, Request, Response, HTTPException
from fastapi.responses import StreamingResponse
from src.config import settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# Better-Auth server URL (can be external or internal microservice)
AUTH_SERVER_URL = settings.better_auth_url


@router.api_route("/auth/{path:path}", methods=["GET", "POST", "PATCH", "DELETE", "OPTIONS"])
async def proxy_auth(path: str, request: Request):
    """
    Proxy all /api/auth/* requests to the Better-Auth server.

    This allows the frontend to make all requests to the FastAPI backend,
    which then forwards auth requests to the Better-Auth Node.js server.
    """
    # Build target URL
    target_url = f"{AUTH_SERVER_URL}/api/auth/{path}"

    # Get request body
    body = None
    if request.method in ["POST", "PATCH", "PUT"]:
        body = await request.body()

    # Get query parameters
    query_params = dict(request.query_params)

    # Get headers (exclude host and content-length)
    headers = {}
    for key, value in request.headers.items():
        if key.lower() not in ["host", "content-length", "connection"]:
            headers[key] = value

    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            # Forward request to Better-Auth server
            response = await client.request(
                method=request.method,
                url=target_url,
                params=query_params,
                headers=headers,
                content=body,
                follow_redirects=False,
            )

            # Build response headers
            response_headers = {}
            for key, value in response.headers.items():
                if key.lower() not in ["content-encoding", "content-length", "transfer-encoding", "connection"]:
                    response_headers[key] = value

            # Return response
            return Response(
                content=response.content,
                status_code=response.status_code,
                headers=response_headers,
            )

    except httpx.RequestError as e:
        logger.error(f"Error proxying auth request to {target_url}: {e}")
        raise HTTPException(status_code=502, detail=f"Auth server unavailable: {str(e)}")
    except Exception as e:
        logger.error(f"Unexpected error proxying auth request: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/validate-session")
async def validate_session(request: Request):
    """
    Proxy session validation endpoint to Better-Auth server.

    This endpoint is used by the auth middleware to validate user sessions.
    """
    target_url = f"{AUTH_SERVER_URL}/api/validate-session"

    # Get cookie header
    cookie_header = request.headers.get("cookie", "")

    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(
                target_url,
                headers={"Cookie": cookie_header},
            )

            if response.status_code != 200:
                raise HTTPException(status_code=response.status_code, detail="Session validation failed")

            return response.json()

    except httpx.RequestError as e:
        logger.error(f"Error validating session: {e}")
        raise HTTPException(status_code=502, detail="Auth server unavailable")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error validating session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
