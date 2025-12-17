"""
Request logging middleware for monitoring and debugging.
Logs all incoming requests with timing, status codes, and metadata.
"""
import time
import logging
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import json

logger = logging.getLogger("rag_chatbot.requests")


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """Middleware to log all HTTP requests with timing and metadata."""

    async def dispatch(self, request: Request, call_next):
        """
        Process request and log details.

        Logs:
        - Request method, path, client IP
        - Response status code
        - Processing time
        - For /chat endpoint: session_id, retrieval_count
        """
        start_time = time.time()

        # Extract request metadata
        client_ip = request.client.host if request.client else "unknown"
        method = request.method
        path = request.url.path

        # For chat requests, try to extract session_id
        session_id = None
        if path == "/v1/chat" and method == "POST":
            try:
                body = await request.body()
                # Re-populate body for downstream handlers
                async def receive():
                    return {"type": "http.request", "body": body}

                request._receive = receive

                # Parse session_id if available
                if body:
                    try:
                        data = json.loads(body)
                        session_id = data.get("session_id", "unknown")
                    except:
                        pass
            except:
                pass

        # Process request
        try:
            response: Response = await call_next(request)
            status_code = response.status_code
        except Exception as e:
            # Log error and re-raise
            logger.error(
                f"Request failed: {method} {path} - Error: {str(e)}",
                extra={
                    "method": method,
                    "path": path,
                    "client_ip": client_ip,
                    "error": str(e),
                }
            )
            raise

        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000  # ms

        # Build log message
        log_data = {
            "method": method,
            "path": path,
            "status_code": status_code,
            "processing_time_ms": round(processing_time, 2),
            "client_ip": client_ip,
        }

        if session_id:
            log_data["session_id"] = session_id

        # Determine log level based on status code
        if status_code >= 500:
            log_level = logging.ERROR
        elif status_code >= 400:
            log_level = logging.WARNING
        else:
            log_level = logging.INFO

        # Log request
        logger.log(
            log_level,
            f"{method} {path} - {status_code} - {round(processing_time, 2)}ms",
            extra=log_data
        )

        return response
