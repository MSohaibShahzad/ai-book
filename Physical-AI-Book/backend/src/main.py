"""
FastAPI application entry point for RAG Chatbot backend.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from src.config import settings
from src.api.routes import health, chat
from src.middleware.rate_limit import limiter
from src.middleware.logging import RequestLoggingMiddleware
from src.logging_config import setup_logging

# Initialize logging
setup_logging()

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API for Physical-AI Textbook",
    version="1.0.0",
    description="REST API for textbook question answering using RAG",
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Configure CORS - allow all Vercel deployments
app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=r"(https://.*\.vercel\.app|http://localhost:\d+)",  # Allow all Vercel deployments
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
)

# Add request logging
app.add_middleware(RequestLoggingMiddleware)

# Add authentication middleware
from src.middleware.auth_middleware import auth_middleware
app.middleware("http")(auth_middleware)

# Include routers
app.include_router(health.router, prefix="/v1", tags=["health"])
app.include_router(chat.router, prefix="/v1", tags=["chat"])


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running"
    }
