"""
FastAPI application entry point for RAG Chatbot backend.

This service handles ONLY:
- RAG chatbot logic
- Vector database queries
- LLM generation

Authentication is handled by a separate auth-service (Node.js + Better Auth).
This service only verifies JWT tokens.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from src.config import settings
from src.api.routes import health, chat, translation
from src.middleware.rate_limit import limiter
from src.middleware.logging import RequestLoggingMiddleware
from src.middleware.jwt_auth import JWTAuthMiddleware
from src.logging_config import setup_logging
from personalization.api import router as personalization_router

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

# Configure CORS - allow localhost and all Vercel deployments
# Dynamically reads from CORS_ORIGINS environment variable
cors_origins = settings.get_cors_origins() if settings.cors_origins else [
    "http://localhost:3000",  # Frontend (Docusaurus)
    "http://localhost:3001",  # Auth service (Better-Auth)
    "http://localhost:8000",  # Backend (self)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_origin_regex=r"https://.*\.vercel\.app",  # All Vercel deployments
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
)

# Add request logging
app.add_middleware(RequestLoggingMiddleware)

# Add JWT authentication middleware (verifies tokens issued by auth-service)
app.add_middleware(JWTAuthMiddleware)

# Include routers
app.include_router(health.router, prefix="/v1", tags=["health"])
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(translation.router, prefix="/v1", tags=["translation"])
app.include_router(personalization_router)  # Already has /api/personalization prefix


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running"
    }
