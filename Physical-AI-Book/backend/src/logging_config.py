"""
Logging configuration for RAG Chatbot backend.
Sets up structured logging with appropriate formatters and handlers.
"""
import logging
import sys
from src.config import settings


def setup_logging():
    """Configure logging for the application."""

    # Get log level from settings
    log_level = getattr(logging, settings.log_level.upper(), logging.INFO)

    # Create formatters
    if settings.environment == "production":
        # JSON format for production (easier to parse by log aggregators)
        formatter = logging.Formatter(
            '{"time": "%(asctime)s", "level": "%(levelname)s", "name": "%(name)s", "message": "%(message)s"}'
        )
    else:
        # Human-readable format for development
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)

    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Add console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # Configure specific loggers
    loggers_config = {
        "rag_chatbot": log_level,           # Our application logs
        "rag_chatbot.requests": log_level,  # Request logging
        "uvicorn": logging.WARNING,         # Uvicorn server logs
        "fastapi": logging.WARNING,         # FastAPI logs
        "slowapi": logging.WARNING,         # Rate limiting logs
        "httpx": logging.WARNING,           # HTTP client logs
        "openai": logging.WARNING,          # OpenAI SDK logs
    }

    for logger_name, level in loggers_config.items():
        logger = logging.getLogger(logger_name)
        logger.setLevel(level)

    # Log startup message
    logging.getLogger("rag_chatbot").info(
        f"Logging initialized - Level: {settings.log_level.upper()}, Environment: {settings.environment}"
    )
