"""
Chapter Personalization Module

This module provides AI-powered personalization of textbook chapters based on user background.

Components:
- agent.py: OpenAI personalization agent with AST-based markdown preservation
- api.py: FastAPI endpoints for personalization requests
- rate_limiter.py: Rate limiting logic (3 requests/day per user)
- metrics.py: Observability and metrics tracking
- models.py: SQLAlchemy models for personalization_quota table
"""

__version__ = "1.0.0"
__author__ = "AI Book Team"
