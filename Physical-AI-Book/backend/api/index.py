"""
Vercel serverless entry point for FastAPI application.
This file is required by Vercel to properly route requests to the FastAPI app.
"""
from src.main import app

# Vercel expects the ASGI app to be named 'app' and exported from api/index.py
# This file simply imports and re-exports the FastAPI app
