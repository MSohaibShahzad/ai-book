"""
Vercel serverless entry point for FastAPI application.
This file is required by Vercel to properly route requests to the FastAPI app.
"""
import sys
from pathlib import Path

# Add the parent directory to Python path
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

# Import the FastAPI app
from src.main import app

# Vercel expects the ASGI app to be named 'app'
__all__ = ['app']
