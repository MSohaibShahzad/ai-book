"""
Vercel serverless entry point for FastAPI application.
This file is required by Vercel to properly route requests to the FastAPI app.
"""
import sys
import os

# Add the parent directory to the Python path for imports to work
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from src.main import app
except ImportError as e:
    print(f"Import error: {e}")
    print(f"Python path: {sys.path}")
    print(f"Current directory: {os.getcwd()}")
    print(f"Directory contents: {os.listdir('.')}")
    raise

# Vercel expects the ASGI app to be named 'app' and exported from api/index.py
# This file simply imports and re-exports the FastAPI app
