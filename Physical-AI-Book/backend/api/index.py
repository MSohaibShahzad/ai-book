"""
Vercel serverless entry point for FastAPI application.
Wraps the ASGI app to work with Vercel's serverless function handler.
"""
import sys
from pathlib import Path
from mangum import Mangum

# Add the parent directory to Python path
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

# Import the FastAPI app
from src.main import app as fastapi_app

# Wrap FastAPI app with Mangum for AWS Lambda/Vercel compatibility
handler = Mangum(fastapi_app, lifespan="off")

# Export as 'app' for Vercel (though Vercel will use 'handler')
app = fastapi_app
