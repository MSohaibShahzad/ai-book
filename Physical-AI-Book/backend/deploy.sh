#!/bin/bash
#
# Deployment Script for Docker-based Backend
#
# This script helps deploy the backend services:
# - auth-service (Node.js + Better Auth) → Vercel/Railway/Render
# - api-service (Python + FastAPI) → Railway/Render/Fly.io
#

set -e

echo "🚀 Physical AI Book Backend Deployment"
echo "======================================"
echo ""

# Show deployment options
echo "Choose deployment option:"
echo ""
echo "1. Deploy auth-service to Vercel (recommended for auth)"
echo "2. Deploy api-service to Railway (recommended for API)"
echo "3. Deploy both with Docker Compose (local/VPS)"
echo "4. Exit"
echo ""
read -p "Enter option (1-4): " option

case $option in
  1)
    echo ""
    echo "🔐 Deploying auth-service to Vercel..."
    echo "========================================"
    echo ""

    # Check if in auth directory
    cd auth/

    # Check Vercel CLI
    if ! command -v vercel &> /dev/null; then
        echo "📦 Installing Vercel CLI..."
        npm i -g vercel
    fi

    # Check dependencies
    if [ ! -d "node_modules" ]; then
        echo "📦 Installing dependencies..."
        npm install
    fi

    # Run migrations
    echo "🗄️  Running database migrations..."
    npm run db:push

    # Deploy
    echo ""
    echo "🌐 Deploying to Vercel..."
    vercel --prod

    echo ""
    echo "✅ Auth service deployed!"
    echo ""
    echo "📝 Post-deployment steps:"
    echo "   1. Note your Vercel URL (e.g., https://your-auth.vercel.app)"
    echo "   2. Update environment variables in Vercel dashboard:"
    echo "      - DATABASE_URL"
    echo "      - BETTER_AUTH_SECRET"
    echo "      - JWT_SECRET"
    echo "      - BETTER_AUTH_URL (set to your Vercel URL)"
    echo "      - NODE_ENV=production"
    echo "   3. Update frontend to use this auth URL"
    echo ""
    ;;

  2)
    echo ""
    echo "🤖 Deploying api-service to Railway..."
    echo "======================================="
    echo ""

    # Check Railway CLI
    if ! command -v railway &> /dev/null; then
        echo "📦 Installing Railway CLI..."
        npm i -g @railway/cli
    fi

    echo "🌐 Deploying to Railway..."
    echo ""
    echo "Manual deployment steps:"
    echo "1. Go to https://railway.app"
    echo "2. Create new project"
    echo "3. Deploy from GitHub (link your repo)"
    echo "4. Set root directory to 'backend'"
    echo "5. Add environment variables:"
    echo "   - QDRANT_URL"
    echo "   - QDRANT_API_KEY"
    echo "   - DATABASE_URL"
    echo "   - OPENAI_API_KEY"
    echo "   - JWT_SECRET (same as auth service!)"
    echo "   - BETTER_AUTH_SECRET"
    echo "   - AUTH_SERVICE_URL (your Vercel auth URL)"
    echo "6. Railway will auto-detect Dockerfile.api"
    echo ""
    echo "Or use Railway CLI:"
    echo "   railway login"
    echo "   railway init"
    echo "   railway up"
    echo ""
    ;;

  3)
    echo ""
    echo "🐳 Deploying with Docker Compose..."
    echo "===================================="
    echo ""

    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        echo "❌ Docker not found. Please install Docker first:"
        echo "   https://docs.docker.com/get-docker/"
        exit 1
    fi

    # Check if .env exists
    if [ ! -f ".env" ]; then
        echo "⚠️  .env file not found!"
        echo "   Copy .env.example to .env and fill in your credentials"
        exit 1
    fi

    # Install auth dependencies
    echo "📦 Installing auth dependencies..."
    cd auth/
    npm install
    cd ..

    # Build and start
    echo ""
    echo "🏗️  Building Docker images..."
    docker compose build

    echo ""
    echo "🚀 Starting services..."
    docker compose up -d

    echo ""
    echo "✅ Services started!"
    echo ""
    echo "Services running:"
    echo "  🔐 Auth: http://localhost:3001"
    echo "  🤖 API:  http://localhost:8000"
    echo ""
    echo "View logs:"
    echo "  docker compose logs -f"
    echo ""
    echo "Stop services:"
    echo "  docker compose down"
    echo ""
    ;;

  4)
    echo "Exiting..."
    exit 0
    ;;

  *)
    echo "Invalid option"
    exit 1
    ;;
esac
