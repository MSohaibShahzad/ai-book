#!/bin/bash

echo "========================================="
echo "  Chapter Personalization - Quick Test"
echo "========================================="
echo ""

# Check backend
echo "1. Checking backend (port 8000)..."
if curl -s http://localhost:8000/health > /dev/null 2>&1; then
  echo "   ✅ Backend is RUNNING"
  curl -s http://localhost:8000/health | head -1
else
  echo "   ❌ Backend NOT running"
  echo "   Start with: cd backend && uvicorn main:app --reload --port 8000"
fi

echo ""

# Check frontend
echo "2. Checking frontend (port 3000)..."
if curl -s http://localhost:3000 > /dev/null 2>&1; then
  echo "   ✅ Frontend is RUNNING"
else
  echo "   ❌ Frontend NOT running"
  echo "   Start with: cd textbook && npm start"
fi

echo ""

# Check personalization API
echo "3. Checking personalization API..."
METRICS=$(curl -s http://localhost:8000/api/personalization/metrics 2>&1)
if [[ $METRICS == *"total_requests"* ]]; then
  echo "   ✅ Personalization API working"
  echo "   Metrics: $METRICS"
else
  echo "   ⚠️  Personalization API not responding (backend may not be running)"
fi

echo ""
echo "========================================="
echo ""

if curl -s http://localhost:8000/health > /dev/null 2>&1 && curl -s http://localhost:3000 > /dev/null 2>&1; then
  echo "✅ ALL SYSTEMS READY!"
  echo ""
  echo "Test the feature:"
  echo "1. Go to: http://localhost:3000/docs/foundations-ros2/what-is-ros2-personalized-example"
  echo "2. Sign in (or sign up)"
  echo "3. Complete profile if needed"
  echo "4. Click 'Personalize for Me' button"
  echo ""
else
  echo "⚠️  Services need to be started first"
  echo ""
  echo "Start backend:"
  echo "  cd backend"
  echo "  source venv/bin/activate"
  echo "  uvicorn main:app --reload --port 8000"
  echo ""
  echo "Start frontend (in another terminal):"
  echo "  cd textbook"
  echo "  npm start"
  echo ""
fi
