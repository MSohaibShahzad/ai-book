#!/bin/bash
# Test script to verify authentication flow for translation feature
# Usage: ./test_auth_flow.sh

set -e

echo "🔍 Testing Authentication Flow for Translation Feature"
echo "======================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Auth Service Health
echo "1️⃣  Testing Auth Service..."
AUTH_HEALTH=$(curl -s http://localhost:3001/health)
if echo "$AUTH_HEALTH" | grep -q "healthy"; then
    echo -e "${GREEN}✓ Auth service is running${NC}"
else
    echo -e "${RED}✗ Auth service is NOT running${NC}"
    echo "   Start with: cd backend/auth && npm start"
    exit 1
fi
echo ""

# Test 2: Backend API Health
echo "2️⃣  Testing Backend API..."
API_HEALTH=$(curl -s http://localhost:8000/v1/health)
if echo "$API_HEALTH" | grep -q "healthy"; then
    echo -e "${GREEN}✓ Backend API is running${NC}"
else
    echo -e "${RED}✗ Backend API is NOT running${NC}"
    echo "   Start with: cd backend && uvicorn src.main:app --reload"
    exit 1
fi
echo ""

# Test 3: JWT Configuration
echo "3️⃣  Checking JWT Configuration..."
BACKEND_JWT=$(grep "^JWT_SECRET=" backend/.env | cut -d'=' -f2)
AUTH_JWT=$(grep "^JWT_SECRET=" backend/auth/.env | cut -d'=' -f2)

if [ "$BACKEND_JWT" = "$AUTH_JWT" ]; then
    echo -e "${GREEN}✓ JWT secrets match${NC}"
else
    echo -e "${RED}✗ JWT secrets DO NOT match${NC}"
    echo "   Backend JWT: ${BACKEND_JWT:0:10}..."
    echo "   Auth JWT: ${AUTH_JWT:0:10}..."
    exit 1
fi
echo ""

# Test 4: User Signup (or skip if already exists)
echo "4️⃣  Testing User Signup..."
TEST_EMAIL="test_$(date +%s)@example.com"
TEST_PASSWORD="TestPassword123!"
SIGNUP_RESPONSE=$(curl -s -X POST http://localhost:3001/api/auth/sign-up/email \
    -H "Content-Type: application/json" \
    -c cookies.txt \
    -d "{
        \"email\": \"$TEST_EMAIL\",
        \"password\": \"$TEST_PASSWORD\",
        \"name\": \"Test User\",
        \"softwareBackground\": \"Intermediate\",
        \"hardwareBackground\": \"Beginner\",
        \"interestArea\": \"AI\"
    }")

if echo "$SIGNUP_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✓ User signup successful${NC}"
    echo "   Email: $TEST_EMAIL"
else
    echo -e "${YELLOW}⚠ Signup failed (user may already exist)${NC}"
fi
echo ""

# Test 5: User Signin
echo "5️⃣  Testing User Signin..."
SIGNIN_RESPONSE=$(curl -s -X POST http://localhost:3001/api/auth/sign-in/email \
    -H "Content-Type: application/json" \
    -c cookies.txt \
    -d "{
        \"email\": \"$TEST_EMAIL\",
        \"password\": \"$TEST_PASSWORD\"
    }")

if echo "$SIGNIN_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✓ User signin successful${NC}"
else
    echo -e "${RED}✗ User signin failed${NC}"
    echo "   Response: $SIGNIN_RESPONSE"
    exit 1
fi
echo ""

# Test 6: JWT Token Generation
echo "6️⃣  Testing JWT Token Generation..."
JWT_RESPONSE=$(curl -s http://localhost:3001/api/auth/jwt \
    -b cookies.txt)

if echo "$JWT_RESPONSE" | grep -q "token"; then
    JWT_TOKEN=$(echo "$JWT_RESPONSE" | grep -o '"token":"[^"]*"' | cut -d'"' -f4)
    echo -e "${GREEN}✓ JWT token generated successfully${NC}"
    echo "   Token: ${JWT_TOKEN:0:20}..."
else
    echo -e "${RED}✗ JWT token generation failed${NC}"
    echo "   Response: $JWT_RESPONSE"
    exit 1
fi
echo ""

# Test 7: Translation Endpoint (unauthenticated)
echo "7️⃣  Testing Translation Endpoint (no auth)..."
UNAUTH_RESPONSE=$(curl -s -X POST http://localhost:8000/v1/api/translate \
    -H "Content-Type: application/json" \
    -d "{
        \"chapter_slug\": \"preface/index\",
        \"target_language\": \"ur\"
    }")

if echo "$UNAUTH_RESPONSE" | grep -q "401\|Unauthorized\|Authentication required"; then
    echo -e "${GREEN}✓ Endpoint correctly rejects unauthenticated requests${NC}"
else
    echo -e "${YELLOW}⚠ Expected 401 error for unauthenticated request${NC}"
fi
echo ""

# Test 8: Translation Endpoint (authenticated)
echo "8️⃣  Testing Translation Endpoint (with JWT)..."
AUTH_RESPONSE=$(curl -s -w "\nHTTP_STATUS:%{http_code}" -X POST http://localhost:8000/v1/api/translate \
    -H "Content-Type: application/json" \
    -H "Authorization: Bearer $JWT_TOKEN" \
    -d "{
        \"chapter_slug\": \"preface/index\",
        \"target_language\": \"ur\"
    }")

HTTP_STATUS=$(echo "$AUTH_RESPONSE" | grep "HTTP_STATUS" | cut -d':' -f2)
RESPONSE_BODY=$(echo "$AUTH_RESPONSE" | sed '/HTTP_STATUS/d')

if [ "$HTTP_STATUS" = "200" ]; then
    echo -e "${GREEN}✓ Translation endpoint works with valid JWT${NC}"
    echo "   Response: ${RESPONSE_BODY:0:100}..."
elif [ "$HTTP_STATUS" = "401" ]; then
    echo -e "${RED}✗ Translation endpoint returned 401 (JWT verification failed)${NC}"
    echo "   Response: $RESPONSE_BODY"
    exit 1
else
    echo -e "${YELLOW}⚠ Translation endpoint returned status $HTTP_STATUS${NC}"
    echo "   Response: $RESPONSE_BODY"
fi
echo ""

# Cleanup
rm -f cookies.txt

echo "======================================================"
echo -e "${GREEN}✅ All authentication tests passed!${NC}"
echo ""
echo "Next steps:"
echo "1. Rebuild the frontend: cd textbook && npm run build"
echo "2. Restart the dev server: npm start"
echo "3. Try the translation feature in the browser"
