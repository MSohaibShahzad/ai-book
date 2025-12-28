# Quickstart: Authentication with Better-Auth

**Feature**: 003-auth-signup-signin
**Audience**: Developers implementing the authentication system
**Time to Complete**: 30-45 minutes

## Prerequisites

Before starting, ensure you have:

- ✅ Node.js 18+ installed
- ✅ Python 3.11+ installed
- ✅ PostgreSQL database running (Neon or local)
- ✅ Backend FastAPI server set up (from 002-rag-chatbot)
- ✅ Frontend Docusaurus site running (textbook)
- ✅ Environment variables configured

## Installation Steps

### Step 1: Install Better-Auth Dependencies

#### Backend (Python/FastAPI)
```bash
cd backend
pip install better-auth-python  # If Python wrapper exists
# OR use Node.js Better-Auth server mounted alongside FastAPI
```

#### Frontend (React/Docusaurus)
```bash
cd textbook
npm install better-auth
npm install @better-auth/react  # React hooks
```

### Step 2: Configure Environment Variables

Create/update `.env` files:

#### Backend `.env`
```bash
# Database
DATABASE_URL=postgresql://user:pass@host:5432/dbname

# Better-Auth
BETTER_AUTH_SECRET=<generate-with-openssl-rand-hex-32>
BETTER_AUTH_URL=http://localhost:3000
NODE_ENV=development

# Existing RAG configs...
```

#### Frontend `.env`
```bash
# Authentication API
REACT_APP_AUTH_API_URL=http://localhost:3000/api/auth

# Existing Docusaurus configs...
```

**Generate Secret**:
```bash
openssl rand -hex 32
```

### Step 3: Initialize Better-Auth Backend

Create `backend/src/auth/config.ts`:

```typescript
import { betterAuth } from "better-auth";
import { Pool } from "pg";

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
  database: {
    provider: "pg",
    pool,
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update every 24 hours
  },
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        defaultValue: "Beginner",
        input: true,
      },
      hardwareBackground: {
        type: "string",
        required: false,
        defaultValue: "None",
        input: true,
      },
      interestArea: {
        type: "string",
        required: false,
        defaultValue: "AI",
        input: true,
      },
    },
  },
  secret: process.env.BETTER_AUTH_SECRET!,
  baseURL: process.env.BETTER_AUTH_URL!,
});
```

### Step 4: Generate and Apply Database Migrations

```bash
cd backend

# Generate Drizzle schema from Better-Auth config
npx better-auth generate

# Review generated schema in drizzle/schema.ts

# Apply migrations
npx drizzle-kit push

# Verify tables created
psql $DATABASE_URL -c "\dt"
# Should see: user, session, account, verification tables
```

### Step 5: Mount Better-Auth API in Backend

Create `backend/src/auth/routes.ts`:

```typescript
import { auth } from "./config";
import { FastifyInstance } from "fastify";

export async function registerAuthRoutes(app: FastifyInstance) {
  // Mount Better-Auth handler at /api/auth/*
  app.all("/api/auth/*", async (request, reply) => {
    return auth.handler(request.raw, reply.raw);
  });
}
```

Update `backend/src/main.ts`:
```typescript
import { registerAuthRoutes } from "./auth/routes";

// After app initialization
await registerAuthRoutes(app);
```

### Step 6: Initialize Better-Auth Client (Frontend)

Create `textbook/src/lib/auth-client.ts`:

```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_API_URL || "http://localhost:3000/api/auth",
});

export const {
  signUp,
  signIn,
  signOut,
  useSession,
} = authClient;
```

### Step 7: Create Signup Form Component

Create `textbook/src/components/SignupForm.tsx`:

```tsx
import React, { useState } from 'react';
import { authClient } from '../lib/auth-client';

export function SignupForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('Beginner');
  const [hardwareBackground, setHardwareBackground] = useState('None');
  const [interestArea, setInterestArea] = useState('AI');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const { data, error: authError } = await authClient.signUp.email({
        email,
        password,
        name: email.split('@')[0], // Default name
        softwareBackground,
        hardwareBackground,
        interestArea,
      });

      if (authError) {
        setError(authError.message);
      } else {
        // Redirect to dashboard or home
        window.location.href = '/';
      }
    } catch (err) {
      setError('An unexpected error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <div>
        <label>Email</label>
        <input
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
      </div>

      <div>
        <label>Password</label>
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          minLength={8}
          required
        />
      </div>

      <div>
        <label>Software Background</label>
        <select
          value={softwareBackground}
          onChange={(e) => setSoftwareBackground(e.target.value)}
        >
          <option value="Beginner">Beginner</option>
          <option value="Intermediate">Intermediate</option>
          <option value="Advanced">Advanced</option>
        </select>
      </div>

      <div>
        <label>Hardware Background</label>
        <select
          value={hardwareBackground}
          onChange={(e) => setHardwareBackground(e.target.value)}
        >
          <option value="None">None</option>
          <option value="Basic">Basic</option>
          <option value="Hands-on">Hands-on</option>
        </select>
      </div>

      <div>
        <label>Interest Area</label>
        <select
          value={interestArea}
          onChange={(e) => setInterestArea(e.target.value)}
        >
          <option value="AI">AI</option>
          <option value="Robotics">Robotics</option>
          <option value="Simulation">Simulation</option>
          <option value="Humanoids">Humanoids</option>
        </select>
      </div>

      {error && <div className="error">{error}</div>}

      <button type="submit" disabled={loading}>
        {loading ? 'Creating account...' : 'Sign Up'}
      </button>
    </form>
  );
}
```

### Step 8: Create Signin Form Component

Create `textbook/src/components/SigninForm.tsx`:

```tsx
import React, { useState } from 'react';
import { authClient } from '../lib/auth-client';

export function SigninForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [rememberMe, setRememberMe] = useState(true);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const { data, error: authError } = await authClient.signIn.email({
        email,
        password,
        rememberMe,
      });

      if (authError) {
        setError('Invalid email or password. Please try again.');
      } else {
        window.location.href = '/';
      }
    } catch (err) {
      setError('An unexpected error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <div>
        <label>Email</label>
        <input
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
      </div>

      <div>
        <label>Password</label>
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
      </div>

      <div>
        <label>
          <input
            type="checkbox"
            checked={rememberMe}
            onChange={(e) => setRememberMe(e.target.checked)}
          />
          Remember me
        </label>
      </div>

      {error && <div className="error">{error}</div>}

      <button type="submit" disabled={loading}>
        {loading ? 'Signing in...' : 'Sign In'}
      </button>
    </form>
  );
}
```

### Step 9: Add Session Management

Create `textbook/src/components/UserMenu.tsx`:

```tsx
import React from 'react';
import { useSession } from '../lib/auth-client';
import { authClient } from '../lib/auth-client';

export function UserMenu() {
  const { data: session, isPending } = useSession();

  const handleSignOut = async () => {
    await authClient.signOut();
    window.location.href = '/';
  };

  if (isPending) {
    return <div>Loading...</div>;
  }

  if (!session) {
    return (
      <div>
        <a href="/signup">Sign Up</a>
        <a href="/signin">Sign In</a>
      </div>
    );
  }

  return (
    <div>
      <span>Welcome, {session.user.name}!</span>
      <a href="/profile">Profile</a>
      <button onClick={handleSignOut}>Sign Out</button>
    </div>
  );
}
```

### Step 10: Test the Integration

#### Start Development Servers

```bash
# Terminal 1: Backend
cd backend
source venv/bin/activate
uvicorn src.main:app --reload --port 8000

# Terminal 2: Frontend
cd textbook
npm start
```

#### Test Signup Flow

1. Navigate to `http://localhost:3000/signup`
2. Fill in email, password, and background selections
3. Submit form
4. Verify:
   - User created in database: `psql $DATABASE_URL -c "SELECT * FROM user;"`
   - Session cookie set in browser DevTools → Application → Cookies
   - Redirected to homepage with authenticated session

#### Test Signin Flow

1. Navigate to `http://localhost:3000/signin`
2. Enter credentials from signup
3. Verify:
   - Session created
   - User redirected to homepage
   - UserMenu shows "Welcome, [name]!"

#### Test Session Persistence

1. Close browser completely
2. Reopen and navigate to `http://localhost:3000`
3. Verify user still signed in (session persisted)

#### Test Chatbot Integration

1. Sign in with user account
2. Open chatbot widget
3. Ask a question
4. Backend should receive user metadata:
   ```python
   # In chatbot endpoint
   user = session.user
   print(f"Software: {user.softwareBackground}")
   print(f"Hardware: {user.hardwareBackground}")
   print(f"Interest: {user.interestArea}")
   ```

## Common Issues and Troubleshooting

### Issue: "Database connection failed"

**Solution**: Verify `DATABASE_URL` is correct and database is running:
```bash
psql $DATABASE_URL -c "SELECT 1;"
```

### Issue: "Session cookie not set"

**Solution**: Check CORS configuration in backend:
```python
# FastAPI CORS middleware
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,  # Required!
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Issue: "Custom fields not appearing in user object"

**Solution**: Ensure migrations applied and `input: true` set in `additionalFields`:
```bash
npx drizzle-kit push  # Re-apply migrations
```

### Issue: "Password too weak" error

**Solution**: Ensure password is at least 8 characters (configured in `minPasswordLength`)

## Next Steps

After completing quickstart:

1. ✅ **Implement Profile Update** (User Story 3):
   - Create profile settings page
   - Add update form for background fields
   - Wire up to `/user/profile` PATCH endpoint

2. ✅ **Add Validation**:
   - Frontend: Real-time password strength indicator
   - Backend: Additional email format validation

3. ✅ **Integrate with Chatbot**:
   - Modify RAG backend to accept user metadata
   - Update LLM prompts to use background context
   - Test personalized responses

4. ✅ **Add Error Handling**:
   - Network failure recovery
   - Duplicate email handling
   - Session expiration handling

5. ✅ **Production Deployment**:
   - Update `BETTER_AUTH_URL` to production domain
   - Enable HTTPS for secure cookies
   - Configure production database

## Resources

- [Better-Auth Documentation](https://better-auth.com/docs)
- [Better-Auth GitHub](https://github.com/better-auth/better-auth)
- [Drizzle ORM Docs](https://orm.drizzle.team)
- [Feature Spec](./spec.md)
- [Data Model](./data-model.md)
- [API Contracts](./contracts/auth-api.openapi.yaml)

## Support

For issues or questions:
- Check [research.md](./research.md) for architecture decisions
- Review [plan.md](./plan.md) for implementation strategy
- Consult Context7 MCP for Better-Auth documentation
