# Data Model: Authentication with Better-Auth

**Feature**: 003-auth-signup-signin
**Date**: 2025-12-16
**Phase**: Phase 1 - Design

## Overview

This document defines the database schema, entity relationships, and data structures for the authentication system using Better-Auth with custom user metadata.

## Entity Relationship Diagram

```
┌─────────────────────┐
│       User          │
│─────────────────────│
│ id (PK)             │
│ name                │
│ email (UNIQUE)      │
│ emailVerified       │
│ image               │
│ createdAt           │
│ updatedAt           │
│ softwareBackground  │◄─────┐
│ hardwareBackground  │      │
│ interestArea        │      │
└─────────────────────┘      │
                             │
                             │ 1:N
                             │
                      ┌──────┴──────────────┐
                      │     Session         │
                      │─────────────────────│
                      │ id (PK)             │
                      │ userId (FK)         │
                      │ expiresAt           │
                      │ token (UNIQUE)      │
                      │ createdAt           │
                      │ updatedAt           │
                      │ ipAddress           │
                      │ userAgent           │
                      └─────────────────────┘
```

## Core Entities

### User Entity

**Description**: Represents an authenticated user with profile information and learning background metadata.

**Table Name**: `user`

**Fields**:

| Field Name          | Type      | Constraints           | Default Value | Description |
|---------------------|-----------|----------------------|---------------|-------------|
| id                  | TEXT      | PRIMARY KEY          | UUID v4       | Unique user identifier |
| name                | TEXT      | NOT NULL             | -             | User's full name or display name |
| email               | TEXT      | NOT NULL, UNIQUE     | -             | User's email address (login identifier) |
| emailVerified       | BOOLEAN   | NOT NULL             | false         | Whether email has been verified |
| image               | TEXT      | NULLABLE             | NULL          | URL to user's profile picture |
| createdAt           | TIMESTAMP | NOT NULL             | NOW()         | Account creation timestamp |
| updatedAt           | TIMESTAMP | NOT NULL             | NOW()         | Last profile update timestamp |
| softwareBackground  | TEXT      | NOT NULL             | 'Beginner'    | User's programming experience level |
| hardwareBackground  | TEXT      | NOT NULL             | 'None'        | User's robotics hardware experience |
| interestArea        | TEXT      | NOT NULL             | 'AI'          | User's primary learning interest |

**Indexes**:
- `PRIMARY KEY` on `id`
- `UNIQUE INDEX` on `email`

**Validation Rules**:
- `email`: Must be valid email format (RFC 5322)
- `softwareBackground`: Must be one of ['Beginner', 'Intermediate', 'Advanced']
- `hardwareBackground`: Must be one of ['None', 'Basic', 'Hands-on']
- `interestArea`: Must be one of ['AI', 'Robotics', 'Simulation', 'Humanoids']

**Relationships**:
- One-to-many with `Session` (one user can have multiple active sessions)

**State Transitions**:
```
[New User] --signup--> [Unverified] --email-verify--> [Verified]
                           |
                           +--signin--> [Active Session]
```

### Session Entity

**Description**: Represents an active user authentication session with security metadata.

**Table Name**: `session`

**Fields**:

| Field Name  | Type      | Constraints                    | Default Value | Description |
|-------------|-----------|-------------------------------|---------------|-------------|
| id          | TEXT      | PRIMARY KEY                   | UUID v4       | Unique session identifier |
| userId      | TEXT      | NOT NULL, FOREIGN KEY (user.id) ON DELETE CASCADE | - | Reference to user who owns this session |
| expiresAt   | TIMESTAMP | NOT NULL                      | NOW() + 7 days | Session expiration timestamp |
| token       | TEXT      | NOT NULL, UNIQUE              | Generated     | Session token stored in httpOnly cookie |
| createdAt   | TIMESTAMP | NOT NULL                      | NOW()         | Session creation timestamp |
| updatedAt   | TIMESTAMP | NOT NULL                      | NOW()         | Last session activity timestamp |
| ipAddress   | TEXT      | NULLABLE                      | NULL          | IP address of session creation |
| userAgent   | TEXT      | NULLABLE                      | NULL          | Browser user agent string |

**Indexes**:
- `PRIMARY KEY` on `id`
- `UNIQUE INDEX` on `token`
- `INDEX` on `userId` (for efficient user session lookups)
- `INDEX` on `expiresAt` (for cleanup queries)

**Validation Rules**:
- `token`: Must be unique, cryptographically secure random string
- `expiresAt`: Must be in the future at creation time

**Relationships**:
- Many-to-one with `User` (foreign key `userId`)
- CASCADE DELETE: When user is deleted, all sessions are deleted

**State Transitions**:
```
[Created] --use--> [Active] --expire/logout--> [Expired/Invalidated]
```

## Supplementary Entities

### Account Entity (Better-Auth Standard)

**Description**: Stores OAuth provider accounts (future extension for social login).

**Table Name**: `account`

**Fields**:

| Field Name     | Type      | Constraints                    | Description |
|----------------|-----------|-------------------------------|-------------|
| id             | TEXT      | PRIMARY KEY                   | Unique account identifier |
| userId         | TEXT      | NOT NULL, FOREIGN KEY (user.id) | Reference to user |
| accountId      | TEXT      | NOT NULL                      | Provider-specific account ID |
| providerId     | TEXT      | NOT NULL                      | OAuth provider identifier |
| accessToken    | TEXT      | NULLABLE                      | OAuth access token |
| refreshToken   | TEXT      | NULLABLE                      | OAuth refresh token |
| expiresAt      | TIMESTAMP | NULLABLE                      | Token expiration |
| createdAt      | TIMESTAMP | NOT NULL                      | Account link timestamp |

**Note**: This table is created by Better-Auth for future extensibility but not used in the initial email/password implementation.

### Verification Entity (Better-Auth Standard)

**Description**: Stores email verification tokens.

**Table Name**: `verification`

**Fields**:

| Field Name  | Type      | Constraints  | Description |
|-------------|-----------|-------------|-------------|
| id          | TEXT      | PRIMARY KEY | Unique verification ID |
| identifier  | TEXT      | NOT NULL    | Email being verified |
| value       | TEXT      | NOT NULL    | Verification token |
| expiresAt   | TIMESTAMP | NOT NULL    | Token expiration |
| createdAt   | TIMESTAMP | NOT NULL    | Token creation time |

**Note**: Used for email verification flow (optional feature, not P1).

## Data Types and Enumerations

### SoftwareBackground Enum

**Type**: String enumeration
**Values**: ['Beginner', 'Intermediate', 'Advanced']

**Definitions**:
- **Beginner**: New to programming or basic scripting experience
- **Intermediate**: Comfortable with programming concepts, some framework experience
- **Advanced**: Professional developer, experienced with multiple languages/frameworks

### HardwareBackground Enum

**Type**: String enumeration
**Values**: ['None', 'Basic', 'Hands-on']

**Definitions**:
- **None**: No robotics hardware experience
- **Basic**: Theoretical knowledge or hobbyist exposure
- **Hands-on**: Practical experience with robotics platforms, sensors, actuators

### InterestArea Enum

**Type**: String enumeration
**Values**: ['AI', 'Robotics', 'Simulation', 'Humanoids']

**Definitions**:
- **AI**: Focus on machine learning, neural networks, cognitive systems
- **Robotics**: Focus on control systems, kinematics, path planning
- **Simulation**: Focus on digital twins, physics engines, virtual environments
- **Humanoids**: Focus on bipedal locomotion, anthropomorphic systems

## Data Access Patterns

### Primary Access Patterns

1. **User Signup**:
   ```sql
   INSERT INTO user (id, name, email, softwareBackground, hardwareBackground, interestArea)
   VALUES ($1, $2, $3, $4, $5, $6)
   RETURNING *;
   ```

2. **User Signin (Fetch by Email)**:
   ```sql
   SELECT * FROM user WHERE email = $1;
   ```

3. **Create Session**:
   ```sql
   INSERT INTO session (id, userId, token, expiresAt, ipAddress, userAgent)
   VALUES ($1, $2, $3, NOW() + INTERVAL '7 days', $4, $5)
   RETURNING *;
   ```

4. **Validate Session**:
   ```sql
   SELECT s.*, u.*
   FROM session s
   JOIN user u ON s.userId = u.id
   WHERE s.token = $1 AND s.expiresAt > NOW();
   ```

5. **Update User Background**:
   ```sql
   UPDATE user
   SET softwareBackground = $1, hardwareBackground = $2, interestArea = $3, updatedAt = NOW()
   WHERE id = $4
   RETURNING *;
   ```

6. **Cleanup Expired Sessions**:
   ```sql
   DELETE FROM session WHERE expiresAt < NOW();
   ```

### Query Performance Considerations

- **Email Lookup**: UNIQUE index on `user.email` ensures O(log n) lookup
- **Session Validation**: UNIQUE index on `session.token` + compound check on `expiresAt`
- **User Sessions**: Index on `session.userId` for efficient multi-session queries
- **Expired Session Cleanup**: Index on `session.expiresAt` for batch deletions

## Data Integrity Rules

### Referential Integrity

1. **Session → User**:
   - `session.userId` must reference valid `user.id`
   - CASCADE DELETE: Deleting user removes all sessions

2. **Account → User** (future):
   - `account.userId` must reference valid `user.id`
   - CASCADE DELETE: Deleting user removes linked accounts

### Business Rules

1. **Email Uniqueness**: Each email can only be associated with one user account
2. **Session Token Uniqueness**: Each session must have a unique token
3. **Session Expiration**: Sessions with `expiresAt < NOW()` are considered invalid
4. **Background Defaults**: New users without specified background get default values
5. **Soft Delete**: Users are not deleted by default (implement soft delete if needed)

## Data Migration Strategy

### Initial Schema Creation

**Migration**: `001_create_auth_tables.sql`

```sql
-- Create user table with custom fields
CREATE TABLE "user" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid(),
  name TEXT NOT NULL,
  email TEXT NOT NULL UNIQUE,
  "emailVerified" BOOLEAN NOT NULL DEFAULT false,
  image TEXT,
  "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
  "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
  "softwareBackground" TEXT NOT NULL DEFAULT 'Beginner',
  "hardwareBackground" TEXT NOT NULL DEFAULT 'None',
  "interestArea" TEXT NOT NULL DEFAULT 'AI'
);

-- Create session table
CREATE TABLE "session" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid(),
  "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  "expiresAt" TIMESTAMP NOT NULL,
  token TEXT NOT NULL UNIQUE,
  "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
  "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
  "ipAddress" TEXT,
  "userAgent" TEXT
);

-- Create indexes
CREATE INDEX session_userId_idx ON "session"("userId");
CREATE INDEX session_expiresAt_idx ON "session"("expiresAt");

-- Create account table (for future OAuth)
CREATE TABLE "account" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid(),
  "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  "accountId" TEXT NOT NULL,
  "providerId" TEXT NOT NULL,
  "accessToken" TEXT,
  "refreshToken" TEXT,
  "expiresAt" TIMESTAMP,
  "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Create verification table (for email verification)
CREATE TABLE "verification" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid(),
  identifier TEXT NOT NULL,
  value TEXT NOT NULL,
  "expiresAt" TIMESTAMP NOT NULL,
  "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);
```

### Rollback Strategy

```sql
-- Rollback migration
DROP TABLE IF EXISTS "verification" CASCADE;
DROP TABLE IF EXISTS "account" CASCADE;
DROP TABLE IF EXISTS "session" CASCADE;
DROP TABLE IF EXISTS "user" CASCADE;
```

## Data Privacy and Security

### Sensitive Data Handling

1. **Passwords**: Never stored in database - Better-Auth handles hashing
2. **Session Tokens**: Stored hashed in database, transmitted only in httpOnly cookies
3. **Email Addresses**: Stored in plaintext but protected by database access controls
4. **IP Addresses**: Stored for security audit only, not exposed to frontend
5. **User Metadata**: Non-sensitive, but access controlled via session authentication

### GDPR Considerations

- **Right to Access**: Users can view their profile (email, name, background)
- **Right to Rectification**: Users can update background fields via profile page
- **Right to Erasure**: User deletion removes all associated data (CASCADE DELETE)
- **Data Minimization**: Only essential fields collected (name, email, background)

## TypeScript Type Definitions

### User Type

```typescript
export interface User {
  id: string;
  name: string;
  email: string;
  emailVerified: boolean;
  image: string | null;
  createdAt: Date;
  updatedAt: Date;
  softwareBackground: 'Beginner' | 'Intermediate' | 'Advanced';
  hardwareBackground: 'None' | 'Basic' | 'Hands-on';
  interestArea: 'AI' | 'Robotics' | 'Simulation' | 'Humanoids';
}
```

### Session Type

```typescript
export interface Session {
  id: string;
  userId: string;
  expiresAt: Date;
  token: string;
  createdAt: Date;
  updatedAt: Date;
  ipAddress: string | null;
  userAgent: string | null;
}
```

### Authentication Response Types

```typescript
export interface SignupResponse {
  user: User;
  session: Session;
}

export interface SigninResponse {
  user: User;
  session: Session;
}

export interface UpdateProfileResponse {
  user: User;
}
```

## Summary

This data model provides:
- ✅ Extended user schema with custom background fields
- ✅ Secure session management with expiration tracking
- ✅ Type-safe entity definitions for frontend/backend
- ✅ Efficient query patterns with proper indexing
- ✅ Referential integrity with cascade deletes
- ✅ Future extensibility for OAuth and email verification

**Next**: Generate API contracts in `/contracts/` directory.
