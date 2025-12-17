# Better Auth Integration Skill

## Name
better-auth

## Description
A comprehensive skill for integrating Better Auth (https://www.better-auth.com/) authentication into applications. This skill provides step-by-step guidance for implementing secure, production-ready authentication using the Better Auth framework with support for email/password, social providers, and session management.

## Use Cases
- Implementing user authentication in web applications
- Setting up sign-up/sign-in flows
- Adding session management and protected routes
- Integrating Better Auth with existing backends (FastAPI, Express, Next.js)
- Configuring OAuth providers (Google, GitHub, etc.)
- Implementing email verification and password reset
- Setting up two-factor authentication

## Prerequisites
- Node.js 18+ installed
- Database (PostgreSQL, MySQL, or SQLite)
- Basic understanding of TypeScript/JavaScript
- Framework-specific setup (React, Vue, Svelte, or vanilla JS)

## Framework Support
- **Frontend**: React, Vue, Svelte, Solid, Vanilla JS
- **Backend**: Node.js/Express, Next.js, Remix, SvelteKit, Solid Start
- **Database**: PostgreSQL, MySQL, SQLite, MongoDB (via adapters)

## Core Capabilities

### 1. Better Auth Server Setup
- Initialize Better Auth instance with database adapter
- Configure email/password authentication
- Set up session management with cookies
- Configure CORS and security settings
- Generate secure secrets and environment variables

### 2. Frontend Integration
- Create auth client for framework (React, Vue, etc.)
- Implement sign-in and sign-up components
- Set up protected routes with authentication guards
- Manage session state with `useSession()` hook
- Handle authentication errors and loading states

### 3. Database Schema
- User table (id, name, email, email_verified, image)
- Session table (id, token, expires_at, user_id, ip_address)
- Account table (id, provider_id, user_id, password, access_token)
- Verification table (id, identifier, value, expires_at)

### 4. Security Features
- Bcrypt password hashing
- Secure session token generation
- CSRF protection
- Rate limiting
- IP address and user agent tracking
- Email verification
- Password reset flows

### 5. Advanced Features
- OAuth social providers (Google, GitHub, Discord, etc.)
- Two-factor authentication (2FA)
- Magic link authentication
- Passkey/WebAuthn support
- Multi-tenancy
- Organization/team support
- Role-based access control (RBAC)

## Implementation Pattern

### Architecture: Backend-for-Frontend (BFF) Pattern

```
┌─────────────────┐
│   Frontend      │
│   (React/Vue)   │
└────────┬────────┘
         │
    ┌────┴─────┐
    │          │
    ▼          ▼
┌─────────┐  ┌────────────┐
│  Auth   │  │   API      │
│ Server  │  │  Server    │
│(Node.js)│  │ (Any tech) │
└────┬────┘  └─────┬──────┘
     │             │
     └──────┬──────┘
            ▼
      ┌─────────┐
      │Database │
      └─────────┘
```

## Standard File Structure

```
project/
├── auth-server/              # Better Auth Node.js server
│   ├── src/
│   │   ├── auth.ts          # Better Auth config
│   │   └── index.ts         # Express/Server entry
│   ├── prisma/
│   │   └── schema.prisma    # Database schema
│   ├── package.json
│   ├── tsconfig.json
│   └── .env
│
├── src/                      # Frontend application
│   ├── lib/
│   │   └── auth-client.js   # Better Auth client
│   ├── components/
│   │   └── Auth/
│   │       ├── SignInPage.js
│   │       ├── SignUpPage.js
│   │       ├── ProtectedRoute.js
│   │       └── UserButton.js
│   └── pages/
│       └── auth/
│           ├── signin.js
│           └── signup.js
```

## Configuration Templates

### Better Auth Server (auth.ts)

```typescript
import { betterAuth } from "better-auth";
import { prismaAdapter } from "better-auth/adapters/prisma";
import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "postgresql"
  }),
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false
  },
  session: {
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60 // 5 minutes
    }
  },
  secret: process.env.BETTER_AUTH_SECRET,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001"
});
```

### React Client (auth-client.js)

```javascript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_URL || "http://localhost:3001"
});

export const { signIn, signUp, signOut, useSession } = authClient;
```

### Protected Route Component

```javascript
import React from "react";
import { useSession } from "../lib/auth-client";
import { Redirect } from "react-router-dom";

export default function ProtectedRoute({ children }) {
  const { data: session, isPending } = useSession();

  if (isPending) return <div>Loading...</div>;
  if (!session) return <Redirect to="/auth/signin" />;

  return <>{children}</>;
}
```

## Environment Variables

### Auth Server
```env
DATABASE_URL=postgresql://user:pass@host/db
BETTER_AUTH_SECRET=your-secure-secret-min-32-chars
BETTER_AUTH_URL=http://localhost:3001
PORT=3001
```

### Frontend
```env
REACT_APP_AUTH_URL=http://localhost:3001
```

## Common Integration Scenarios

### Scenario 1: React + FastAPI Backend
1. Create Node.js auth server with Better Auth
2. Use Better Auth React client in frontend
3. Validate JWT tokens in FastAPI using PyJWT
4. Share same database (PostgreSQL/MySQL)

### Scenario 2: Next.js Full-Stack
1. Use Better Auth with Next.js API routes
2. Leverage server-side session validation
3. Use `useSession()` for client-side state

### Scenario 3: Vue + Express Backend
1. Create Express server with Better Auth
2. Use Better Auth Vue client
3. Share session cookies between services

## Validation Checklist

Before considering Better Auth integration complete, verify:

- [ ] Users can sign up with email/password
- [ ] Users can sign in successfully
- [ ] Session persists across page refresh
- [ ] Protected routes redirect unauthenticated users
- [ ] Users can log out and session is cleared
- [ ] Password is securely hashed (never stored plain text)
- [ ] Session tokens are httpOnly cookies (secure in production)
- [ ] CORS is configured properly
- [ ] Environment secrets are secure and not committed to git
- [ ] Database migrations ran successfully
- [ ] User data is isolated (users only see own data)

## Troubleshooting

### Issue: Session cookie not set
**Solution**: Ensure `credentials: 'include'` on fetch requests and CORS allows credentials

### Issue: JWT validation fails in backend
**Solution**: Verify `BETTER_AUTH_SECRET` matches `JWT_SECRET` exactly

### Issue: "Invalid signature" error
**Solution**: Check that token hasn't expired and secret is correct

### Issue: CORS errors
**Solution**: Add auth server URL to allowed origins, enable credentials

## Documentation Resources

- Better Auth Official Docs: https://www.better-auth.com/docs
- Better Auth React Integration: https://www.better-auth.com/docs/integrations/react
- Better Auth GitHub: https://github.com/better-auth/better-auth
- Context7 Better Auth Docs: `/better-auth/better-auth` (use with MCP)

## Usage Instructions

To invoke this skill in Claude Code, use:
```
Can you help me implement Better Auth for my application?
```

Or for specific tasks:
```
Set up Better Auth with React and PostgreSQL
Add Google OAuth to my Better Auth setup
Create a protected route with Better Auth
```

## Plugin Ecosystem

Better Auth supports plugins for:
- Username authentication
- Magic link (passwordless)
- Passkey/WebAuthn
- Two-factor authentication
- Organization/team management
- API key management
- Admin panel
- Rate limiting
- IP blocking

Refer to Better Auth documentation for plugin-specific setup.

## Version Compatibility

- Better Auth: 1.0.0+
- Node.js: 18.0.0+
- TypeScript: 5.0.0+ (recommended)
- React: 18.0.0+ (for React integration)
- Prisma: 5.0.0+ (for Prisma adapter)

## Notes

- Better Auth is framework-agnostic but requires Node.js runtime for the server
- For Python backends (FastAPI, Django), use BFF pattern with JWT validation
- Always use HTTPS in production for secure cookies
- Rotate secrets periodically for enhanced security
- Consider implementing email verification for production apps
