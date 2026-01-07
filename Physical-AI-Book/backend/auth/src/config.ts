import { betterAuth } from "better-auth";
import { Pool } from "pg";
import * as dotenv from "dotenv";

// Load environment variables
dotenv.config();

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

// Better-Auth configuration with JWT support
export const auth = betterAuth({
  database: pool,
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Automatically sign in after signup
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },
  // Enable cross-domain cookies (Railway + Vercel)
  advanced: {
    defaultCookieAttributes: {
      sameSite: "none", // Required for cross-domain
      secure: true,     // Required with sameSite=none
      partitioned: true // New browser standard for cross-domain cookies
    }
  },
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        defaultValue: "Beginner",
        input: true, // Allow user input during signup
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
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
  trustedOrigins: process.env.TRUSTED_ORIGINS
    ? process.env.TRUSTED_ORIGINS.split(',').map(origin => origin.trim())
    : [
        "http://localhost:3000", // Docusaurus dev server
        "http://localhost:8000", // FastAPI backend
      ],
});

// Export types for use in other modules
export type AuthUser = typeof auth.$Infer.Session.user;
export type AuthSession = typeof auth.$Infer.Session.session;
