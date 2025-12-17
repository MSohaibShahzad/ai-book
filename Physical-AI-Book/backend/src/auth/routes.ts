import { auth } from "./config";
import { toNodeHandler } from "better-auth/node";

/**
 * Better-Auth route handler for /api/auth/* endpoints
 *
 * This exports a Node.js request handler that can be mounted in:
 * 1. Express.js: app.use("/api/auth/*", authHandler)
 * 2. Standalone HTTP server
 * 3. Serverless function (Vercel, Netlify)
 *
 * Handles all Better-Auth endpoints:
 * - POST /api/auth/sign-up/email
 * - POST /api/auth/sign-in/email
 * - POST /api/auth/sign-out
 * - GET /api/auth/session
 * - PATCH /api/auth/user/profile
 */
export const authHandler = toNodeHandler(auth);

// Export auth instance for middleware use
export { auth };
