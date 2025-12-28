#!/usr/bin/env node
/**
 * Standalone Node.js server for Better-Auth endpoints with JWT support
 *
 * Runs on port 3001 alongside FastAPI backend (port 8000)
 * Handles all /api/auth/* routes for authentication
 *
 * Features:
 * - Email/password signup & signin
 * - Session management (httpOnly cookies)
 * - JWT token generation
 * - Rate limiting
 * - CORS protection
 */
import http from "http";
import { authHandler } from "./routes";
import { getSessionForPython } from "./middleware";
import { generateJWT } from "./jwt";
import { auth } from "./config";
import * as dotenv from "dotenv";

// Load environment variables
dotenv.config();

const PORT = process.env.AUTH_SERVER_PORT || 3001;
const HOST = process.env.AUTH_SERVER_HOST || "0.0.0.0";

// Rate limiting storage (in-memory, use Redis for production)
const rateLimitStore = new Map<string, { count: number; resetTime: number }>();

// Rate limit configuration
const RATE_LIMIT_WINDOW_MS = 15 * 60 * 1000; // 15 minutes
const RATE_LIMIT_MAX_REQUESTS = 5; // 5 attempts per window

/**
 * Check rate limit for an IP address
 */
function checkRateLimit(ip: string, path: string): { allowed: boolean; retryAfter?: number } {
  // Only rate limit auth endpoints
  if (!path.startsWith("/api/auth/sign-in") && !path.startsWith("/api/auth/sign-up")) {
    return { allowed: true };
  }

  const key = `${ip}:${path}`;
  const now = Date.now();
  const record = rateLimitStore.get(key);

  if (!record || now > record.resetTime) {
    // New window or expired, reset
    rateLimitStore.set(key, {
      count: 1,
      resetTime: now + RATE_LIMIT_WINDOW_MS,
    });
    return { allowed: true };
  }

  if (record.count >= RATE_LIMIT_MAX_REQUESTS) {
    // Rate limit exceeded
    const retryAfter = Math.ceil((record.resetTime - now) / 1000);
    return { allowed: false, retryAfter };
  }

  // Increment counter
  record.count++;
  rateLimitStore.set(key, record);
  return { allowed: true };
}

// Clean up expired rate limit records every hour
setInterval(() => {
  const now = Date.now();
  for (const [key, record] of rateLimitStore.entries()) {
    if (now > record.resetTime) {
      rateLimitStore.delete(key);
    }
  }
}, 60 * 60 * 1000);

// Create HTTP server
const server = http.createServer(async (req, res) => {
  // Add CORS headers
  const allowedOrigins = process.env.ALLOWED_ORIGINS
    ? process.env.ALLOWED_ORIGINS.split(',').map(origin => origin.trim())
    : [
        "http://localhost:3000", // Docusaurus dev server
        "http://localhost:8000", // FastAPI backend
      ];

  const origin = req.headers.origin || "";
  const isOriginAllowed = allowedOrigins.includes(origin) || origin.endsWith('.vercel.app');

  if (isOriginAllowed) {
    res.setHeader("Access-Control-Allow-Origin", origin);
    res.setHeader("Access-Control-Allow-Credentials", "true");
    res.setHeader("Access-Control-Allow-Methods", "GET, POST, PATCH, DELETE, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, Cookie, Set-Cookie");
    res.setHeader("Access-Control-Expose-Headers", "Set-Cookie");
  }

  // Handle preflight requests
  if (req.method === "OPTIONS") {
    res.writeHead(204);
    res.end();
    return;
  }

  // Health check endpoint
  if (req.url === "/health") {
    res.writeHead(200, { "Content-Type": "application/json" });
    res.end(JSON.stringify({ status: "healthy", service: "auth-service" }));
    return;
  }

  // JWT endpoint - issue JWT from session
  if (req.url === "/api/auth/jwt" && req.method === "GET") {
    try {
      const sessionData = await auth.api.getSession({
        headers: req.headers,
      });

      if (!sessionData || !sessionData.user) {
        res.writeHead(401, { "Content-Type": "application/json" });
        res.end(JSON.stringify({ error: "Unauthorized - no valid session" }));
        return;
      }

      const token = generateJWT(sessionData.user);

      res.writeHead(200, { "Content-Type": "application/json" });
      res.end(JSON.stringify({
        token,
        user: sessionData.user,
        expiresIn: "7d"
      }));
    } catch (error) {
      console.error("JWT generation error:", error);
      res.writeHead(500, { "Content-Type": "application/json" });
      res.end(JSON.stringify({ error: "Failed to generate JWT" }));
    }
    return;
  }

  // Session validation endpoint for Python backend (backwards compatibility)
  if (req.url === "/api/validate-session") {
    try {
      const cookieHeader = req.headers.cookie || "";
      const sessionData = await getSessionForPython(cookieHeader);
      res.writeHead(200, { "Content-Type": "application/json" });
      res.end(sessionData);
    } catch (error) {
      console.error("Session validation error:", error);
      res.writeHead(500, { "Content-Type": "application/json" });
      res.end(JSON.stringify({ error: "Session validation failed" }));
    }
    return;
  }

  // Route all requests starting with /api/auth to Better-Auth handler
  if (req.url?.startsWith("/api/auth")) {
    try {
      // Apply rate limiting
      const clientIp = req.headers["x-forwarded-for"] || req.socket.remoteAddress || "unknown";
      const ip = Array.isArray(clientIp) ? clientIp[0] : clientIp.split(",")[0];
      const rateLimit = checkRateLimit(ip, req.url);

      if (!rateLimit.allowed) {
        res.writeHead(429, {
          "Content-Type": "application/json",
          "Retry-After": rateLimit.retryAfter?.toString() || "900",
        });
        res.end(
          JSON.stringify({
            error: "Too many requests",
            message: "Too many authentication attempts. Please try again later.",
            retryAfter: rateLimit.retryAfter,
          })
        );
        return;
      }

      await authHandler(req, res);
    } catch (error) {
      console.error("Auth handler error:", error);
      res.writeHead(500, { "Content-Type": "application/json" });
      res.end(JSON.stringify({ error: "Internal server error" }));
    }
    return;
  }

  // 404 for other routes
  res.writeHead(404, { "Content-Type": "application/json" });
  res.end(JSON.stringify({ error: "Not found" }));
});

// Start server
server.listen(Number(PORT), HOST, () => {
  console.log(`🔐 Auth Service running on http://${HOST}:${PORT}`);
  console.log(`   Auth endpoints: http://${HOST}:${PORT}/api/auth/*`);
  console.log(`   JWT endpoint: http://${HOST}:${PORT}/api/auth/jwt`);
  console.log(`   Session validation: http://${HOST}:${PORT}/api/validate-session`);
  console.log(`   Health check: http://${HOST}:${PORT}/health`);
});

// Graceful shutdown
process.on("SIGTERM", () => {
  console.log("SIGTERM signal received: closing HTTP server");
  server.close(() => {
    console.log("HTTP server closed");
    process.exit(0);
  });
});

process.on("SIGINT", () => {
  console.log("SIGINT signal received: closing HTTP server");
  server.close(() => {
    console.log("HTTP server closed");
    process.exit(0);
  });
});
