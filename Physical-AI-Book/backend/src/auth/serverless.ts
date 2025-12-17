/**
 * Serverless handler for Better-Auth on Vercel
 *
 * This file adapts the Better-Auth server for Vercel's serverless environment.
 * Routes all /api/auth/* requests to Better-Auth.
 */
import type { VercelRequest, VercelResponse } from '@vercel/node';
import { authHandler } from './routes';
import * as dotenv from 'dotenv';

// Load environment variables
dotenv.config();

// CORS configuration
const allowedOrigins = [
  'http://localhost:3000',
  'https://ai-book-green.vercel.app',
  'https://ai-book-ki61.vercel.app',
  // Add your production domains here
];

/**
 * Set CORS headers
 */
function setCorsHeaders(req: VercelRequest, res: VercelResponse) {
  const origin = req.headers.origin || '';

  if (allowedOrigins.includes(origin)) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PATCH, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, Cookie');
  }
}

/**
 * Main serverless handler
 */
export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Set CORS headers
  setCorsHeaders(req, res);

  // Handle preflight requests
  if (req.method === 'OPTIONS') {
    res.status(204).end();
    return;
  }

  const url = req.url || '';

  // Health check endpoint
  if (url === '/health') {
    res.status(200).json({
      status: 'ok',
      service: 'better-auth-server',
      timestamp: new Date().toISOString()
    });
    return;
  }

  // Session validation endpoint for Python backend
  if (url === '/api/validate-session') {
    try {
      const { getSessionForPython } = await import('./middleware');
      const cookieHeader = req.headers.cookie || '';
      const sessionData = await getSessionForPython(cookieHeader);

      // Parse the JSON string returned by getSessionForPython
      const parsedData = JSON.parse(sessionData);
      res.status(200).json(parsedData);
    } catch (error) {
      console.error('Session validation error:', error);
      res.status(500).json({ error: 'Session validation failed' });
    }
    return;
  }

  // Route all /api/auth/* requests to Better-Auth
  if (url.startsWith('/api/auth')) {
    try {
      // Convert Vercel request/response to Node.js IncomingMessage/ServerResponse
      await authHandler(req as any, res as any);
    } catch (error) {
      console.error('Auth handler error:', error);
      res.status(500).json({ error: 'Internal server error' });
    }
    return;
  }

  // 404 for other routes
  res.status(404).json({ error: 'Not found' });
}
