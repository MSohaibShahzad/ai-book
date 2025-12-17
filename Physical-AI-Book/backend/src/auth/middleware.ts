import { auth } from "./config";
import type { User, Session } from "./types";

/**
 * Session middleware for extracting authenticated user from Better-Auth session
 *
 * This middleware validates the session token from the request cookies
 * and attaches the user data to the request for use in downstream handlers.
 *
 * Usage in Node.js/Express:
 * ```typescript
 * app.get("/api/protected", sessionMiddleware, (req, res) => {
 *   const user = req.user;
 *   // ...
 * });
 * ```
 */

export interface SessionRequest {
  user?: User;
  session?: Session;
}

/**
 * Extract session from request cookies (Node.js/Express)
 */
export async function getSessionFromRequest(
  req: any
): Promise<{ user: User | null; session: Session | null }> {
  try {
    // Get session token from cookies
    const cookies = req.headers.cookie || "";
    const sessionToken = parseCookie(cookies, "better-auth.session_token");

    if (!sessionToken) {
      return { user: null, session: null };
    }

    // Verify session with Better-Auth
    const sessionData = await auth.api.getSession({
      headers: req.headers,
    });

    if (!sessionData) {
      return { user: null, session: null };
    }

    return {
      user: sessionData.user as User,
      session: sessionData.session as Session,
    };
  } catch (error) {
    console.error("Session validation error:", error);
    return { user: null, session: null };
  }
}

/**
 * Parse a specific cookie from the cookie header string
 */
function parseCookie(cookieHeader: string, name: string): string | null {
  const cookies = cookieHeader.split(";").map((c) => c.trim());
  for (const cookie of cookies) {
    const [cookieName, cookieValue] = cookie.split("=");
    if (cookieName === name) {
      return decodeURIComponent(cookieValue);
    }
  }
  return null;
}

/**
 * Express/Node.js middleware function
 */
export async function sessionMiddleware(req: any, res: any, next: any) {
  const { user, session } = await getSessionFromRequest(req);
  req.user = user;
  req.session = session;
  next();
}

/**
 * Python FastAPI integration helper
 *
 * Export user data as JSON for FastAPI to consume
 */
export async function getSessionForPython(
  cookieHeader: string
): Promise<string> {
  try {
    const sessionToken = parseCookie(cookieHeader, "better-auth.session_token");

    if (!sessionToken) {
      return JSON.stringify({ user: null, session: null });
    }

    // Create a minimal request object for Better-Auth
    const mockRequest = {
      headers: {
        cookie: cookieHeader,
      },
    };

    const sessionData = await auth.api.getSession({
      headers: mockRequest.headers,
    });

    if (!sessionData) {
      return JSON.stringify({ user: null, session: null });
    }

    return JSON.stringify({
      user: sessionData.user,
      session: sessionData.session,
    });
  } catch (error) {
    console.error("Session validation error:", error);
    return JSON.stringify({ user: null, session: null, error: error.message });
  }
}
