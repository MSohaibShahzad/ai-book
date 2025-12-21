import jwt from "jsonwebtoken";
import type { AuthUser } from "./config";

const JWT_SECRET = process.env.JWT_SECRET || process.env.BETTER_AUTH_SECRET!;
const JWT_EXPIRY = "7d"; // 7 days

export interface JWTPayload {
  userId: string;
  email: string;
  name: string;
  emailVerified: boolean;
  softwareBackground?: string;
  hardwareBackground?: string;
  interestArea?: string;
  iat?: number;
  exp?: number;
}

/**
 * Generate JWT token from user data
 */
export function generateJWT(user: AuthUser): string {
  const payload: JWTPayload = {
    userId: user.id,
    email: user.email,
    name: user.name,
    emailVerified: user.emailVerified,
    softwareBackground: (user as any).softwareBackground,
    hardwareBackground: (user as any).hardwareBackground,
    interestArea: (user as any).interestArea,
  };

  return jwt.sign(payload, JWT_SECRET, {
    expiresIn: JWT_EXPIRY,
    issuer: "auth-service",
    audience: "api-service",
  });
}

/**
 * Verify and decode JWT token
 */
export function verifyJWT(token: string): JWTPayload {
  try {
    const decoded = jwt.verify(token, JWT_SECRET, {
      issuer: "auth-service",
      audience: "api-service",
    }) as JWTPayload;
    return decoded;
  } catch (error) {
    throw new Error(`Invalid JWT: ${error.message}`);
  }
}

/**
 * Get JWT public key for verification by other services
 * (For asymmetric JWT, not used in shared secret approach)
 */
export function getJWTPublicKey(): string {
  // In production, use asymmetric keys (RS256)
  // For now, return instruction to use shared secret
  return JWT_SECRET;
}
