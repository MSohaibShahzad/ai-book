/**
 * TypeScript types for Better-Auth user and session data
 */

export interface User {
  id: string;
  name: string;
  email: string;
  emailVerified: boolean;
  image?: string;
  createdAt: Date;
  updatedAt: Date;
  // Custom background fields
  softwareBackground: "Beginner" | "Intermediate" | "Advanced" | "Expert";
  hardwareBackground: "None" | "Beginner" | "Intermediate" | "Advanced";
  interestArea: "AI" | "Robotics" | "Computer Vision" | "Motion Control" | "General";
}

export interface Session {
  id: string;
  userId: string;
  expiresAt: Date;
  token: string;
  ipAddress?: string;
  userAgent?: string;
  user?: User;
}

export interface SignupRequest {
  email: string;
  password: string;
  name: string;
  // Optional background fields
  softwareBackground?: User["softwareBackground"];
  hardwareBackground?: User["hardwareBackground"];
  interestArea?: User["interestArea"];
}

export interface SigninRequest {
  email: string;
  password: string;
  rememberMe?: boolean;
}

export interface ProfileUpdateRequest {
  softwareBackground?: User["softwareBackground"];
  hardwareBackground?: User["hardwareBackground"];
  interestArea?: User["interestArea"];
}

export interface AuthError {
  error: string;
  code: string;
  details?: Record<string, any>;
}

export interface AuthResponse {
  user: User;
  session: Session;
}
