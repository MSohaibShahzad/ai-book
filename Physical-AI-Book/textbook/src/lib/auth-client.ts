/**
 * Better-Auth client for frontend authentication
 *
 * Provides React hooks and client methods for:
 * - User signup with background information
 * - User signin with remember me
 * - Session management
 * - User profile updates
 */
import { createAuthClient } from "better-auth/react";

// Auth API URL - points to Better-Auth server
// In production, update this to your auth server domain
const AUTH_API_URL = "http://localhost:3001";

/**
 * Better-Auth client instance
 * Configured to connect to the auth server at port 3001
 */
export const authClient = createAuthClient({
  baseURL: AUTH_API_URL,
  // Enable credentials for httpOnly cookies
  credentials: "include",
});

/**
 * Export authentication hooks for use in React components
 */
export const {
  useSession,
  signIn,
  signOut,
  signUp,
} = authClient;

/**
 * Custom types for signup with background fields
 */
export interface SignupData {
  email: string;
  password: string;
  name: string;
  softwareBackground?: "Beginner" | "Intermediate" | "Advanced" | "Expert";
  hardwareBackground?: "None" | "Beginner" | "Intermediate" | "Advanced";
  interestArea?: "AI" | "Robotics" | "Computer Vision" | "Motion Control" | "General";
}

/**
 * Custom types for signin
 */
export interface SigninData {
  email: string;
  password: string;
  rememberMe?: boolean;
}

/**
 * Custom types for profile updates
 */
export interface ProfileUpdateData {
  softwareBackground?: "Beginner" | "Intermediate" | "Advanced" | "Expert";
  hardwareBackground?: "None" | "Beginner" | "Intermediate" | "Advanced";
  interestArea?: "AI" | "Robotics" | "Computer Vision" | "Motion Control" | "General";
}

/**
 * Helper function to sign up with custom background fields
 */
export async function signUpWithBackground(data: SignupData) {
  return signUp.email({
    email: data.email,
    password: data.password,
    name: data.name,
    // Pass custom fields through callbackURL or additional data
    // Better-Auth will handle these via the additionalFields config
    data: {
      softwareBackground: data.softwareBackground || "Beginner",
      hardwareBackground: data.hardwareBackground || "None",
      interestArea: data.interestArea || "AI",
    },
  });
}

/**
 * Helper function to sign in with remember me option
 */
export async function signInWithEmail(data: SigninData) {
  return signIn.email({
    email: data.email,
    password: data.password,
    // Remember me extends session duration
    rememberMe: data.rememberMe,
  });
}

/**
 * Helper function to update user profile
 */
export async function updateProfile(data: ProfileUpdateData) {
  // Use the Better-Auth client to update user data
  const response = await fetch(`${AUTH_API_URL}/api/auth/user/profile`, {
    method: "PATCH",
    headers: {
      "Content-Type": "application/json",
    },
    credentials: "include",
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.message || "Failed to update profile");
  }

  return response.json();
}

export default authClient;
