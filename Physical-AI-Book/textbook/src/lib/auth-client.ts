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
import { inferAdditionalFields } from "better-auth/client/plugins";

// Auth API URL - points to Better-Auth server
// Reads from Docusaurus customFields (configured in docusaurus.config.js)
const getAuthApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:3001';

  // In development, use local Better-Auth server
  if (window.location.hostname === 'localhost') {
    return 'http://localhost:3001';
  }

  // In production, use the Better-Auth microservice
  return 'https://ai-book-auth.vercel.app';
};

const AUTH_API_URL = getAuthApiUrl();

/**
 * Better-Auth client instance
 * Configured to connect to the FastAPI backend
 * Auth requests are proxied through FastAPI to the Better-Auth microservice
 * Uses inferAdditionalFields plugin to infer custom user fields from server
 */
export const authClient = createAuthClient({
  baseURL: AUTH_API_URL,
  // Enable credentials for httpOnly cookies
  credentials: "include",
  plugins: [
    inferAdditionalFields({
      user: {
        softwareBackground: {
          type: "string",
        },
        hardwareBackground: {
          type: "string",
        },
        interestArea: {
          type: "string",
        },
      },
    }),
  ],
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
  const response = await signUp.email({
    email: data.email,
    password: data.password,
    name: data.name,
    fetchOptions: {
      body: {
        softwareBackground: data.softwareBackground || "Beginner",
        hardwareBackground: data.hardwareBackground || "None",
        interestArea: data.interestArea || "AI",
      },
    },
  }) as any;

  // Check if signup was successful
  // Better-Auth returns { error: ... } on failure
  if (response && response.error) {
    // Extract error message from Better-Auth response
    const errorMessage = typeof response.error === 'string'
      ? response.error
      : response.error?.message || 'Signup failed';

    throw new Error(errorMessage);
  }

  // Verify that signup was successful
  if (!response || (response.data && !response.data.session)) {
    throw new Error('Signup failed. Please try again.');
  }

  return response;
}

/**
 * Helper function to sign in with remember me option
 */
export async function signInWithEmail(data: SigninData) {
  const response = await signIn.email({
    email: data.email,
    password: data.password,
    // Remember me extends session duration
    rememberMe: data.rememberMe,
  }) as any;

  // Check if signin was successful
  // Better-Auth returns { error: ... } on failure
  if (response && response.error) {
    // Extract error message from Better-Auth response
    const errorMessage = typeof response.error === 'string'
      ? response.error
      : response.error?.message || 'Invalid credentials';

    throw new Error(errorMessage);
  }

  // Verify that we actually have a session after signin
  if (!response || (response.data && !response.data.session)) {
    throw new Error('Invalid email or password');
  }

  return response;
}

/**
 * Helper function to update user profile
 */
export async function updateProfile(data: ProfileUpdateData) {
  // Use the Better-Auth client to update user data
  const response = await fetch(`${AUTH_API_URL}/api/auth/user/update`, {
    method: "POST",
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

/**
 * Get JWT token from auth service using the current session cookie
 * This token is required for authenticated API requests to the backend
 */
export async function getJWTToken(): Promise<string | null> {
  try {
    const response = await fetch(`${AUTH_API_URL}/api/auth/jwt`, {
      method: 'GET',
      credentials: 'include', // Include session cookie
    });

    if (!response.ok) {
      console.error('[Auth] Failed to get JWT token:', response.status);
      return null;
    }

    const data = await response.json();
    return data.token || null;
  } catch (error) {
    console.error('[Auth] Error getting JWT token:', error);
    return null;
  }
}

export default authClient;
