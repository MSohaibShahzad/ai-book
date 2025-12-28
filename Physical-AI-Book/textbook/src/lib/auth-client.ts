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

  // If no error was returned, signup was successful
  // Better-Auth sets the session cookie automatically
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

  console.log('Sign-in response:', response);

  // Check if signin was successful
  // Better-Auth returns { error: ... } on failure
  if (response && response.error) {
    // Extract error message from Better-Auth response
    const errorMessage = typeof response.error === 'string'
      ? response.error
      : response.error?.message || 'Invalid credentials';

    throw new Error(errorMessage);
  }

  // If there's no error, sign-in was successful
  // Better-Auth sets the session cookie automatically
  return response;
}

/**
 * Helper function to update user profile
 */
export async function updateProfile(data: ProfileUpdateData) {
  // Use the Better-Auth client's update-user endpoint
  const response = await fetch(`${AUTH_API_URL}/api/auth/update-user`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    credentials: "include",
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const errorText = await response.text();
    let errorMessage = "Failed to update profile";
    try {
      const error = JSON.parse(errorText);
      errorMessage = error.message || errorMessage;
    } catch {
      // If response is not JSON, use the text as error message
      errorMessage = errorText || errorMessage;
    }
    throw new Error(errorMessage);
  }

  return response.json();
}

/**
 * Get JWT token from auth service using the current session cookie
 * This token is required for authenticated API requests to the backend
 */
export async function getJWTToken(): Promise<string | null> {
  try {
    console.log('[Auth] Fetching JWT token from:', `${AUTH_API_URL}/api/auth/jwt`);
    console.log('[Auth] Document cookies:', document.cookie ? 'present' : 'none');

    const response = await fetch(`${AUTH_API_URL}/api/auth/jwt`, {
      method: 'GET',
      credentials: 'include', // Include session cookie
    });

    console.log('[Auth] JWT response status:', response.status);

    if (!response.ok) {
      const errorText = await response.text();
      console.error('[Auth] Failed to get JWT token:', response.status, errorText);
      return null;
    }

    const data = await response.json();
    console.log('[Auth] JWT token received:', data.token ? 'yes (length: ' + data.token.length + ')' : 'no');
    return data.token || null;
  } catch (error) {
    console.error('[Auth] Error getting JWT token:', error);
    return null;
  }
}

export default authClient;
