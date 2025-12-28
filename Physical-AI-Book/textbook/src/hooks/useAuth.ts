/**
 * useAuth Hook
 *
 * Provides authentication state and user profile for personalization.
 * Wraps Better-Auth useSession hook and transforms user data for personalization.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Integration Task: Create useAuth hook
 */

import { useSession } from '../lib/auth-client';
import { UserProfile } from '../contexts/PersonalizationContext';

/**
 * User authentication state
 */
export interface AuthState {
  /**
   * Whether user is authenticated
   */
  isAuthenticated: boolean;

  /**
   * Whether auth state is loading
   */
  isLoading: boolean;

  /**
   * User's display name
   */
  userName: string | null;

  /**
   * User's email
   */
  userEmail: string | null;

  /**
   * User profile for personalization (null if not authenticated or profile incomplete)
   */
  userProfile: UserProfile | null;

  /**
   * Whether user profile is complete (all required fields filled)
   */
  isProfileComplete: boolean;

  /**
   * User ID
   */
  userId: string | null;
}

/**
 * Check if user profile is complete for personalization
 */
function isProfileComplete(user: any): boolean {
  if (!user) return false;

  return !!(
    user.softwareBackground &&
    user.hardwareBackground &&
    user.interestArea &&
    // Validate that values are not empty strings
    user.softwareBackground.trim() !== '' &&
    user.hardwareBackground.trim() !== '' &&
    user.interestArea.trim() !== ''
  );
}

/**
 * Transform Better-Auth user data to UserProfile
 */
function transformUserProfile(user: any): UserProfile | null {
  if (!user || !isProfileComplete(user)) {
    return null;
  }

  return {
    softwareBackground: user.softwareBackground as UserProfile['softwareBackground'],
    hardwareBackground: user.hardwareBackground as UserProfile['hardwareBackground'],
    interestArea: user.interestArea as UserProfile['interestArea'],
  };
}

/**
 * Hook for authentication and user profile
 *
 * Usage:
 * ```tsx
 * const { isAuthenticated, userName, userProfile, isProfileComplete } = useAuth();
 *
 * if (isAuthenticated && userProfile) {
 *   // User can personalize chapters
 * }
 * ```
 */
export function useAuth(): AuthState {
  const { data: session, isPending } = useSession();

  // Extract user data from session
  const user = session?.user;

  return {
    isAuthenticated: !!user,
    isLoading: isPending,
    userName: user?.name || null,
    userEmail: user?.email || null,
    userId: user?.id || null,
    userProfile: transformUserProfile(user),
    isProfileComplete: isProfileComplete(user),
  };
}
