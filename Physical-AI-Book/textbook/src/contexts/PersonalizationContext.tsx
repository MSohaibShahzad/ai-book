/**
 * PersonalizationContext
 *
 * React Context for managing chapter personalization state.
 * Provides session-only caching of personalized content and rate limit tracking.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 */

import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';

/**
 * User profile metadata for personalization
 */
export interface UserProfile {
  softwareBackground: 'Beginner' | 'Intermediate' | 'Advanced' | 'Expert';
  hardwareBackground: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  interestArea: 'AI' | 'Robotics' | 'Computer Vision' | 'Motion Control' | 'General';
}

/**
 * Personalized content cache entry
 */
export interface PersonalizedContent {
  chapterId: string;
  personalizedMarkdown: string;
  originalMarkdown: string;
  generatedAt: Date;
  profileSnapshot: UserProfile;
}

/**
 * Current view mode for chapter display
 */
export type ViewMode = 'original' | 'personalized';

/**
 * Personalization state
 */
export interface PersonalizationState {
  // Cached personalized content (Map<chapterId, PersonalizedContent>)
  personalizedContent: Map<string, PersonalizedContent>;

  // Rate limit remaining (0-3)
  rateLimitRemaining: number;

  // Rate limit reset timestamp (ISO 8601)
  rateLimitResetAt: string | null;

  // Current view mode
  currentView: ViewMode;

  // Loading state
  isLoading: boolean;

  // Error message
  error: string | null;

  // Profile update notification
  showProfileUpdateNotification: boolean;

  // Actions
  setPersonalizedContent: (chapterId: string, content: PersonalizedContent) => void;
  getPersonalizedContent: (chapterId: string) => PersonalizedContent | undefined;
  setRateLimitRemaining: (remaining: number) => void;
  setRateLimitResetAt: (resetAt: string | null) => void;
  setCurrentView: (view: ViewMode) => void;
  setIsLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  clearCache: () => void;
  clearCacheOnProfileUpdate: () => Promise<void>;
  dismissProfileUpdateNotification: () => void;
}

/**
 * Default context value
 */
const defaultState: PersonalizationState = {
  personalizedContent: new Map(),
  rateLimitRemaining: 3,
  rateLimitResetAt: null,
  currentView: 'original',
  isLoading: false,
  error: null,
  showProfileUpdateNotification: false,
  setPersonalizedContent: () => {},
  getPersonalizedContent: () => undefined,
  setRateLimitRemaining: () => {},
  setRateLimitResetAt: () => {},
  setCurrentView: () => {},
  setIsLoading: () => {},
  setError: () => {},
  clearCache: () => {},
  clearCacheOnProfileUpdate: async () => {},
  dismissProfileUpdateNotification: () => {},
};

/**
 * Create context
 */
export const PersonalizationContext = createContext<PersonalizationState>(defaultState);

/**
 * Hook to use PersonalizationContext
 */
export function usePersonalizationContext(): PersonalizationState {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalizationContext must be used within PersonalizationProvider');
  }
  return context;
}

/**
 * PersonalizationProvider Props
 */
interface PersonalizationProviderProps {
  children: ReactNode;
}

/**
 * PersonalizationProvider Component
 *
 * Provides personalization state to all child components.
 * State is session-only and cleared on page reload.
 */
export function PersonalizationProvider({ children }: PersonalizationProviderProps) {
  const [personalizedContent, setPersonalizedContentState] = useState<Map<string, PersonalizedContent>>(new Map());
  const [rateLimitRemaining, setRateLimitRemaining] = useState<number>(3);
  const [rateLimitResetAt, setRateLimitResetAt] = useState<string | null>(null);
  const [currentView, setCurrentView] = useState<ViewMode>('original');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [showProfileUpdateNotification, setShowProfileUpdateNotification] = useState<boolean>(false);

  /**
   * Store personalized content in cache
   */
  const setPersonalizedContent = useCallback((chapterId: string, content: PersonalizedContent) => {
    setPersonalizedContentState(prev => {
      const newMap = new Map(prev);
      newMap.set(chapterId, content);
      return newMap;
    });
  }, []);

  /**
   * Retrieve personalized content from cache
   */
  const getPersonalizedContent = useCallback((chapterId: string): PersonalizedContent | undefined => {
    return personalizedContent.get(chapterId);
  }, [personalizedContent]);

  /**
   * Clear all cached personalized content
   * Called when user updates profile
   */
  const clearCache = useCallback(() => {
    console.log('[Personalization] Cache cleared:', personalizedContent.size, 'entries');
    setPersonalizedContentState(new Map());
    setCurrentView('original');
    setError(null);
  }, [personalizedContent.size]);

  /**
   * Clear cache on profile update and show notification
   * Calls backend API to invalidate server-side cache (if any)
   */
  const clearCacheOnProfileUpdate = useCallback(async () => {
    const { invalidateCache } = await import('../lib/personalizationService');

    try {
      console.log('[Personalization] Profile updated - invalidating cache');

      // Call backend API to invalidate cache
      await invalidateCache();

      // Clear client-side cache
      clearCache();

      // Show notification
      setShowProfileUpdateNotification(true);

      console.log('[Personalization] Cache invalidated successfully');
    } catch (err) {
      console.error('[Personalization] Error invalidating cache:', err);
      // Still clear client-side cache even if API call fails
      clearCache();
      setShowProfileUpdateNotification(true);
    }
  }, [clearCache]);

  /**
   * Dismiss profile update notification
   */
  const dismissProfileUpdateNotification = useCallback(() => {
    setShowProfileUpdateNotification(false);
  }, []);

  const value: PersonalizationState = {
    personalizedContent,
    rateLimitRemaining,
    rateLimitResetAt,
    currentView,
    isLoading,
    error,
    showProfileUpdateNotification,
    setPersonalizedContent,
    getPersonalizedContent,
    setRateLimitRemaining,
    setRateLimitResetAt,
    setCurrentView,
    setIsLoading,
    setError,
    clearCache,
    clearCacheOnProfileUpdate,
    dismissProfileUpdateNotification,
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
}
