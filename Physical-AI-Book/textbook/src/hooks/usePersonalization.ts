/**
 * usePersonalization Hook
 *
 * Custom React hook for chapter personalization API calls.
 * Manages personalization state, API calls, and error handling.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T018
 */

import { useCallback } from 'react';
import { usePersonalizationContext, UserProfile } from '../contexts/PersonalizationContext';
import {
  personalizeChapter,
  PersonalizeChapterRequest,
  PersonalizationError,
} from '../lib/personalizationService';
import { useSession } from '../lib/auth-client';

/**
 * Return type for usePersonalization hook
 */
export interface UsePersonalizationReturn {
  /**
   * Personalize chapter content
   * @param chapterId Chapter identifier
   * @param content Original chapter markdown
   * @param profile User profile metadata
   */
  personalize: (chapterId: string, content: string, profile: UserProfile) => Promise<void>;

  /**
   * Loading state (true while personalization in progress)
   */
  isLoading: boolean;

  /**
   * Error message (null if no error)
   */
  error: string | null;

  /**
   * Get personalized content for a chapter
   * @param chapterId Chapter identifier
   * @returns Personalized markdown or undefined if not cached
   */
  getPersonalizedContent: (chapterId: string) => string | undefined;

  /**
   * Remaining personalization requests
   */
  rateLimitRemaining: number;

  /**
   * Clear error message
   */
  clearError: () => void;
}

/**
 * Hook for chapter personalization
 *
 * Usage:
 * ```tsx
 * const { personalize, isLoading, error, getPersonalizedContent } = usePersonalization();
 *
 * const handlePersonalize = async () => {
 *   await personalize('01-foundations-ros2/01-what-is-ros2', chapterMarkdown, userProfile);
 * };
 * ```
 */
export function usePersonalization(): UsePersonalizationReturn {
  const {
    personalizedContent,
    rateLimitRemaining,
    isLoading,
    error,
    setPersonalizedContent,
    getPersonalizedContent: getContentFromContext,
    setRateLimitRemaining,
    setRateLimitResetAt,
    setCurrentView,
    setIsLoading,
    setError,
  } = usePersonalizationContext();

  /**
   * Personalize chapter content
   */
  const personalize = useCallback(
    async (chapterId: string, content: string, profile: UserProfile): Promise<void> => {
      // Check if already personalized (use cached version)
      const cached = getContentFromContext(chapterId);
      if (cached) {
        console.log(`[usePersonalization] Using cached content for ${chapterId}`);
        return;
      }

      // Prevent concurrent requests for the same chapter
      if (isLoading) {
        console.warn(`[usePersonalization] Already processing personalization request, ignoring duplicate`);
        return;
      }

      // Start loading
      setIsLoading(true);
      setError(null);

      try {
        console.log(`[usePersonalization] Personalizing ${chapterId}...`);

        // Build request
        const request: PersonalizeChapterRequest = {
          chapter_id: chapterId,
          chapter_content: content,
          user_profile: profile,
        };

        // Call API
        const response = await personalizeChapter(request);

        // Store personalized content in cache
        setPersonalizedContent(chapterId, {
          chapterId,
          personalizedMarkdown: response.personalized_content,
          originalMarkdown: content,
          generatedAt: new Date(),
          profileSnapshot: profile,
        });

        // Update rate limit and reset timestamp
        setRateLimitRemaining(response.remaining_limit);
        setRateLimitResetAt(response.reset_at);

        // Automatically switch to personalized view
        setCurrentView('personalized');

        console.log(
          `[usePersonalization] Successfully personalized ${chapterId} (${(response.generation_time_ms / 1000).toFixed(1)}s)`
        );
      } catch (err) {
        console.error('[usePersonalization] Error:', err);

        // Handle PersonalizationError
        if (err instanceof PersonalizationError) {
          if (err.isRateLimitError()) {
            setError(
              `Daily personalization limit exceeded. ${err.details?.reset_at ? `Resets at ${new Date(err.details.reset_at).toLocaleString()}` : 'Try again tomorrow.'}`
            );
            setRateLimitRemaining(0);
            // Store reset_at from error details
            if (err.details?.reset_at) {
              setRateLimitResetAt(err.details.reset_at);
            }
          } else if (err.isTimeoutError()) {
            setError(
              'Personalization timed out after 30 seconds. Please try again with a shorter chapter or try again later.'
            );
          } else if (err.isValidationError()) {
            setError(
              `Profile incomplete. Please complete your profile to enable personalization. Missing: ${err.details?.missing_fields?.join(', ') || 'unknown fields'}`
            );
          } else if (err.isAuthError()) {
            setError('You must be signed in to personalize chapters. Please sign in and try again.');
          } else {
            setError(
              `Failed to personalize chapter: ${err.message}. Please try again later.`
            );
          }
        } else if (err instanceof Error) {
          setError(`Unexpected error: ${err.message}`);
        } else {
          setError('An unknown error occurred. Please try again.');
        }
      } finally {
        setIsLoading(false);
      }
    },
    [
      getContentFromContext,
      isLoading,
      setIsLoading,
      setError,
      setPersonalizedContent,
      setRateLimitRemaining,
      setRateLimitResetAt,
      setCurrentView,
    ]
  );

  /**
   * Get personalized content for a chapter
   */
  const getPersonalizedContent = useCallback(
    (chapterId: string): string | undefined => {
      const content = getContentFromContext(chapterId);
      return content?.personalizedMarkdown;
    },
    [getContentFromContext]
  );

  /**
   * Clear error message
   */
  const clearError = useCallback(() => {
    setError(null);
  }, [setError]);

  return {
    personalize,
    isLoading,
    error,
    getPersonalizedContent,
    rateLimitRemaining,
    clearError,
  };
}
