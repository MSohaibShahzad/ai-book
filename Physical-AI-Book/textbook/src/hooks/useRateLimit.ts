/**
 * useRateLimit Hook
 *
 * Custom React hook for fetching and managing personalization quota.
 * Fetches quota from backend API and provides remaining count and reset timestamp.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T031
 */

import { useState, useEffect, useCallback } from 'react';
import { usePersonalizationContext } from '../contexts/PersonalizationContext';
import { getQuotaStatus, QuotaStatusResponse } from '../lib/personalizationService';

/**
 * Return type for useRateLimit hook
 */
export interface UseRateLimitReturn {
  /**
   * Remaining personalization requests (0-3)
   */
  remaining: number;

  /**
   * Total requests allowed per day
   */
  total: number;

  /**
   * Reset timestamp (ISO 8601 format)
   */
  resetAt: string | null;

  /**
   * Hours until quota resets
   */
  hoursUntilReset: number | null;

  /**
   * Loading state
   */
  isLoading: boolean;

  /**
   * Error message
   */
  error: string | null;

  /**
   * Refresh quota status from backend
   */
  refresh: () => Promise<void>;
}

/**
 * Hook for personalization rate limit management
 *
 * Fetches quota status from backend and syncs with context.
 * Automatically refreshes on mount and when context remaining changes.
 *
 * Usage:
 * ```tsx
 * const { remaining, resetAt, refresh } = useRateLimit();
 *
 * useEffect(() => {
 *   // Refresh quota after personalization
 *   refresh();
 * }, [personalizedContent]);
 * ```
 */
export function useRateLimit(): UseRateLimitReturn {
  const { rateLimitRemaining, setRateLimitRemaining, setRateLimitResetAt } = usePersonalizationContext();

  const [quotaStatus, setQuotaStatus] = useState<QuotaStatusResponse | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Fetch quota status from backend
   */
  const fetchQuotaStatus = useCallback(async () => {
    setIsLoading(true);
    setError(null);

    try {
      console.log('[useRateLimit] Fetching quota status...');

      const status = await getQuotaStatus();

      console.log('[useRateLimit] Quota status:', status);

      // Update context
      setRateLimitRemaining(status.remaining_requests);
      setRateLimitResetAt(status.reset_at);

      // Update local state
      setQuotaStatus(status);
    } catch (err) {
      // Only log non-auth errors (401 is expected when not signed in)
      if (err instanceof Error && !err.message.includes('401') && !err.message.includes('unauthorized')) {
        console.error('[useRateLimit] Error fetching quota:', err);
      }

      const errorMessage =
        err instanceof Error ? err.message : 'Failed to fetch quota status';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [setRateLimitRemaining, setRateLimitResetAt]);

  /**
   * Fetch quota on mount (only once)
   */
  useEffect(() => {
    // Only fetch if we might be authenticated (check for session cookie or token)
    // The API call will handle auth validation
    // Note: Empty dependency array ensures this runs only once on mount
    fetchQuotaStatus().catch((err) => {
      // Silently fail if not authenticated (401 errors are expected)
      // This prevents console errors on page load when user is not signed in
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  /**
   * Refresh quota (public method)
   */
  const refresh = useCallback(async () => {
    await fetchQuotaStatus();
  }, [fetchQuotaStatus]);

  return {
    remaining: rateLimitRemaining,
    total: quotaStatus?.total_requests ?? 3,
    resetAt: quotaStatus?.reset_at ?? null,
    hoursUntilReset: quotaStatus?.hours_until_reset ?? null,
    isLoading,
    error,
    refresh,
  };
}

/**
 * Hook for quota status polling
 *
 * Automatically refreshes quota at specified interval.
 *
 * Usage:
 * ```tsx
 * const quota = useRateLimitPolling(60000); // Refresh every 60 seconds
 * ```
 */
export function useRateLimitPolling(intervalMs: number = 60000): UseRateLimitReturn {
  const rateLimitData = useRateLimit();
  const { refresh } = rateLimitData;

  /**
   * Poll quota status at interval
   */
  useEffect(() => {
    if (intervalMs <= 0) return;

    const intervalId = setInterval(() => {
      console.log('[useRateLimitPolling] Refreshing quota...');
      refresh();
    }, intervalMs);

    return () => {
      clearInterval(intervalId);
    };
  }, [intervalMs, refresh]);

  return rateLimitData;
}
