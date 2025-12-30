/**
 * Personalization API Service
 *
 * Client for backend personalization API endpoints.
 * Handles chapter personalization, quota management, and cache invalidation.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 */

import { UserProfile } from '../contexts/PersonalizationContext';
import { getJWTToken } from './auth-client';

/**
 * Get API base URL based on environment
 */
const getApiBaseUrl = () => {
  // During SSR/build time, return a default value
  if (typeof window === 'undefined') return 'http://localhost:8000/api';

  // In development, use local backend
  if (window.location.hostname === 'localhost') {
    return 'http://localhost:8000/api';
  }

  // In production, use deployed backend (update this with your production URL)
  return 'https://ai-book-production-6886.up.railway.app/api';
};

/**
 * API base URL
 */
const API_BASE_URL = getApiBaseUrl();

/**
 * Personalization request body
 */
export interface PersonalizeChapterRequest {
  chapter_id: string;
  chapter_content: string;
  user_profile: UserProfile;
}

/**
 * Personalization response body
 */
export interface PersonalizeChapterResponse {
  personalized_content: string;
  remaining_limit: number;
  generation_time_ms: number;
  reset_at: string; // ISO 8601 timestamp
}

/**
 * Quota status response body
 */
export interface QuotaStatusResponse {
  remaining_requests: number;
  total_requests: number;
  reset_at: string; // ISO 8601 timestamp
  hours_until_reset: number;
}

/**
 * Error response body
 */
export interface ErrorResponse {
  error: string;
  message: string;
  details?: Record<string, any>;
  retry_allowed?: boolean;
  remaining_limit?: number;
}

/**
 * Make authenticated API request
 *
 * Requires JWT token for authentication (same pattern as translation).
 * Fails early if token is not available.
 */
async function makeAuthenticatedRequest<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<T> {
  console.log('[Personalization] Making authenticated request to:', endpoint);

  // Get JWT token for authentication (fail early if not available)
  const token = await getJWTToken();
  console.log('[Personalization] JWT token obtained:', token ? 'yes' : 'no');

  if (!token) {
    console.error('[Personalization] No JWT token available - user not authenticated');
    throw new PersonalizationError(
      'Please sign in to personalize chapters.',
      401,
      'unauthorized',
      { login_url: '/signin' }
    );
  }

  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${token}`,
    ...options.headers as Record<string, string>,
  };

  console.log('[Personalization] Request URL:', `${API_BASE_URL}${endpoint}`);
  console.log('[Personalization] Request headers:', { ...headers, Authorization: 'Bearer [REDACTED]' });

  const response = await fetch(`${API_BASE_URL}${endpoint}`, {
    ...options,
    headers,
    credentials: 'include', // Send cookies (Better-Auth session)
  });

  console.log('[Personalization] Response status:', response.status);

  if (!response.ok) {
    const errorData: ErrorResponse = await response.json().catch(() => ({
      error: 'unknown_error',
      message: `Request failed with status ${response.status}`,
    }));

    console.error('[Personalization] Request failed:', errorData);

    throw new PersonalizationError(
      errorData.message,
      response.status,
      errorData.error,
      errorData.details
    );
  }

  // Handle 204 No Content responses
  if (response.status === 204) {
    return {} as T;
  }

  return response.json();
}

/**
 * Custom error class for personalization errors
 */
export class PersonalizationError extends Error {
  constructor(
    message: string,
    public statusCode: number,
    public errorCode: string,
    public details?: Record<string, any>
  ) {
    super(message);
    this.name = 'PersonalizationError';
  }

  /**
   * Check if error is a rate limit error
   */
  isRateLimitError(): boolean {
    return this.statusCode === 429;
  }

  /**
   * Check if error is a timeout error
   */
  isTimeoutError(): boolean {
    return this.statusCode === 408;
  }

  /**
   * Check if error is an authentication error
   */
  isAuthError(): boolean {
    return this.statusCode === 401;
  }

  /**
   * Check if error is a validation error (incomplete profile)
   */
  isValidationError(): boolean {
    return this.statusCode === 400;
  }
}

/**
 * Personalize chapter content
 *
 * @param request Personalization request with chapter content and user profile
 * @returns Personalized content and remaining quota
 * @throws PersonalizationError on API errors
 */
export async function personalizeChapter(
  request: PersonalizeChapterRequest
): Promise<PersonalizeChapterResponse> {
  console.log('[Personalization] Request started for chapter:', request.chapter_id);

  try {
    const response = await makeAuthenticatedRequest<PersonalizeChapterResponse>(
      '/personalization/personalize',
      {
        method: 'POST',
        body: JSON.stringify(request),
      }
    );

    console.log(
      `[Personalization] Response received in ${(response.generation_time_ms / 1000).toFixed(1)}s`
    );
    console.log('[Personalization] Remaining limit:', response.remaining_limit);

    return response;
  } catch (error) {
    console.error('[Personalization] Error:', error);
    throw error;
  }
}

/**
 * Get user's personalization quota status
 *
 * @returns Quota status with remaining requests and reset timestamp
 * @throws PersonalizationError on API errors
 */
export async function getQuotaStatus(): Promise<QuotaStatusResponse> {
  console.log('[Personalization] Fetching quota status');

  try {
    const response = await makeAuthenticatedRequest<QuotaStatusResponse>(
      '/personalization/quota',
      {
        method: 'GET',
      }
    );

    console.log('[Personalization] Quota status:', response);
    return response;
  } catch (error) {
    console.error('[Personalization] Error fetching quota:', error);
    throw error;
  }
}

/**
 * Invalidate personalized content cache
 *
 * Called when user updates their profile.
 * Backend endpoint returns 204 No Content.
 *
 * @throws PersonalizationError on API errors
 */
export async function invalidateCache(): Promise<void> {
  console.log('[Personalization] Invalidating cache');

  try {
    await makeAuthenticatedRequest<void>('/personalization/cache', {
      method: 'DELETE',
    });

    console.log('[Personalization] Cache invalidated successfully');
  } catch (error) {
    console.error('[Personalization] Error invalidating cache:', error);
    throw error;
  }
}

/**
 * Get metrics for monitoring (public endpoint, no auth required)
 *
 * @returns Personalization metrics
 */
export async function getMetrics(): Promise<{
  total_requests: number;
  successful_requests: number;
  failed_requests: number;
  timeout_requests: number;
  avg_duration_ms: number;
  success_rate_percent: number;
  last_updated: string;
}> {
  const response = await fetch(`${API_BASE_URL}/metrics`);

  if (!response.ok) {
    throw new Error(`Failed to fetch metrics: ${response.statusText}`);
  }

  return response.json();
}
