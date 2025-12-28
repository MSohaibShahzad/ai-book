/**
 * PersonalizeButton Component
 *
 * Button for triggering chapter personalization.
 * Displays "Personalize for Me" for authenticated users with complete profiles.
 * Shows remaining limit after first use.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Tasks: T016, T017
 */

import React from 'react';
import Link from '@docusaurus/Link';
import { UserProfile, usePersonalizationContext } from '../contexts/PersonalizationContext';
import { usePersonalization } from '../hooks/usePersonalization';
import { useRateLimit } from '../hooks/useRateLimit';
import { RateLimitDisplay } from './RateLimitDisplay';

/**
 * Props for PersonalizeButton
 */
export interface PersonalizeButtonProps {
  /**
   * Chapter identifier (e.g., 'foundations-ros2/nodes-topics')
   */
  chapterId: string;

  /**
   * Original chapter markdown content
   */
  chapterContent: string;

  /**
   * User profile (null if not authenticated or profile incomplete)
   */
  userProfile: UserProfile | null;

  /**
   * Whether user is authenticated
   */
  isAuthenticated: boolean;

  /**
   * Callback when personalization starts
   */
  onPersonalizationStart?: () => void;

  /**
   * Callback when personalization completes
   */
  onPersonalizationComplete?: () => void;

  /**
   * Callback when personalization fails
   */
  onPersonalizationError?: (error: string) => void;
}

/**
 * Check if user profile is complete
 */
function isProfileComplete(profile: UserProfile | null): profile is UserProfile {
  if (!profile) return false;

  return !!(
    profile.softwareBackground &&
    profile.hardwareBackground &&
    profile.interestArea
  );
}

/**
 * Get missing profile fields
 */
function getMissingFields(profile: UserProfile | null): string[] {
  if (!profile) return ['softwareBackground', 'hardwareBackground', 'interestArea'];

  const missing: string[] = [];

  if (!profile.softwareBackground) missing.push('Software Background');
  if (!profile.hardwareBackground) missing.push('Hardware Background');
  if (!profile.interestArea) missing.push('Interest Area');

  return missing;
}

/**
 * PersonalizeButton Component
 *
 * Usage:
 * ```tsx
 * <PersonalizeButton
 *   chapterId="foundations-ros2/what-is-ros2"
 *   chapterContent={markdownContent}
 *   userProfile={userProfile}
 *   isAuthenticated={isAuthenticated}
 * />
 * ```
 */
export function PersonalizeButton({
  chapterId,
  chapterContent,
  userProfile,
  isAuthenticated,
  onPersonalizationStart,
  onPersonalizationComplete,
  onPersonalizationError,
}: PersonalizeButtonProps) {
  const { personalize, isLoading, rateLimitRemaining } = usePersonalization();
  const { rateLimitResetAt } = usePersonalizationContext();

  // Fetch quota status on mount ONLY if authenticated (updates context with real limit)
  const rateLimitData = useRateLimit();

  // Trigger quota fetch when auth becomes available (only once)
  const hasInitialFetch = React.useRef(false);
  React.useEffect(() => {
    if (isAuthenticated && rateLimitData.refresh && !hasInitialFetch.current) {
      hasInitialFetch.current = true;
      rateLimitData.refresh();
    }
  }, [isAuthenticated, rateLimitData.refresh]);

  // State to track which alert to show
  const [showAlert, setShowAlert] = React.useState<'none' | 'signin' | 'profile' | 'ratelimit'>('none');

  // Track if personalization is in progress to prevent double-clicks
  const isProcessing = React.useRef(false);

  /**
   * Handle personalize button click
   */
  const handlePersonalize = async () => {
    // Prevent double-clicks/rapid clicks
    if (isProcessing.current || isLoading) {
      console.warn('[PersonalizeButton] Ignoring duplicate click - already processing');
      return;
    }

    // Check if user is signed in
    if (!isAuthenticated) {
      setShowAlert('signin');
      return;
    }

    // Check if profile is complete
    if (!isProfileComplete(userProfile)) {
      setShowAlert('profile');
      return;
    }

    // Check rate limit
    if (rateLimitRemaining === 0) {
      setShowAlert('ratelimit');
      return;
    }

    // Everything is good - proceed with personalization
    setShowAlert('none');
    isProcessing.current = true;

    try {
      onPersonalizationStart?.();
      await personalize(chapterId, chapterContent, userProfile);
      onPersonalizationComplete?.();
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      onPersonalizationError?.(errorMessage);
    } finally {
      isProcessing.current = false;
    }
  };

  // Get missing fields for profile alert
  const missingFields = getMissingFields(userProfile);

  return (
    <div className="personalize-button-container" style={{ marginBottom: '1rem' }}>
      {/* Always show the Personalize button */}
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        className={`button button--primary button--lg ${isLoading ? 'button--disabled' : ''}`}
        style={{
          width: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '0.5rem',
        }}
      >
        {isLoading ? (
          <>
            <span className="spinner-border spinner-border-sm" role="status" aria-hidden="true" />
            <span>Generating personalized content...</span>
          </>
        ) : (
          <>
            <span>✨ Personalize for Me</span>
            {isAuthenticated && rateLimitRemaining < 3 && rateLimitRemaining > 0 && (
              <span style={{ fontSize: '0.875rem', opacity: 0.8 }}>
                ({rateLimitRemaining} remaining)
              </span>
            )}
          </>
        )}
      </button>

      {/* Alert: Sign in required */}
      {showAlert === 'signin' && (
        <div className="alert alert--warning" style={{ marginTop: '1rem' }}>
          <strong>⚠️ Sign in to personalize the content</strong>
          <p style={{ marginTop: '0.5rem', marginBottom: '0.75rem' }}>
            Please sign in to personalize this chapter based on your background and interests.
          </p>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <Link to="/signin" className="button button--primary button--sm">
              Sign In
            </Link>
            <Link to="/signup" className="button button--secondary button--sm">
              Sign Up
            </Link>
            <button
              onClick={() => setShowAlert('none')}
              className="button button--link button--sm"
            >
              Close
            </button>
          </div>
        </div>
      )}

      {/* Alert: Profile incomplete */}
      {showAlert === 'profile' && (
        <div className="alert alert--warning" style={{ marginTop: '1rem' }}>
          <strong>⚠️ Complete your profile to personalize</strong>
          <p style={{ marginTop: '0.5rem', marginBottom: '0.5rem' }}>
            Missing information:
          </p>
          <ul style={{ marginBottom: '0.75rem', paddingLeft: '1.5rem' }}>
            {missingFields.map((field) => (
              <li key={field}>Your {field} is missing</li>
            ))}
          </ul>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <Link to="/profile/settings" className="button button--primary button--sm">
              Complete Profile
            </Link>
            <button
              onClick={() => setShowAlert('none')}
              className="button button--link button--sm"
            >
              Close
            </button>
          </div>
        </div>
      )}

      {/* Alert: Rate limit reached */}
      {showAlert === 'ratelimit' && (
        <div className="alert alert--danger" style={{ marginTop: '1rem' }}>
          <strong>⛔ Daily limit reached</strong>
          <p style={{ marginTop: '0.5rem', marginBottom: '0.75rem' }}>
            You've used all 3 personalizations today.
            {rateLimitResetAt && (() => {
              try {
                const resetDate = new Date(rateLimitResetAt);
                if (!isNaN(resetDate.getTime())) {
                  return <> Your limit will reset at: <strong>{resetDate.toLocaleString()}</strong></>;
                }
              } catch {}
              return null;
            })()}
          </p>
          <button
            onClick={() => setShowAlert('none')}
            className="button button--link button--sm"
          >
            Close
          </button>
        </div>
      )}

      {/* Show rate limit display when authenticated (only when no alert) */}
      {showAlert === 'none' && isAuthenticated && (
        <RateLimitDisplay resetAt={rateLimitResetAt ?? undefined} compact={true} />
      )}
    </div>
  );
}
