/**
 * ProfileUpdateNotification Component
 *
 * Displays notification when user updates profile, informing them to re-personalize chapters.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T027
 */

import React, { useEffect } from 'react';
import { usePersonalizationContext } from '../contexts/PersonalizationContext';

/**
 * Props for ProfileUpdateNotification
 */
export interface ProfileUpdateNotificationProps {
  /**
   * Auto-dismiss after N milliseconds (default: 10000 = 10 seconds)
   */
  autoDismissAfter?: number;

  /**
   * Position of notification (default: 'bottom-right')
   */
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';
}

/**
 * ProfileUpdateNotification Component
 *
 * Shows a notification after profile update, informing user to re-personalize chapters.
 * Auto-dismisses after 10 seconds.
 *
 * Usage:
 * ```tsx
 * // Add to root layout (e.g., Layout component)
 * <ProfileUpdateNotification />
 * ```
 *
 * Trigger notification:
 * ```tsx
 * const { clearCacheOnProfileUpdate } = usePersonalizationContext();
 *
 * // Call when user updates profile
 * await clearCacheOnProfileUpdate();
 * ```
 */
export function ProfileUpdateNotification({
  autoDismissAfter = 10000,
  position = 'bottom-right',
}: ProfileUpdateNotificationProps) {
  const { showProfileUpdateNotification, dismissProfileUpdateNotification } =
    usePersonalizationContext();

  /**
   * Auto-dismiss notification after timeout
   */
  useEffect(() => {
    if (showProfileUpdateNotification && autoDismissAfter > 0) {
      const timer = setTimeout(() => {
        dismissProfileUpdateNotification();
      }, autoDismissAfter);

      return () => clearTimeout(timer);
    }
  }, [showProfileUpdateNotification, autoDismissAfter, dismissProfileUpdateNotification]);

  /**
   * Don't render if notification not shown
   */
  if (!showProfileUpdateNotification) {
    return null;
  }

  /**
   * Position styles
   */
  const positionStyles: Record<string, React.CSSProperties> = {
    'top-left': { top: '1rem', left: '1rem' },
    'top-right': { top: '1rem', right: '1rem' },
    'bottom-left': { bottom: '1rem', left: '1rem' },
    'bottom-right': { bottom: '1rem', right: '1rem' },
  };

  return (
    <div
      className="profile-update-notification"
      style={{
        position: 'fixed',
        ...positionStyles[position],
        zIndex: 9999,
        maxWidth: '400px',
        minWidth: '320px',
        animation: 'slideIn 0.3s ease-out',
      }}
      role="alert"
      aria-live="polite"
    >
      <div
        className="alert alert--info"
        style={{
          margin: 0,
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
          display: 'flex',
          flexDirection: 'column',
          gap: '0.75rem',
        }}
      >
        {/* Header with dismiss button */}
        <div style={{ display: 'flex', alignItems: 'flex-start', justifyContent: 'space-between' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <span style={{ fontSize: '1.25rem' }}>ℹ️</span>
            <strong style={{ fontSize: '1rem' }}>Profile Updated</strong>
          </div>
          <button
            onClick={dismissProfileUpdateNotification}
            className="close"
            aria-label="Dismiss notification"
            style={{
              background: 'none',
              border: 'none',
              fontSize: '1.5rem',
              cursor: 'pointer',
              padding: 0,
              lineHeight: 1,
              color: 'var(--ifm-color-emphasis-600)',
              marginLeft: '0.5rem',
            }}
          >
            ×
          </button>
        </div>

        {/* Message */}
        <p style={{ margin: 0, fontSize: '0.875rem', lineHeight: 1.5 }}>
          Your personalized chapters have been cleared. Re-personalize your chapters to see content
          adapted to your updated profile.
        </p>

        {/* Dismiss button */}
        <button
          onClick={dismissProfileUpdateNotification}
          className="button button--secondary button--sm"
          style={{ alignSelf: 'flex-end', marginTop: '0.25rem' }}
        >
          Got it
        </button>
      </div>

      {/* Inline animation */}
      <style>
        {`
          @keyframes slideIn {
            from {
              transform: translateX(${position.includes('right') ? '100%' : '-100%'});
              opacity: 0;
            }
            to {
              transform: translateX(0);
              opacity: 1;
            }
          }
        `}
      </style>
    </div>
  );
}
