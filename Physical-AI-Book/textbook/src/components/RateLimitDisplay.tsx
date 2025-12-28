/**
 * RateLimitDisplay Component
 *
 * Displays remaining personalization requests and reset time.
 * Shows "Remaining limit: X" after first use, "Limit exceeded" when quota exhausted.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T028
 */

import React from 'react';
import { usePersonalizationContext } from '../contexts/PersonalizationContext';

/**
 * Props for RateLimitDisplay
 */
export interface RateLimitDisplayProps {
  /**
   * Reset timestamp (ISO 8601 format)
   */
  resetAt?: string;

  /**
   * Show reset time (default: true)
   */
  showResetTime?: boolean;

  /**
   * Compact mode (smaller text, less padding)
   */
  compact?: boolean;
}

/**
 * Format reset time as relative string
 */
function formatResetTime(resetAt: string | undefined): string {
  if (!resetAt) {
    return 'soon';
  }

  try {
    const resetDate = new Date(resetAt);

    // Check if date is valid
    if (isNaN(resetDate.getTime())) {
      return 'soon';
    }

    const now = new Date();
    const diffMs = resetDate.getTime() - now.getTime();

    if (diffMs <= 0) {
      return 'soon';
    }

    const diffHours = Math.floor(diffMs / (1000 * 60 * 60));
    const diffMinutes = Math.floor((diffMs % (1000 * 60 * 60)) / (1000 * 60));

    if (diffHours > 0) {
      return `${diffHours}h ${diffMinutes}m`;
    }

    return `${diffMinutes}m`;
  } catch {
    return 'soon';
  }
}

/**
 * RateLimitDisplay Component
 *
 * Displays personalization quota with visual feedback.
 *
 * Usage:
 * ```tsx
 * <RateLimitDisplay resetAt="2025-12-24T10:00:00Z" />
 * ```
 */
export function RateLimitDisplay({
  resetAt,
  showResetTime = true,
  compact = false,
}: RateLimitDisplayProps) {
  const { rateLimitRemaining } = usePersonalizationContext();

  /**
   * Determine status color
   */
  const getStatusColor = (): string => {
    if (rateLimitRemaining === 0) return 'var(--ifm-color-danger)';
    if (rateLimitRemaining === 1) return 'var(--ifm-color-warning)';
    return 'var(--ifm-color-success)';
  };

  /**
   * Determine alert type
   */
  const getAlertType = (): string => {
    if (rateLimitRemaining === 0) return 'alert--danger';
    if (rateLimitRemaining === 1) return 'alert--warning';
    return 'alert--info';
  };

  /**
   * Render status icon
   */
  const renderStatusIcon = (): string => {
    if (rateLimitRemaining === 0) return '🚫';
    if (rateLimitRemaining === 1) return '⚠️';
    return '✅';
  };

  /**
   * Render message
   */
  const renderMessage = (): React.ReactNode => {
    if (rateLimitRemaining === 0) {
      return (
        <>
          <strong>Daily limit reached</strong>
          {resetAt && showResetTime && (
            <div style={{ fontSize: '0.875rem', marginTop: '0.25rem' }}>
              Resets in {formatResetTime(resetAt)}
            </div>
          )}
        </>
      );
    }

    return (
      <>
        <strong>
          {rateLimitRemaining} personalization{rateLimitRemaining === 1 ? '' : 's'} remaining
        </strong>
        {resetAt && showResetTime && rateLimitRemaining < 3 && (
          <div style={{ fontSize: '0.875rem', marginTop: '0.25rem', opacity: 0.8 }}>
            Resets in {formatResetTime(resetAt)}
          </div>
        )}
      </>
    );
  };

  return (
    <div
      className={`rate-limit-display ${compact ? 'rate-limit-display--compact' : ''}`}
      style={{
        marginTop: compact ? '0.5rem' : '1rem',
      }}
    >
      <div
        className={`alert ${getAlertType()}`}
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: '0.75rem',
          padding: compact ? '0.5rem 0.75rem' : '0.75rem 1rem',
          fontSize: compact ? '0.875rem' : '1rem',
          margin: 0,
        }}
      >
        <span style={{ fontSize: compact ? '1.25rem' : '1.5rem' }}>{renderStatusIcon()}</span>
        <div style={{ flex: 1 }}>{renderMessage()}</div>
        {rateLimitRemaining < 3 && (
          <div
            style={{
              display: 'flex',
              gap: '0.25rem',
              alignItems: 'center',
            }}
          >
            {[...Array(3)].map((_, i) => (
              <div
                key={i}
                style={{
                  width: compact ? '8px' : '10px',
                  height: compact ? '8px' : '10px',
                  borderRadius: '50%',
                  backgroundColor: i < rateLimitRemaining ? getStatusColor() : 'var(--ifm-color-emphasis-300)',
                  transition: 'background-color 0.3s ease',
                }}
                title={`Request ${i + 1}/3`}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
