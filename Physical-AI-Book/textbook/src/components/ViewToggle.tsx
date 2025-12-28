/**
 * ViewToggle Component
 *
 * Toggle between original and personalized chapter views.
 * Preserves scroll position when switching views.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Tasks: T022, T023, T024
 */

import React, { useRef, useEffect } from 'react';
import { ViewMode, usePersonalizationContext } from '../contexts/PersonalizationContext';

/**
 * Props for ViewToggle
 */
export interface ViewToggleProps {
  /**
   * User's display name (for badge)
   */
  userName?: string;

  /**
   * Whether personalized content exists
   */
  hasPersonalizedContent: boolean;

  /**
   * Callback when view changes
   */
  onViewChange?: (view: ViewMode) => void;
}

/**
 * ViewToggle Component
 *
 * Displays "Original" / "Personalized" toggle with active state.
 * Preserves scroll position when switching views.
 *
 * Usage:
 * ```tsx
 * <ViewToggle
 *   userName="John Doe"
 *   hasPersonalizedContent={true}
 *   onViewChange={(view) => console.log('View changed to:', view)}
 * />
 * ```
 */
export function ViewToggle({
  userName,
  hasPersonalizedContent,
  onViewChange,
}: ViewToggleProps) {
  const { currentView, setCurrentView } = usePersonalizationContext();
  const scrollPositionRef = useRef<number>(0);

  /**
   * Preserve scroll position when view changes
   */
  useEffect(() => {
    // Save scroll position before view change
    const handleScroll = () => {
      scrollPositionRef.current = window.scrollY;
    };

    window.addEventListener('scroll', handleScroll, { passive: true });

    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  /**
   * Handle view toggle with scroll preservation
   */
  const handleViewToggle = (view: ViewMode) => {
    if (view === currentView) return;

    // Save current scroll position
    const currentScrollY = window.scrollY;
    scrollPositionRef.current = currentScrollY;

    // Update view
    setCurrentView(view);
    onViewChange?.(view);

    // Restore scroll position after render (using requestAnimationFrame for better timing)
    requestAnimationFrame(() => {
      window.scrollTo({
        top: currentScrollY,
        behavior: 'auto', // Instant scroll to maintain position
      });
    });
  };

  // Don't render if no personalized content
  if (!hasPersonalizedContent) {
    return null;
  }

  return (
    <div
      className="view-toggle-container"
      style={{
        marginBottom: '1.5rem',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        padding: '1rem',
        backgroundColor: 'var(--ifm-color-emphasis-100)',
        borderRadius: '8px',
        border: '1px solid var(--ifm-color-emphasis-200)',
      }}
    >
      {/* Toggle buttons */}
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
        <span
          style={{
            fontWeight: 600,
            fontSize: '0.875rem',
            color: 'var(--ifm-color-emphasis-700)',
          }}
        >
          View:
        </span>
        <div
          className="button-group button-group--block"
          role="group"
          aria-label="Toggle between original and personalized views"
          style={{ display: 'flex', gap: '0.5rem' }}
        >
          <button
            onClick={() => handleViewToggle('original')}
            className={`button button--sm ${
              currentView === 'original'
                ? 'button--primary'
                : 'button--secondary button--outline'
            }`}
            aria-pressed={currentView === 'original'}
            style={{
              minWidth: '100px',
              transition: 'all 0.2s ease',
            }}
          >
            📄 Original
          </button>
          <button
            onClick={() => handleViewToggle('personalized')}
            className={`button button--sm ${
              currentView === 'personalized'
                ? 'button--primary'
                : 'button--secondary button--outline'
            }`}
            aria-pressed={currentView === 'personalized'}
            style={{
              minWidth: '100px',
              transition: 'all 0.2s ease',
            }}
          >
            ✨ Personalized
          </button>
        </div>
      </div>

      {/* Badge for personalized view */}
      {currentView === 'personalized' && (
        <div
          className="badge badge--success"
          style={{
            fontSize: '0.875rem',
            padding: '0.375rem 0.75rem',
            display: 'flex',
            alignItems: 'center',
            gap: '0.375rem',
          }}
        >
          <span>✨</span>
          <span>Personalized {userName ? `for ${userName}` : 'for you'}</span>
        </div>
      )}
    </div>
  );
}

/**
 * Hook for scroll position preservation
 *
 * Usage in parent component:
 * ```tsx
 * const scrollPosition = useScrollPreservation(currentView);
 * ```
 */
export function useScrollPreservation(currentView: ViewMode): number {
  const scrollPositionRef = useRef<number>(0);

  useEffect(() => {
    // Save scroll position when view changes
    scrollPositionRef.current = window.scrollY;

    // Restore scroll position after render
    const timer = requestAnimationFrame(() => {
      window.scrollTo({
        top: scrollPositionRef.current,
        behavior: 'auto',
      });
    });

    return () => {
      cancelAnimationFrame(timer);
    };
  }, [currentView]);

  return scrollPositionRef.current;
}
