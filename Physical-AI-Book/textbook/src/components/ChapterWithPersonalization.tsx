/**
 * ChapterWithPersonalization Component
 *
 * Complete integration example showing how to use all personalization components together.
 * This component demonstrates the full workflow for chapter personalization.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Integration Example
 */

import React, { useState } from 'react';
import { PersonalizeButton } from './PersonalizeButton';
import { PersonalizedChapter } from './PersonalizedChapter';
import { ProfileUpdateNotification } from './ProfileUpdateNotification';
import { usePersonalizationContext, UserProfile } from '../contexts/PersonalizationContext';
import { usePersonalization } from '../hooks/usePersonalization';

/**
 * Props for ChapterWithPersonalization
 */
export interface ChapterWithPersonalizationProps {
  /**
   * Chapter identifier (e.g., 'foundations-ros2/nodes-topics')
   */
  chapterId: string;

  /**
   * Original chapter markdown content
   */
  chapterContent: string;

  /**
   * User profile (null if not authenticated or incomplete)
   */
  userProfile: UserProfile | null;

  /**
   * Whether user is authenticated
   */
  isAuthenticated: boolean;

  /**
   * User's display name
   */
  userName?: string;
}

/**
 * ChapterWithPersonalization Component
 *
 * Complete integration example showing:
 * 1. PersonalizeButton - Trigger personalization
 * 2. PersonalizedChapter - Display content with toggle
 * 3. ProfileUpdateNotification - Show notification after profile update
 * 4. Error handling and loading states
 *
 * Usage in Docusaurus MDX:
 * ```tsx
 * import ChapterWithPersonalization from '@site/src/components/ChapterWithPersonalization';
 * import { useAuth } from '@site/src/hooks/useAuth';
 *
 * export default function ChapterPage({ content }) {
 *   const { user, isAuthenticated } = useAuth();
 *
 *   return (
 *     <ChapterWithPersonalization
 *       chapterId="foundations-ros2/nodes-topics"
 *       chapterContent={content}
 *       userProfile={user?.profile}
 *       isAuthenticated={isAuthenticated}
 *       userName={user?.name}
 *     />
 *   );
 * }
 * ```
 */
export function ChapterWithPersonalization({
  chapterId,
  chapterContent,
  userProfile,
  isAuthenticated,
  userName,
}: ChapterWithPersonalizationProps) {
  const { currentView, getPersonalizedContent } = usePersonalizationContext();
  const { isLoading, error } = usePersonalization();
  const [showNotification, setShowNotification] = useState<string | null>(null);

  // Check if personalized content exists for this chapter
  const hasPersonalizedContent = !!getPersonalizedContent(chapterId);

  /**
   * Handle personalization start
   */
  const handlePersonalizationStart = () => {
    console.log('[ChapterWithPersonalization] Personalization started');
    setShowNotification(null);
  };

  /**
   * Handle personalization completion
   */
  const handlePersonalizationComplete = () => {
    console.log('[ChapterWithPersonalization] Personalization completed');
    setShowNotification('success');

    // Auto-dismiss success notification after 5 seconds
    setTimeout(() => {
      setShowNotification(null);
    }, 5000);
  };

  /**
   * Handle personalization error
   */
  const handlePersonalizationError = (errorMessage: string) => {
    console.error('[ChapterWithPersonalization] Personalization error:', errorMessage);
    setShowNotification('error');
  };

  /**
   * Render success notification
   */
  const renderSuccessNotification = () => {
    if (showNotification !== 'success') return null;

    return (
      <div
        className="alert alert--success"
        style={{
          marginBottom: '1rem',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <span>✅</span>
          <strong>Chapter personalized successfully!</strong>
          <span style={{ fontSize: '0.875rem', opacity: 0.8 }}>
            Use the toggle above to switch between views.
          </span>
        </div>
        <button
          onClick={() => setShowNotification(null)}
          className="close"
          aria-label="Dismiss"
          style={{
            background: 'none',
            border: 'none',
            fontSize: '1.5rem',
            cursor: 'pointer',
            padding: 0,
            lineHeight: 1,
          }}
        >
          ×
        </button>
      </div>
    );
  };

  /**
   * Render main content
   */
  return (
    <div className="chapter-with-personalization">
      {/* Profile Update Notification (global) */}
      <ProfileUpdateNotification />

      {/* Personalize Button */}
      {!hasPersonalizedContent && (
        <PersonalizeButton
          chapterId={chapterId}
          chapterContent={chapterContent}
          userProfile={userProfile}
          isAuthenticated={isAuthenticated}
          onPersonalizationStart={handlePersonalizationStart}
          onPersonalizationComplete={handlePersonalizationComplete}
          onPersonalizationError={handlePersonalizationError}
        />
      )}

      {/* Success Notification */}
      {renderSuccessNotification()}

      {/* Error Display (if error exists and not handled by PersonalizeButton) */}
      {error && hasPersonalizedContent && (
        <div
          className="alert alert--danger"
          style={{ marginBottom: '1rem' }}
        >
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Personalized Chapter Content */}
      <PersonalizedChapter
        chapterId={chapterId}
        originalContent={chapterContent}
        userName={userName}
      />

      {/* Debug Info (only in development) */}
      {process.env.NODE_ENV === 'development' && (
        <details style={{ marginTop: '2rem', fontSize: '0.875rem', opacity: 0.6 }}>
          <summary>Debug Info (Development Only)</summary>
          <pre style={{ fontSize: '0.75rem', marginTop: '0.5rem' }}>
            {JSON.stringify(
              {
                chapterId,
                isAuthenticated,
                hasUserProfile: !!userProfile,
                hasPersonalizedContent,
                currentView,
                isLoading,
                hasError: !!error,
              },
              null,
              2
            )}
          </pre>
        </details>
      )}
    </div>
  );
}

/**
 * Example: Docusaurus MDX Integration
 *
 * Create a wrapper component in your Docusaurus theme:
 *
 * ```tsx
 * // src/theme/DocItem/Content/index.tsx
 * import React from 'react';
 * import Content from '@theme-original/DocItem/Content';
 * import { ChapterWithPersonalization } from '@site/src/components/ChapterWithPersonalization';
 * import { useAuth } from '@site/src/hooks/useAuth';
 * import { useDoc } from '@docusaurus/theme-common/internal';
 *
 * export default function ContentWrapper(props) {
 *   const { metadata } = useDoc();
 *   const { user, isAuthenticated } = useAuth();
 *
 *   // Check if personalization is enabled for this page
 *   const enablePersonalization = metadata.frontMatter?.personalization !== false;
 *
 *   if (!enablePersonalization) {
 *     return <Content {...props} />;
 *   }
 *
 *   return (
 *     <ChapterWithPersonalization
 *       chapterId={metadata.id}
 *       chapterContent={metadata.source}
 *       userProfile={user?.profile}
 *       isAuthenticated={isAuthenticated}
 *       userName={user?.name}
 *     />
 *   );
 * }
 * ```
 *
 * Then in your markdown files, you can disable personalization if needed:
 *
 * ```markdown
 * ---
 * title: My Chapter
 * personalization: false  # Disable personalization for this page
 * ---
 * ```
 */
