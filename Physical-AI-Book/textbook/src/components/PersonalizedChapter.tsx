/**
 * PersonalizedChapter Component
 *
 * Renders personalized chapter content with badge, loading state, and error handling.
 * Supports toggle between original and personalized views.
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Tasks: T019, T020, T021
 */

import React, { useMemo } from 'react';
import MDXContent from '@theme/MDXContent';
import { usePersonalizationContext, ViewMode } from '../contexts/PersonalizationContext';
import { usePersonalization } from '../hooks/usePersonalization';
import { ViewToggle } from './ViewToggle';
import { RateLimitDisplay } from './RateLimitDisplay';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

/**
 * Props for PersonalizedChapter
 */
export interface PersonalizedChapterProps {
  /**
   * Chapter identifier (e.g., 'foundations-ros2/nodes-topics')
   */
  chapterId: string;

  /**
   * Original chapter content (React children for MDX rendering)
   */
  originalChildren: React.ReactNode;

  /**
   * User's display name (for badge)
   */
  userName?: string;

  /**
   * Callback when user switches views
   */
  onViewChange?: (view: ViewMode) => void;
}

/**
 * PersonalizedChapter Component
 *
 * Displays personalized or original chapter content with:
 * - Badge showing "Personalized for [User Name]"
 * - Loading spinner during generation
 * - Error messages with retry button
 * - Toggle between original and personalized views
 *
 * Usage:
 * ```tsx
 * <PersonalizedChapter
 *   chapterId="foundations-ros2/what-is-ros2"
 *   originalContent={markdownContent}
 *   userName="John Doe"
 * />
 * ```
 */
export function PersonalizedChapter({
  chapterId,
  originalChildren,
  userName,
  onViewChange,
}: PersonalizedChapterProps) {
  const { currentView, rateLimitRemaining, rateLimitResetAt } = usePersonalizationContext();
  const { isLoading, error, getPersonalizedContent, clearError } = usePersonalization();

  // Get personalized content from cache
  const personalizedMarkdown = getPersonalizedContent(chapterId);
  const hasPersonalizedContent = !!personalizedMarkdown;

  /**
   * Render error message with retry
   */
  if (error) {
    return (
      <div className="personalized-chapter-error" style={{ marginBottom: '2rem' }}>
        <div className="alert alert--danger">
          <h4>Personalization Error</h4>
          <p>{error}</p>
          <button
            onClick={clearError}
            className="button button--secondary button--sm"
            style={{ marginTop: '0.5rem' }}
          >
            Dismiss
          </button>
        </div>
        {hasPersonalizedContent && (
          <div className="alert alert--info" style={{ marginTop: '1rem' }}>
            You can still view your previously personalized version using the toggle below.
          </div>
        )}
      </div>
    );
  }

  /**
   * Render loading state
   */
  if (isLoading) {
    return (
      <div
        className="personalized-chapter-loading"
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          padding: '3rem 1rem',
          textAlign: 'center',
        }}
      >
        <div
          className="spinner-border"
          style={{ width: '3rem', height: '3rem', marginBottom: '1rem' }}
          role="status"
        >
          <span className="visually-hidden">Loading...</span>
        </div>
        <h3 style={{ marginBottom: '0.5rem' }}>Generating personalized content...</h3>
        <p style={{ color: 'var(--ifm-color-emphasis-600)', maxWidth: '500px' }}>
          Our AI is adapting this chapter to your background and interests. This may take up to 30
          seconds.
        </p>
      </div>
    );
  }

  return (
    <div className="personalized-chapter">
      {/* Rate Limit Display (show if user has used personalizations) */}
      {rateLimitRemaining < 3 && rateLimitRemaining >= 0 && (
        <RateLimitDisplay
          resetAt={rateLimitResetAt ?? undefined}
          compact={true}
        />
      )}

      {/* Toggle between original and personalized views */}
      <ViewToggle
        userName={userName}
        hasPersonalizedContent={hasPersonalizedContent}
        onViewChange={onViewChange}
      />

      {/* Render chapter content based on current view */}
      {currentView === 'personalized' && personalizedMarkdown ? (
        // Personalized view: Render markdown from API with Docusaurus styling
        <div className="personalized-chapter-content theme-doc-markdown markdown">
          <ReactMarkdown
            remarkPlugins={[remarkGfm, remarkMath]}
            rehypePlugins={[rehypeKatex]}
            components={{
              // Apply Docusaurus h1 styling with gradient and border
              h1({ children, ...props }) {
                return <h1 {...props}>{children}</h1>;
              },
              // Apply Docusaurus h2 styling with cyan color and left border
              h2({ children, ...props }) {
                return <h2 {...props}>{children}</h2>;
              },
              // Apply Docusaurus h3 styling with purple color
              h3({ children, ...props }) {
                return <h3 {...props}>{children}</h3>;
              },
              // Apply Docusaurus h4, h5, h6 styling
              h4({ children, ...props }) {
                return <h4 {...props}>{children}</h4>;
              },
              h5({ children, ...props }) {
                return <h5 {...props}>{children}</h5>;
              },
              h6({ children, ...props }) {
                return <h6 {...props}>{children}</h6>;
              },
              // Use Docusaurus styling for code blocks
              code({ node, inline, className, children, ...props }) {
                const match = /language-(\w+)/.exec(className || '');
                return !inline && match ? (
                  <pre className={className}>
                    <code className={className} {...props}>
                      {children}
                    </code>
                  </pre>
                ) : (
                  <code className={className} {...props}>
                    {children}
                  </code>
                );
              },
              // Use Docusaurus styling for tables
              table({ children, ...props }) {
                return (
                  <div className="table-wrapper">
                    <table {...props}>{children}</table>
                  </div>
                );
              },
              // Use Docusaurus styling for links
              a({ href, children, ...props }) {
                const isExternal = href?.startsWith('http');
                return (
                  <a
                    href={href}
                    target={isExternal ? '_blank' : undefined}
                    rel={isExternal ? 'noopener noreferrer' : undefined}
                    {...props}
                  >
                    {children}
                  </a>
                );
              },
            }}
          >
            {personalizedMarkdown}
          </ReactMarkdown>
        </div>
      ) : (
        // Original view: Render actual MDX children
        <div className="chapter-content-original">
          {originalChildren}
        </div>
      )}
    </div>
  );
}
