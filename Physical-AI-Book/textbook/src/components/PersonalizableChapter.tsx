/**
 * PersonalizableChapter Component
 *
 * High-level wrapper for chapters that want to support personalization.
 * Automatically handles authentication state, profile fetching, and UI integration.
 *
 * Usage in MDX files:
 * ```mdx
 * import PersonalizableChapter from '@site/src/components/PersonalizableChapter';
 *
 * <PersonalizableChapter chapterId="foundations-ros2/what-is-ros2">
 *
 * # Your chapter content here
 *
 * </PersonalizableChapter>
 * ```
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Integration Component
 */

import React, { ReactNode, useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { PersonalizeButton } from './PersonalizeButton';
import { PersonalizedChapter } from './PersonalizedChapter';
import { usePersonalizationContext } from '../contexts/PersonalizationContext';

/**
 * Props for PersonalizableChapter
 */
export interface PersonalizableChapterProps {
  /**
   * Chapter identifier (e.g., 'foundations-ros2/what-is-ros2')
   * Should match the chapter's slug in the docs directory
   */
  chapterId: string;

  /**
   * Chapter content (MDX children)
   */
  children: ReactNode;
}

/**
 * Fetch raw markdown content from source files
 *
 * Tries multiple possible file paths and extensions
 */
async function fetchRawMarkdown(chapterId: string): Promise<string> {
  // Possible file extensions
  const extensions = ['.md', '.mdx'];

  // Possible base paths
  const basePaths = [
    `/docs/${chapterId}`,
    `/${chapterId}`,
  ];

  // Try all combinations
  for (const basePath of basePaths) {
    for (const ext of extensions) {
      try {
        const url = `${basePath}${ext}`;
        console.log(`[PersonalizableChapter] Trying to fetch: ${url}`);

        const response = await fetch(url);
        if (response.ok) {
          const content = await response.text();

          // Check if we got HTML instead of markdown (Docusaurus serves HTML for .md paths)
          if (content.trim().startsWith('<!DOCTYPE') || content.trim().startsWith('<html')) {
            console.log(`[PersonalizableChapter] ${url} returned HTML, not markdown - skipping`);
            continue; // Try next path
          }

          console.log(`[PersonalizableChapter] Successfully fetched ${url} (${content.length} chars)`);
          return content;
        }
      } catch (error) {
        // Continue to next path
        console.log(`[PersonalizableChapter] Failed to fetch ${basePath}${ext}`);
      }
    }
  }

  // Fallback: extract markdown-like content from DOM
  console.warn('[PersonalizableChapter] Could not fetch raw markdown files, extracting from DOM');

  // Try to get the article element - be very specific to avoid getting the whole page
  const article = document.querySelector('article[class*="markdown"]') ||
                   document.querySelector('main article') ||
                   document.querySelector('.markdown article') ||
                   document.querySelector('article');

  if (!article) {
    console.error('[PersonalizableChapter] Could not find article element');
    return '';
  }

  console.log(`[PersonalizableChapter] Found article element:`, article.className);

  // Extract text content with basic markdown structure preservation
  let content = '';

  // Get title from h1 within article only
  const h1 = article.querySelector('h1');
  if (h1) {
    content += `# ${h1.textContent.trim()}\n\n`;
  }

  // Get all paragraphs, headings, lists from article (allow nested elements)
  // Exclude navigation, table of contents, and other non-content elements
  const elements = article.querySelectorAll('h1, h2, h3, h4, h5, h6, p, ul, ol, blockquote');

  // Filter out elements that are inside nav, aside, or other non-content containers
  const contentElements = Array.from(elements).filter((el) => {
    // Check if element is inside a nav or aside
    const isInNav = el.closest('nav') !== null;
    const isInAside = el.closest('aside') !== null;
    const isInTOC = el.closest('.table-of-contents') !== null;

    return !isInNav && !isInAside && !isInTOC;
  });

  console.log(`[PersonalizableChapter] Found ${contentElements.length} content elements (filtered from ${elements.length} total)`);

  contentElements.forEach((el) => {
    const tag = el.tagName.toLowerCase();

    if (tag === 'h1') {
      // Skip if already added
      if (el === h1) return;
      content += `# ${el.textContent}\n\n`;
    } else if (tag === 'h2') {
      content += `## ${el.textContent}\n\n`;
    } else if (tag === 'h3') {
      content += `### ${el.textContent}\n\n`;
    } else if (tag === 'h4') {
      content += `#### ${el.textContent}\n\n`;
    } else if (tag === 'h5') {
      content += `##### ${el.textContent}\n\n`;
    } else if (tag === 'h6') {
      content += `###### ${el.textContent}\n\n`;
    } else if (tag === 'p') {
      content += `${el.textContent}\n\n`;
    } else if (tag === 'ul' || tag === 'ol') {
      const items = el.querySelectorAll('li');
      items.forEach((li) => {
        content += `- ${li.textContent}\n`;
      });
      content += '\n';
    } else if (tag === 'blockquote') {
      content += `> ${el.textContent}\n\n`;
    }
  });

  console.log(`[PersonalizableChapter] Extracted ${content.length} chars from DOM`);
  return content.trim();
}

/**
 * PersonalizableChapter Component
 *
 * Wraps chapter content with personalization capabilities.
 * Handles all the integration complexity automatically.
 *
 * Usage:
 * ```tsx
 * <PersonalizableChapter chapterId="foundations-ros2/what-is-ros2">
 *   <h1>What is ROS 2?</h1>
 *   <p>Your chapter content here...</p>
 * </PersonalizableChapter>
 * ```
 */
export function PersonalizableChapter({
  chapterId,
  children,
}: PersonalizableChapterProps) {
  const { isAuthenticated, userName, userProfile, isLoading: authLoading } = useAuth();
  const { getPersonalizedContent } = usePersonalizationContext();

  // State for chapter markdown content
  const [chapterContent, setChapterContent] = useState<string>('');
  const [contentLoading, setContentLoading] = useState<boolean>(true);

  // Fetch raw markdown content on mount
  useEffect(() => {
    let isMounted = true;

    async function loadContent() {
      try {
        console.log(`[PersonalizableChapter] Loading content for: ${chapterId}`);
        const content = await fetchRawMarkdown(chapterId);

        if (isMounted) {
          console.log(`[PersonalizableChapter] Content loaded: ${content.length} chars`);
          setChapterContent(content);
          setContentLoading(false);
        }
      } catch (error) {
        console.error('[PersonalizableChapter] Error loading content:', error);
        if (isMounted) {
          setContentLoading(false);
        }
      }
    }

    loadContent();

    return () => {
      isMounted = false;
    };
  }, [chapterId]);

  // Check if personalized content exists
  const personalizedContent = getPersonalizedContent(chapterId);
  const hasPersonalizedContent = !!personalizedContent;

  return (
    <div className="personalizable-chapter">
      {/* Personalize Button (only show if no personalized content exists and content is loaded) */}
      {!authLoading && !contentLoading && !hasPersonalizedContent && chapterContent && (
        <PersonalizeButton
          chapterId={chapterId}
          chapterContent={chapterContent}
          userProfile={userProfile}
          isAuthenticated={isAuthenticated}
        />
      )}

      {/* Chapter Content */}
      {hasPersonalizedContent ? (
        // PersonalizedChapter handles view switching internally (original vs personalized)
        <PersonalizedChapter
          chapterId={chapterId}
          originalChildren={children}
          userName={userName ?? undefined}
        />
      ) : (
        // No personalized content yet, show original
        <div className="chapter-content-original">
          {children}
        </div>
      )}
    </div>
  );
}

export default PersonalizableChapter;
