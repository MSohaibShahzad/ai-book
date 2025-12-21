/**
 * Swizzled DocItem/Content component with Urdu translation support.
 *
 * Feature: 004-urdu-translation
 * This wraps the document content to enable switching between English and Urdu.
 */

import React, { type ReactNode, useEffect } from 'react';
import clsx from 'clsx';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import Heading from '@theme/Heading';
import MDXContent from '@theme/MDXContent';
import type { Props } from '@theme/DocItem/Content';
import { useTranslationContext } from '@site/src/contexts/TranslationContext';
import ReactMarkdown from 'react-markdown';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

/**
 * Title can be declared inside md content or declared through
 * front matter and added manually. To make both cases consistent,
 * the added title is added under the same div.markdown block
 *
 * We render a "synthetic title" if:
 * - user doesn't ask to hide it with front matter
 * - the markdown content does not already contain a top-level h1 heading
 */
function useSyntheticTitle(): string | null {
  const { metadata, frontMatter, contentTitle } = useDoc();
  const shouldRender =
    !frontMatter.hide_title && typeof contentTitle === 'undefined';
  if (!shouldRender) {
    return null;
  }
  return metadata.title;
}

/**
 * Strip YAML frontmatter from markdown content.
 * Frontmatter is enclosed between --- markers at the start of the document.
 */
function stripFrontmatter(markdown: string): string {
  // Match YAML frontmatter pattern: starts with ---, ends with ---
  const frontmatterRegex = /^---\s*\n[\s\S]*?\n---\s*\n/;
  return markdown.replace(frontmatterRegex, '').trim();
}

export default function DocItemContent({ children }: Props): ReactNode {
  const syntheticTitle = useSyntheticTitle();
  const { metadata } = useDoc();
  const { currentLanguage, translatedContent, isCurrentPageTranslated, toggleLanguage } = useTranslationContext();

  // Get the current page's chapter slug
  const chapterSlug = (metadata.permalink || '')
    .replace('/docs/', '')
    .replace(/^\//, '')
    .replace(/\/$/, '');

  // Reset to English when navigating to an untranslated page while in Urdu mode
  useEffect(() => {
    if (currentLanguage === 'ur' && !isCurrentPageTranslated(chapterSlug)) {
      toggleLanguage(); // Switch back to English
      console.log(`[Translation] Switched to English - page "${chapterSlug}" not translated`);
    }
  }, [chapterSlug, currentLanguage, isCurrentPageTranslated, toggleLanguage]);

  // Show Urdu translation ONLY if it matches the current page AND language is set to Urdu
  const shouldShowTranslation = currentLanguage === 'ur' && isCurrentPageTranslated(chapterSlug);

  if (shouldShowTranslation && translatedContent) {
    // Strip frontmatter from translated content before rendering
    const contentWithoutFrontmatter = stripFrontmatter(translatedContent);

    return (
      <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')} dir="rtl">
        <ReactMarkdown
          remarkPlugins={[remarkMath]}
          rehypePlugins={[rehypeKatex]}
        >
          {contentWithoutFrontmatter}
        </ReactMarkdown>
      </div>
    );
  }

  // Show original English content
  return (
    <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
      {syntheticTitle && (
        <header>
          <Heading as="h1">{syntheticTitle}</Heading>
        </header>
      )}
      <MDXContent>{children}</MDXContent>
    </div>
  );
}
