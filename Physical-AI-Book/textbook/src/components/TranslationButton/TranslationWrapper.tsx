/**
 * TranslationWrapper component - wraps chapter content with translation capability.
 *
 * Feature: 004-urdu-translation
 * Usage: Can be imported in MDX files to add translation button
 */

import React, { useState } from 'react';
import TranslationButton from './index';
import styles from './styles.module.css';

interface TranslationWrapperProps {
  chapterSlug: string;
  children?: React.ReactNode;
}

export const TranslationWrapper: React.FC<TranslationWrapperProps> = ({
  chapterSlug,
  children
}) => {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showUrdu, setShowUrdu] = useState(false);

  const handleTranslate = (content: string) => {
    setTranslatedContent(content);
    setShowUrdu(true);
  };

  const handleToggleLanguage = () => {
    setShowUrdu(!showUrdu);
    // TODO: Log analytics for toggle action
  };

  return (
    <div className={styles.translationWrapper}>
      <TranslationButton
        chapterSlug={chapterSlug}
        onTranslate={handleTranslate}
      />

      {translatedContent && showUrdu && (
        <div>
          <button
            className={styles.toggleButton}
            onClick={handleToggleLanguage}
          >
            ← Show Original English
          </button>

          <div
            className={styles.urduContent}
            dir="rtl"
            dangerouslySetInnerHTML={{ __html: convertMarkdownToHtml(translatedContent) }}
          />
        </div>
      )}

      {(!translatedContent || !showUrdu) && (
        <div className={styles.originalContent}>
          {translatedContent && !showUrdu && (
            <button
              className={styles.toggleButton}
              onClick={handleToggleLanguage}
            >
              → Show Urdu Translation
            </button>
          )}
          {children}
        </div>
      )}
    </div>
  );
};

/**
 * Simple markdown to HTML converter for Urdu content.
 * In production, use a proper markdown renderer.
 */
function convertMarkdownToHtml(markdown: string): string {
  // Basic markdown conversion (for MVP)
  // TODO: Use proper markdown renderer (react-markdown or similar)
  return markdown
    .replace(/^### (.*$)/gim, '<h3>$1</h3>')
    .replace(/^## (.*$)/gim, '<h2>$1</h2>')
    .replace(/^# (.*$)/gim, '<h1>$1</h1>')
    .replace(/\*\*(.*)\*\*/gim, '<strong>$1</strong>')
    .replace(/\*(.*)\*/gim, '<em>$1</em>')
    .replace(/\n/gim, '<br />');
}

export default TranslationWrapper;
