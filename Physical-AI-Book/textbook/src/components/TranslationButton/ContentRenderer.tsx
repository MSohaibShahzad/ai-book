/**
 * ContentRenderer - Switches between original and translated content.
 *
 * Feature: 004-urdu-translation
 * User Story 2: Seamless language toggle
 */

import React from 'react';
import { useTranslationContext } from '../../contexts/TranslationContext';
import ReactMarkdown from 'react-markdown';
import styles from './styles.module.css';

interface ContentRendererProps {
  originalContent?: React.ReactNode;
}

/**
 * Renders either original English or translated Urdu content
 * based on current language state.
 */
export const ContentRenderer: React.FC<ContentRendererProps> = ({ originalContent }) => {
  const { currentLanguage, translatedContent } = useTranslationContext();

  // Show Urdu translation
  if (currentLanguage === 'ur' && translatedContent) {
    return (
      <div className="urdu-content" dir="rtl">
        <ReactMarkdown>{translatedContent}</ReactMarkdown>
      </div>
    );
  }

  // Show original English content
  return (
    <div className="original-content">
      {originalContent}
    </div>
  );
};

export default ContentRenderer;
