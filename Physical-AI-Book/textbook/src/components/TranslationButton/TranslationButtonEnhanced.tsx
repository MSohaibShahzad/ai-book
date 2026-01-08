/**
 * Enhanced TranslationButton with language toggle support.
 *
 * Feature: 004-urdu-translation
 * User Stories: US1 (translation) + US2 (toggle) + US3 (loading feedback)
 */

import React, { useState } from 'react';
import { useTranslationContext } from '../../contexts/TranslationContext';
import { getJWTToken } from '../../lib/auth-client';
import styles from './styles.module.css';

interface TranslationButtonEnhancedProps {
  chapterSlug: string;
}

export const TranslationButtonEnhanced: React.FC<TranslationButtonEnhancedProps> = ({
  chapterSlug
}) => {
  const {
    currentLanguage,
    toggleLanguage,
    setTranslation,
    isCurrentPageTranslated,
  } = useTranslationContext();

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(true); // TODO: Get from auth context

  /**
   * Handle translation request.
   */
  const handleTranslate = async () => {
    setLoading(true);
    setError(null);

    try {
      // Use production URL, fallback to localhost only in development
      const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
        ? 'http://localhost:8000/v1'
        : 'https://ai-book-production-6886.up.railway.app/v1';

      console.log('[Translation] API URL:', apiUrl);
      console.log('[Translation] Chapter slug:', chapterSlug);

      // Get JWT token for authentication
      const token = await getJWTToken();
      if (!token) {
        setError('Please log in to use Urdu translation.');
        setIsAuthenticated(false);
        return;
      }

      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        credentials: 'include',
        body: JSON.stringify({
          chapter_slug: chapterSlug,
          target_language: 'ur'
        })
      });

      if (!response.ok) {
        const errorData = await response.json();

        if (response.status === 401) {
          setError('Please log in to use Urdu translation.');
          setIsAuthenticated(false);
          return;
        }

        if (response.status === 400) {
          throw new Error('Chapter not found.');
        }

        if (response.status === 429) {
          throw new Error('Translation service is busy. Please try again in a moment.');
        }

        throw new Error(errorData.detail || 'Translation failed.');
      }

      const data = await response.json();

      // Store translation in context with chapter slug
      setTranslation(data.translated_content, chapterSlug);

      console.log(`[Translation] ${data.from_cache ? 'Cache hit' : 'Generated'} for ${chapterSlug}`);

    } catch (err) {
      console.error('[Translation Error]', err);
      console.error('[Translation Error] Full error:', JSON.stringify(err, null, 2));

      const errorMessage = err instanceof Error ? err.message : 'Translation failed. Please try again.';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  /**
   * Handle language toggle.
   */
  const handleToggle = () => {
    toggleLanguage();
  };

  // Not authenticated - show disabled button
  if (!isAuthenticated) {
    return (
      <div className={styles.translationControls}>
        <button
          className={styles.translateButton}
          disabled
          title="Please log in to use Urdu translation"
        >
          🌐 Translate to Urdu
        </button>
        <p className={styles.authHint}>Please log in to use Urdu translation</p>
      </div>
    );
  }

  // Check if THIS page is translated - show toggle button only for this page
  const isThisPageTranslated = isCurrentPageTranslated(chapterSlug);

  if (isThisPageTranslated) {
    return (
      <div className={styles.translationControls}>
        {currentLanguage === 'ur' ? (
          <button
            className={styles.toggleButton}
            onClick={handleToggle}
          >
            ← Show Original English
          </button>
        ) : (
          <button
            className={styles.toggleButton}
            onClick={handleToggle}
          >
            → Show Urdu Translation
          </button>
        )}
      </div>
    );
  }

  // No translation yet - show translate button
  return (
    <div className={styles.translationControls}>
      <button
        className={styles.translateButton}
        onClick={handleTranslate}
        disabled={loading}
      >
        {loading ? (
          <>
            <span className={styles.spinner}></span>
            Translating to Urdu...
          </>
        ) : (
          '🌐 Translate to Urdu'
        )}
      </button>

      {error && (
        <div className={styles.errorMessage}>
          ⚠️ {error}
        </div>
      )}
    </div>
  );
};

export default TranslationButtonEnhanced;
