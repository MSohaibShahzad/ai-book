/**
 * TranslationButton component for Urdu translation feature.
 *
 * Feature: 004-urdu-translation
 * Provides: "Translate to Urdu" button at the start of each chapter
 */

import React, { useState } from 'react';
import styles from './styles.module.css';
import { getJWTToken } from '../../lib/auth-client';

interface TranslationButtonProps {
  chapterSlug: string;
  onTranslate?: (translatedContent: string) => void;
}

export const TranslationButton: React.FC<TranslationButtonProps> = ({
  chapterSlug,
  onTranslate
}) => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(true); // TODO: Get from auth context

  const handleTranslate = async () => {
    setLoading(true);
    setError(null);

    try {
      // Get JWT token for authentication
      const token = await getJWTToken();

      if (!token) {
        setError('Please log in to use Urdu translation.');
        setIsAuthenticated(false);
        setLoading(false);
        return;
      }

      // Use production URL, fallback to localhost only in development
      const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
        ? 'http://localhost:8000/v1'
        : 'https://ai-book-production-6886.up.railway.app/v1';

      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        credentials: 'include', // Include cookies for session-based auth
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

        throw new Error(errorData.message || 'Translation failed');
      }

      const data = await response.json();

      // Call parent callback with translated content
      if (onTranslate) {
        onTranslate(data.translated_content);
      }

      console.log(`[Translation] ${data.from_cache ? 'Cache hit' : 'Generated'}`);

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed. Please try again.';
      setError(errorMessage);
      console.error('[Translation Error]', err);
    } finally {
      setLoading(false);
    }
  };

  // If not authenticated, show disabled button with tooltip
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

export default TranslationButton;
