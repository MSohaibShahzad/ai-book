/**
 * Custom React hook for translation API calls.
 *
 * Feature: 004-urdu-translation
 * Provides: Translation state management and API integration
 */

import { useState, useCallback } from 'react';

interface TranslationResponse {
  status: string;
  translated_content: string;
  from_cache: boolean;
  cached_at: string | null;
  message?: string;
}

interface TranslationState {
  translatedContent: string | null;
  loading: boolean;
  error: string | null;
  fromCache: boolean;
}

interface UseTranslationReturn extends TranslationState {
  translateChapter: (chapterSlug: string) => Promise<void>;
  resetTranslation: () => void;
}

/**
 * Hook for managing translation state and API calls.
 *
 * @returns Translation state and control functions
 */
export const useTranslation = (): UseTranslationReturn => {
  const [state, setState] = useState<TranslationState>({
    translatedContent: null,
    loading: false,
    error: null,
    fromCache: false,
  });

  /**
   * Translate a chapter to Urdu.
   */
  const translateChapter = useCallback(async (chapterSlug: string) => {
    setState(prev => ({ ...prev, loading: true, error: null }));

    try {
      // Get API URL from Docusaurus custom fields
      const apiUrl = (window as any).docusaurus?.siteConfig?.customFields?.apiUrl ||
                     'http://localhost:8000/v1';

      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include session cookies
        body: JSON.stringify({
          chapter_slug: chapterSlug,
          target_language: 'ur'
        })
      });

      if (!response.ok) {
        const errorData = await response.json();

        if (response.status === 401) {
          throw new Error('Please log in to use Urdu translation.');
        }

        if (response.status === 400) {
          throw new Error(errorData.detail || 'Chapter not found.');
        }

        if (response.status === 429) {
          throw new Error('Translation service is busy. Please try again in a moment.');
        }

        throw new Error(errorData.detail || 'Translation failed. Please try again.');
      }

      const data: TranslationResponse = await response.json();

      setState({
        translatedContent: data.translated_content,
        loading: false,
        error: null,
        fromCache: data.from_cache,
      });

      // Log analytics for translation request
      logTranslationAction(chapterSlug, 'translate_requested');

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed. Please try again.';

      setState({
        translatedContent: null,
        loading: false,
        error: errorMessage,
        fromCache: false,
      });

      console.error('[Translation Error]', err);
    }
  }, []);

  /**
   * Reset translation state.
   */
  const resetTranslation = useCallback(() => {
    setState({
      translatedContent: null,
      loading: false,
      error: null,
      fromCache: false,
    });
  }, []);

  return {
    ...state,
    translateChapter,
    resetTranslation,
  };
};

/**
 * Log translation action for analytics.
 */
async function logTranslationAction(
  chapterSlug: string,
  action: 'translate_requested' | 'toggle_to_english' | 'toggle_to_urdu'
): Promise<void> {
  try {
    const apiUrl = (window as any).docusaurus?.siteConfig?.customFields?.apiUrl ||
                   'http://localhost:8000/v1';

    await fetch(`${apiUrl}/api/translate/log`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({
        chapter_slug: chapterSlug,
        action: action
      })
    });
  } catch (err) {
    // Don't throw on logging errors
    console.warn('[Analytics] Failed to log action:', err);
  }
}

export default useTranslation;
