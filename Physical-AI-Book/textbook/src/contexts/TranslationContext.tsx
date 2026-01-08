/**
 * Translation Context for managing language state globally.
 *
 * Feature: 004-urdu-translation
 * User Story 2: Toggle between English and Urdu
 */

import React, { createContext, useState, useContext, useCallback, ReactNode } from 'react';

type Language = 'en' | 'ur';

interface TranslationContextType {
  currentLanguage: Language;
  translatedContent: string | null;
  currentChapterSlug: string | null;
  toggleLanguage: () => void;
  setTranslation: (content: string, chapterSlug: string) => void;
  clearTranslation: () => void;
  isTranslated: boolean;
  isCurrentPageTranslated: (chapterSlug: string) => boolean;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

interface TranslationProviderProps {
  children: ReactNode;
}

/**
 * Provider component for translation context.
 * Manages language state and translated content at the app level.
 */
export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  const [currentLanguage, setCurrentLanguage] = useState<Language>('en');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [currentChapterSlug, setCurrentChapterSlug] = useState<string | null>(null);

  /**
   * Toggle between English and Urdu.
   * Logs analytics for toggle actions.
   */
  const toggleLanguage = useCallback(() => {
    const newLanguage = currentLanguage === 'en' ? 'ur' : 'en';
    setCurrentLanguage(newLanguage);

    // Log analytics
    const action = newLanguage === 'ur' ? 'toggle_to_urdu' : 'toggle_to_english';
    logToggleAction(action);

    console.log(`[Translation] Language toggled to: ${newLanguage}`);
  }, [currentLanguage]);

  /**
   * Set translated content and switch to Urdu.
   * Associates the translation with a specific chapter.
   */
  const setTranslation = useCallback((content: string, chapterSlug: string) => {
    setTranslatedContent(content);
    setCurrentChapterSlug(chapterSlug);
    setCurrentLanguage('ur');
    console.log(`[Translation] Content set for chapter: ${chapterSlug}`);
  }, []);

  /**
   * Clear translation and return to English.
   */
  const clearTranslation = useCallback(() => {
    setTranslatedContent(null);
    setCurrentChapterSlug(null);
    setCurrentLanguage('en');
  }, []);

  /**
   * Check if the current page has been translated.
   */
  const isCurrentPageTranslated = useCallback((chapterSlug: string): boolean => {
    return translatedContent !== null && currentChapterSlug === chapterSlug;
  }, [translatedContent, currentChapterSlug]);

  const isTranslated = translatedContent !== null;

  const value: TranslationContextType = {
    currentLanguage,
    translatedContent,
    currentChapterSlug,
    toggleLanguage,
    setTranslation,
    clearTranslation,
    isTranslated,
    isCurrentPageTranslated,
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

/**
 * Hook to access translation context.
 * Must be used within TranslationProvider.
 */
export const useTranslationContext = (): TranslationContextType => {
  const context = useContext(TranslationContext);

  if (!context) {
    throw new Error('useTranslationContext must be used within TranslationProvider');
  }

  return context;
};

/**
 * Log toggle action for analytics.
 */
async function logToggleAction(
  action: 'toggle_to_english' | 'toggle_to_urdu'
): Promise<void> {
  try {
    // Import getJWTToken dynamically to avoid circular dependencies
    const { getJWTToken } = await import('../lib/auth-client');

    // Use production URL, fallback to localhost only in development
    const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
      ? 'http://localhost:8000/v1'
      : 'https://ai-book-production-6886.up.railway.app/v1';

    // Get current chapter slug from URL
    const pathname = window.location.pathname;
    const match = pathname.match(/\/docs\/(.+)/);
    const chapterSlug = match ? match[1].replace(/\//g, '-') : 'unknown';

    // Get JWT token for authentication
    const token = await getJWTToken();
    if (!token) {
      console.warn('[Analytics] No auth token available, skipping log');
      return;
    }

    await fetch(`${apiUrl}/api/translate/log`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      credentials: 'include',
      body: JSON.stringify({
        chapter_slug: chapterSlug,
        action: action
      })
    });
  } catch (err) {
    console.warn('[Analytics] Failed to log toggle action:', err);
  }
}

export default TranslationContext;
