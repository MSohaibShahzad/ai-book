import React from 'react';
import ChatWidget from '../components/ChatWidget';
import { TranslationProvider } from '../contexts/TranslationContext';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { ProfileUpdateNotification } from '../components/ProfileUpdateNotification';

/**
 * Root wrapper for Docusaurus app
 *
 * Provides global context providers:
 * - TranslationProvider: For Urdu translation feature
 * - PersonalizationProvider: For chapter personalization feature
 *
 * Also includes global components:
 * - ChatWidget: RAG-powered chatbot
 * - ProfileUpdateNotification: Alerts when profile updates require re-personalization
 */
export default function Root({children}) {
  return (
    <TranslationProvider>
      <PersonalizationProvider>
        {children}
        <ChatWidget />
        <ProfileUpdateNotification />
      </PersonalizationProvider>
    </TranslationProvider>
  );
}
