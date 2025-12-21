import React from 'react';
import ChatWidget from '../components/ChatWidget';
import { TranslationProvider } from '../contexts/TranslationContext';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <TranslationProvider>
      {children}
      <ChatWidget />
    </TranslationProvider>
  );
}
