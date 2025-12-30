import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

// Use window object to avoid SSR issues
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'https://ai-book-production-6886.up.railway.app/v1';
  return window.location.hostname === 'localhost'
    ? 'http://localhost:8000/v1'
    : 'https://ai-book-production-6886.up.railway.app/v1';
};

const API_URL = getApiUrl();

// Generate a valid UUID v4
function generateUUID() {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    const r = Math.random() * 16 | 0;
    const v = c === 'x' ? r : (r & 0x3 | 0x8);
    return v.toString(16);
  });
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => generateUUID());
  const [highlightedText, setHighlightedText] = useState('');
  const [showAskAIButton, setShowAskAIButton] = useState(false);
  const [askAIPosition, setAskAIPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Detect text selection on the page
  useEffect(() => {
    const handleSelection = (e) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // Only capture if text is selected
      if (text && text.length > 0 && text.length <= 5000) {
        setHighlightedText(text);

        // Get selection position to show "Ask AI" button
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Position button below the selection
        setAskAIPosition({
          x: rect.left + rect.width / 2,
          y: rect.bottom + window.scrollY + 5
        });
        setShowAskAIButton(true);
      } else {
        setShowAskAIButton(false);
      }
    };

    // Hide button when clicking elsewhere
    const handleClickAway = (e) => {
      if (!e.target.closest('.askAIButton')) {
        setTimeout(() => setShowAskAIButton(false), 100);
      }
    };

    // Listen for mouseup (after text selection)
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('mousedown', handleClickAway);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('mousedown', handleClickAway);
    };
  }, []);

  const sendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    const userMessage = inputText.trim();
    setInputText('');

    // Add user message to chat
    const newMessages = [...messages, { role: 'user', content: userMessage }];
    setMessages(newMessages);
    setIsLoading(true);

    try {
      // Get conversation history (last 3 exchanges)
      const conversationHistory = newMessages.slice(-6).map(msg => ({
        role: msg.role,
        content: msg.content
      }));

      // Call API
      const response = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage,
          session_id: sessionId,
          conversation_history: conversationHistory.slice(0, -1), // Exclude current message
          highlighted_text: highlightedText || null, // Include highlighted text if available
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response with sources
      setMessages([
        ...newMessages,
        {
          role: 'assistant',
          content: data.response,
          sources: data.sources || [],
          processingTime: data.processing_time_ms,
        },
      ]);
    } catch (error) {
      console.error('Chat error:', error);
      setMessages([
        ...newMessages,
        {
          role: 'assistant',
          content: `Sorry, I encountered an error: ${error.message}. Please make sure the backend is running.`,
          sources: [],
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleAskAI = async () => {
    setIsOpen(true);
    setShowAskAIButton(false);

    // Auto-send explanation request for the selected text
    if (highlightedText.trim()) {
      const explanationPrompt = `Please explain this text: "${highlightedText}"`;

      // Add user message to chat
      const newMessages = [...messages, { role: 'user', content: explanationPrompt }];
      setMessages(newMessages);
      setIsLoading(true);

      try {
        // Get conversation history (last 3 exchanges)
        const conversationHistory = newMessages.slice(-6).map(msg => ({
          role: msg.role,
          content: msg.content
        }));

        // Call API
        const response = await fetch(`${API_URL}/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: explanationPrompt,
            session_id: sessionId,
            conversation_history: conversationHistory.slice(0, -1), // Exclude current message
            highlighted_text: highlightedText, // Include highlighted text
          }),
        });

        if (!response.ok) {
          throw new Error(`API error: ${response.status}`);
        }

        const data = await response.json();

        // Add assistant response with sources
        setMessages([
          ...newMessages,
          {
            role: 'assistant',
            content: data.response,
            sources: data.sources || [],
            processingTime: data.processing_time_ms,
          },
        ]);
      } catch (error) {
        console.error('Chat error:', error);
        setMessages([
          ...newMessages,
          {
            role: 'assistant',
            content: `Sorry, I encountered an error: ${error.message}. Please make sure the backend is running.`,
            sources: [],
          },
        ]);
      } finally {
        setIsLoading(false);
      }
    }
  };

  return (
    <>
      {/* Ask AI Button - appears when text is selected */}
      {showAskAIButton && !isOpen && (
        <button
          className="askAIButton"
          style={{
            position: 'absolute',
            left: `${askAIPosition.x}px`,
            top: `${askAIPosition.y}px`,
            transform: 'translateX(-50%)',
            zIndex: 1001,
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            color: 'white',
            border: 'none',
            padding: '8px 16px',
            borderRadius: '20px',
            fontSize: '13px',
            fontWeight: '600',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(102, 126, 234, 0.4)',
            display: 'flex',
            alignItems: 'center',
            gap: '6px',
            transition: 'all 0.2s ease',
          }}
          onClick={handleAskAI}
          onMouseEnter={(e) => e.target.style.transform = 'translateX(-50%) scale(1.05)'}
          onMouseLeave={(e) => e.target.style.transform = 'translateX(-50%) scale(1)'}
        >
          <span>✨</span>
          Ask AI
        </button>
      )}

      {/* Floating Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>Physical-AI Tutor</h3>
            <p>Ask questions about the textbook</p>
          </div>

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Welcome! Ask me anything about Physical-AI and Robotics.</p>
                <p className={styles.exampleQuestions}>
                  Example questions:
                  <br />• What is inverse kinematics?
                  <br />• Explain the Denavit-Hartenberg convention
                  <br />• How do VLAs work?
                </p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={styles[`message-${msg.role}`]}>
                <div className={styles.messageContent}>
                  {msg.role === 'user' ? (
                    <p>{msg.content}</p>
                  ) : (
                    <>
                      <div className={styles.assistantText}>
                        {msg.content}
                      </div>

                      {/* Sources */}
                      {msg.sources && msg.sources.length > 0 && (
                        <div className={styles.sources}>
                          <p className={styles.sourcesTitle}>📚 Sources:</p>
                          {msg.sources.map((source, sidx) => (
                            <a
                              key={sidx}
                              href={source.url}
                              className={styles.sourceLink}
                              target="_blank"
                              rel="noopener noreferrer"
                            >
                              {source.module_name} → {source.chapter_name}
                            </a>
                          ))}
                        </div>
                      )}

                      {/* Processing time */}
                      {msg.processingTime && (
                        <div className={styles.metadata}>
                          <small>Response time: {msg.processingTime}ms</small>
                        </div>
                      )}
                    </>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={styles['message-assistant']}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingDots}>
                    <span>.</span>
                    <span>.</span>
                    <span>.</span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Highlighted Text Display */}
          {highlightedText && (
            <div className={styles.highlightedTextBox}>
              <div className={styles.highlightedTextHeader}>
                <span>📋 Selected text:</span>
                <button
                  onClick={() => setHighlightedText('')}
                  className={styles.clearButton}
                  title="Clear selection"
                >
                  ✕
                </button>
              </div>
              <div className={styles.highlightedTextContent}>
                {highlightedText.length > 150
                  ? highlightedText.substring(0, 150) + '...'
                  : highlightedText}
              </div>
            </div>
          )}

          {/* Input */}
          <div className={styles.chatInput}>
            <textarea
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={highlightedText ? "Ask about the selected text..." : "Ask a question..."}
              rows={2}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={!inputText.trim() || isLoading}
              className={styles.sendButton}
            >
              {isLoading ? '...' : '→'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}
