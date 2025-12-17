# RAG Chatbot Frontend (Chat Widget)

This document describes the chat widget implementation for the Physical-AI textbook.

## Overview

The chat widget is integrated directly into the Docusaurus-based textbook, providing students with AI-powered question answering while they read.

## Architecture

```
Docusaurus Site
├── src/
│   ├── components/
│   │   └── ChatWidget/           # Chat widget component
│   │       ├── index.js          # React component
│   │       └── styles.module.css # Scoped styles
│   └── theme/
│       └── Root.js               # Global integration point
```

## Features

✅ **Floating Chat Button** - Bottom-right corner, always accessible
✅ **Expandable Interface** - Smooth animations, clean UI
✅ **Message History** - Maintains conversation within session
✅ **Source References** - Clickable links to textbook chapters
✅ **Loading States** - Visual feedback during API calls
✅ **Error Handling** - User-friendly error messages
✅ **Responsive Design** - Works on mobile and desktop
✅ **Dark Mode Support** - Adapts to Docusaurus theme

## Component Structure

### ChatWidget (src/components/ChatWidget/index.js)

Main component that handles:
- Chat state management
- API communication
- Message rendering
- User input

**Key State:**
- `isOpen` - Chat window visibility
- `messages` - Conversation history
- `inputText` - Current user input
- `isLoading` - API call in progress
- `sessionId` - Unique session identifier

### Styling (src/components/ChatWidget/styles.module.css)

CSS Modules for:
- Floating button styles
- Chat window layout
- Message bubbles (user/assistant)
- Source reference cards
- Loading animations
- Responsive breakpoints

### Integration (src/theme/Root.js)

Docusaurus theme swizzling to inject chat widget globally across all pages.

## API Integration

### Endpoint

```
POST http://localhost:8000/v1/chat
```

### Request Format

```json
{
  "message": "What is inverse kinematics?",
  "session_id": "session-1234567890",
  "conversation_history": [
    {"role": "user", "content": "Previous question"},
    {"role": "assistant", "content": "Previous answer"}
  ]
}
```

### Response Format

```json
{
  "response": "Inverse kinematics is...",
  "sources": [
    {
      "module_name": "Kinematics",
      "chapter_name": "Inverse Kinematics",
      "slug": "kinematics/inverse-kinematics",
      "url": "/docs/kinematics/inverse-kinematics",
      "preview": "Inverse kinematics refers to..."
    }
  ],
  "session_id": "session-1234567890",
  "retrieval_count": 3,
  "processing_time_ms": 850
}
```

## Configuration

### Environment Variables

Create `.env` file:

```bash
REACT_APP_API_URL=http://localhost:8000/v1
```

For production, update to your deployed backend URL.

### CORS Configuration

Ensure backend allows requests from your frontend origin:

```python
# backend/src/config.py
cors_origins: str = '["http://localhost:3000", "https://yourdomain.com"]'
```

## Development

### Running Locally

```bash
# Start Docusaurus dev server
npm start

# Access at http://localhost:3000
# Chat button appears bottom-right
```

### Hot Reload

Docusaurus automatically reloads when you edit:
- `src/components/ChatWidget/index.js`
- `src/components/ChatWidget/styles.module.css`
- `src/theme/Root.js`

## Customization

### Changing Colors

Edit `styles.module.css`:

```css
/* Chat button gradient */
background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);

/* User message bubble */
background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
```

### Adjusting Position

```css
.chatButton {
  bottom: 30px;  /* Distance from bottom */
  right: 30px;   /* Distance from right */
}
```

### Changing Size

```css
.chatWindow {
  width: 400px;   /* Widget width */
  height: 600px;  /* Widget height */
}
```

## User Experience

### Conversation Flow

1. User clicks floating button (💬)
2. Chat window expands with welcome message
3. User types question and presses Enter (or clicks send)
4. Loading indicator appears (...)
5. AI response displays with source references
6. User can click source links to jump to chapters
7. Conversation continues with context awareness

### Session Management

- **Session ID**: Generated on mount using `Date.now()`
- **Persistence**: None - clears on page refresh
- **History**: Last 3 exchanges (6 messages) sent to backend
- **Storage**: In-memory only (no localStorage/cookies)

### Example Questions

Shown in welcome message:
- "What is inverse kinematics?"
- "Explain the Denavit-Hartenberg convention"
- "How do VLAs work?"

## Accessibility

- **Keyboard Navigation**: Enter to send, Esc to close (future)
- **ARIA Labels**: Chat button has `aria-label="Open chat"`
- **Focus Management**: Auto-focus input when opened
- **Screen Readers**: Compatible with semantic HTML

## Performance

### Bundle Size

- Component: ~8KB gzipped
- Styles: ~2KB gzipped
- Total impact: <10KB added to Docusaurus bundle

### Optimizations

- CSS Modules for scoped styles (no global pollution)
- Lazy rendering (only renders when open)
- Debounced scrolling
- Memoized message rendering (future)

## Browser Support

- Chrome/Edge: ✅ Full support
- Firefox: ✅ Full support
- Safari: ✅ Full support
- Mobile browsers: ✅ Responsive design

## Troubleshooting

### Chat button not appearing

1. Check `src/theme/Root.js` exists and imports ChatWidget
2. Verify no JavaScript errors in browser console
3. Clear browser cache and refresh

### "Failed to fetch" errors

1. Verify backend is running: `curl http://localhost:8000/v1/health`
2. Check CORS configuration in backend `.env`
3. Ensure `REACT_APP_API_URL` in frontend `.env` is correct
4. Check browser console for CORS errors

### No response from chatbot

1. Verify textbook is indexed (backend has data)
2. Check backend logs for errors
3. Test backend directly with curl/Postman
4. Verify OpenAI API key is valid

### Styling conflicts

1. CSS Modules should prevent conflicts
2. Check browser DevTools for z-index issues
3. Verify no Docusaurus plugins overriding styles

## Production Deployment

### Build

```bash
# Build static site
npm run build

# Output in build/ directory
```

### Environment Variables

Update `.env` for production:

```bash
REACT_APP_API_URL=https://api.yourdomain.com/v1
```

### Deployment Checklist

- [ ] Set production `REACT_APP_API_URL`
- [ ] Update backend CORS to include production domain
- [ ] Test on production domain
- [ ] Verify HTTPS (mixed content warnings)
- [ ] Check performance (Lighthouse score)
- [ ] Test on mobile devices

## Future Enhancements

Potential improvements (not yet implemented):

1. **Highlighted Text**: Select text → Ask about it
2. **Typing Indicators**: Real-time streaming
3. **Message Actions**: Copy, regenerate, feedback
4. **Conversation Export**: Download chat history
5. **Voice Input**: Speech-to-text for questions
6. **Keyboard Shortcuts**: Cmd/Ctrl+K to open
7. **Analytics**: Track popular questions
8. **Feedback Loop**: Thumbs up/down on responses

## Related Documentation

- Backend API: `backend/README.md`
- Setup Guide: `QUICKSTART_RAG.md`
- Ingestion: `backend/INGESTION_GUIDE.md`
- Architecture: `specs/002-rag-chatbot/plan.md`

## Support

For issues:
1. Check browser console for errors
2. Review backend logs
3. Verify environment variables
4. Test backend health endpoint
5. Create GitHub issue with details

---

**Built with React 18+ and integrated into Docusaurus 3.x**
