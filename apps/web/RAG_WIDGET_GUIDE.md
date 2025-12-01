"""
RAG Chat Widget - Integration Guide

Overview
========
The RAG (Retrieval-Augmented Generation) Chat Widget is a collapsible chat panel
that provides AI-powered Q&A functionality using retrieved passages from ingested
content as context.

Features
========
✓ Collapsible bottom-right chat panel
✓ Query backend RAG endpoints for source passages
✓ Generate LLM answers using OpenAI/Claude with retrieved context
✓ Show source citations with confidence scores
✓ Keyboard accessible (Escape to close, Enter to send)
✓ Mobile responsive (Tailwind CSS)
✓ Session auto-ingestion of current page content

Files
=====

React Components:
- apps/web/src/components/RagChatWidget.jsx
  * Main React component with full RAG logic
  * Manages message state, API calls, embedding generation
  * Supports custom chapter context

- apps/web/src/components/ChapterRAGCTA.jsx
  * Call-to-action button for chapter-specific queries
  * Injected at top of chapter content
  * Extracts chapter text on click

- apps/web/src/components/RootWithRAGWidget.jsx
  * Root layout wrapper for integration
  * Provides global widget context
  * Manages widget state across navigation

Standalone Script:
- apps/web/src/rag-widget.js
  * Vanilla JavaScript version (no React dependency)
  * Can be embedded in any HTML page
  * Loads via <script src="/rag-widget.js"></script>

Integration Methods
===================

Method 1: React (Recommended for Docusaurus)
------
import RagChatWidget from './components/RagChatWidget';

<RagChatWidget
  chapterTitle="Chapter 1: ROS 2 Basics"
  chapterText={content}
  chapterUrl="/docs/module-1"
/>

Method 2: Standalone JavaScript
--------
Include in HTML <head> or <body>:
<script>
  window.RAG_CONFIG = {
    apiUrl: 'http://localhost:8000'
  };
</script>
<script src="/rag-widget.js"></script>

API Calls Made by Widget
========================

1. Ingest Document
   POST /ingest
   {
     "id": "chapter_123",
     "title": "Chapter Title",
     "text": "Full chapter content..."
   }

2. Query RAG System
   POST /query
   {
     "query": "User question",
     "top_k": 3,
     "context_ids": ["doc_id"] // optional
   }

3. Generate Answer
   POST /chat
   {
     "messages": [
       { "role": "user", "content": "Context: ... Question: ..." }
     ],
     "model": "openai",
     "system_prompt": "You are a helpful assistant..."
   }

Configuration
=============

Backend API URL:
- Default: http://localhost:8000
- Override: window.RAG_CONFIG.apiUrl = 'https://api.example.com'

Environment Variables:
- OPENAI_GEMINI_KEY: OpenAI API key (required for embeddings)
- QDRANT_URL: Qdrant instance URL
- QDRANT_API_KEY: Qdrant API key (if needed)
- NEON_DB_URL: PostgreSQL connection string (if using persistent storage)

Keyboard Shortcuts
==================
- Escape: Close widget
- Enter: Send message (in input field)
- Shift+Enter: New line in input

Mobile Responsiveness
======================
- Bottom-right button on all screen sizes
- Panel width: 100% max-w-md (mobile), max-w-lg (desktop)
- Touch-friendly button sizes (56px height)
- Full-height messages area with scroll

Accessibility
==============
✓ ARIA labels on buttons
✓ Focus management (input autofocus when opened)
✓ Keyboard navigation (Tab, Enter, Escape)
✓ Semantic HTML structure
✓ Color contrast meets WCAG standards
✓ Error messages clearly labeled

Example Usage
=============

Basic HTML page:
```html
<!DOCTYPE html>
<html>
<head>
  <script>
    window.RAG_CONFIG = { apiUrl: 'http://localhost:8000' };
  </script>
</head>
<body>
  <h1>My Documentation</h1>
  <p>Content here...</p>
  
  <script src="/rag-widget.js"></script>
</body>
</html>
```

React/Docusaurus page:
```jsx
import RagChatWidget from '../components/RagChatWidget';

export default function Chapter() {
  return (
    <>
      <h1>Chapter 1</h1>
      <p>Content...</p>
      
      <RagChatWidget
        chapterTitle="Chapter 1: ROS 2"
        chapterText={pageContent}
        chapterUrl="/docs/module-1"
      />
    </>
  );
}
```

Global JavaScript API
=====================

// Open widget
window.__ragWidget.open();

// Close widget
window.__ragWidget.close();

// Toggle widget
window.__ragWidget.toggle();

Testing
=======

1. Build the project:
   yarn web:build
   
2. Start backend:
   yarn backend:start
   
3. Open in browser:
   http://localhost:3000
   
4. Click the blue chat button (bottom-right)

5. Ask a question about the page

6. Verify:
   - Widget appears and is responsive
   - Questions are sent to backend
   - Results show with source citations
   - Sources are clickable links
   - Widget closes on Escape key

Troubleshooting
===============

Widget not appearing?
- Check browser console for errors
- Verify /rag-widget.js is being loaded
- Ensure Tailwind CSS is loaded

No results when querying?
- Verify backend is running (http://localhost:8000/docs)
- Check that page content was ingested successfully
- Try a different query
- Check OPENAI_GEMINI_KEY is set

Slow responses?
- Check network tab in browser DevTools
- Verify OpenAI API is responding
- Check Qdrant/Postgres connectivity

Sources not showing?
- Verify /query endpoint returns results
- Check QueryResult structure in backend
- Look for JavaScript console errors
"""
