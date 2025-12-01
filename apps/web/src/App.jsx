/**
 * App.jsx - Root application component with RAG widget integration
 * 
 * This serves as the main entry point and wraps all content with RAG functionality.
 */

import React from 'react';
import { RagChatWidget } from './components/RagChatWidget';
import { ChapterRAGCTA } from './components/ChapterRAGCTA';

export function App({ children }: { children?: React.ReactNode }) {
  const [isWidgetOpen, setIsWidgetOpen] = React.useState(false);
  const [chapterContext, setChapterContext] = React.useState({
    title: 'Chapter',
    text: '',
    url: window.location.pathname
  });

  const handleOpenWidget = React.useCallback(() => {
    // Extract chapter content
    const mainContent = document.querySelector('article') || document.querySelector('main');
    const title = document.querySelector('h1')?.textContent || 'Chapter';
    const text = mainContent?.textContent?.slice(0, 5000) || '';

    setChapterContext({
      title: title.trim(),
      text: text.trim(),
      url: window.location.pathname
    });

    setIsWidgetOpen(true);
  }, []);

  // Expose to window for global access
  React.useEffect(() => {
    (window as any).__ragWidget = {
      open: handleOpenWidget,
      close: () => setIsWidgetOpen(false),
      toggle: () => setIsWidgetOpen(!isWidgetOpen)
    };
  }, [handleOpenWidget]);

  return (
    <>
      {children}
      
      {/* RAG Chat Widget - Bottom Right */}
      <RagChatWidget
        isOpen={isWidgetOpen}
        onOpenChange={setIsWidgetOpen}
        chapterTitle={chapterContext.title}
        chapterText={chapterContext.text}
        chapterUrl={chapterContext.url}
      />

      {/* Chapter CTA - Can be injected into articles */}
      <style>{`
        .rag-chapter-cta {
          display: block;
        }
      `}</style>
    </>
  );
}

export default App;
