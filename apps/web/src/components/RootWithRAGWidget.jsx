import React, { useState, useRef, useCallback } from 'react';
import RagChatWidget from './RagChatWidget';
import ChapterRAGCTA from './ChapterRAGCTA';

/**
 * RootWithRAGWidget - Root layout wrapper that provides RAG widget context
 * 
 * This component wraps the entire Docusaurus site and:
 * 1. Provides a persistent RagChatWidget in bottom-right
 * 2. Exposes ref for opening widget from chapter CTAs
 * 3. Manages widget state across page navigation
 * 4. Extracts chapter metadata from current page
 */

interface RootWithRAGWidgetProps {
  children?: React.ReactNode;
}

export const RootWithRAGWidget: React.FC<RootWithRAGWidgetProps> = ({ children }) => {
  const [isWidgetOpen, setIsWidgetOpen] = useState(false);
  const [currentChapter, setCurrentChapter] = useState({
    title: 'Chapter',
    text: '',
    url: ''
  });
  const widgetRef = useRef<any>(null);

  /**
   * Extract chapter metadata from current page
   */
  const extractChapterMetadata = useCallback(() => {
    // Try to get title from h1 or document title
    const h1 = document.querySelector('article h1') || document.querySelector('main h1');
    const title = h1?.textContent || document.title || 'Chapter';

    // Extract text from main article content
    const article = document.querySelector('article') || document.querySelector('main');
    const text = article?.textContent?.slice(0, 3000) || ''; // Limit to 3000 chars

    // Get current URL
    const url = window.location.pathname;

    setCurrentChapter({
      title: title.trim(),
      text: text.trim(),
      url
    });

    return { title, text, url };
  }, []);

  /**
   * Open widget with current chapter context
   */
  const handleOpenWidget = useCallback(() => {
    extractChapterMetadata();
    setIsWidgetOpen(true);
  }, [extractChapterMetadata]);

  /**
   * Update chapter metadata when page changes
   */
  React.useEffect(() => {
    // Update metadata on page load
    extractChapterMetadata();

    // Listen for navigation changes (Docusaurus uses history API)
    const handlePopState = () => {
      extractChapterMetadata();
    };

    window.addEventListener('popstate', handlePopState);

    // Also watch for DOM changes (for dynamic content)
    const observer = new MutationObserver(() => {
      extractChapterMetadata();
    });

    const mainContent = document.querySelector('article') || document.querySelector('main');
    if (mainContent) {
      observer.observe(mainContent, {
        childList: true,
        subtree: true,
        characterData: false
      });
    }

    return () => {
      window.removeEventListener('popstate', handlePopState);
      observer.disconnect();
    };
  }, [extractChapterMetadata]);

  return (
    <>
      {children}
      
      {/* Inject RagChatWidget globally */}
      <RagChatWidget
        ref={widgetRef}
        isOpen={isWidgetOpen}
        onOpenChange={setIsWidgetOpen}
        chapterTitle={currentChapter.title}
        chapterText={currentChapter.text}
        chapterUrl={currentChapter.url}
      />

      {/* Make CTA available globally via window object for manual injection */}
      {typeof window !== 'undefined' && (
        (window as any).__ragWidget = {
          open: handleOpenWidget,
          close: () => setIsWidgetOpen(false),
          toggle: () => setIsWidgetOpen(!isWidgetOpen),
          setChapter: (title: string, text: string) => {
            setCurrentChapter(prev => ({ ...prev, title, text }));
          }
        }
      )}
    </>
  );
};

/**
 * Export both components for manual integration
 */
export { RagChatWidget, ChapterRAGCTA };
export default RootWithRAGWidget;
