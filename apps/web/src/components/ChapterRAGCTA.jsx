import React, { useState } from 'react';

/**
 * ChapterRAGCTA - Call-to-Action button for chapter-specific RAG queries
 * 
 * Appears at the top of chapter content pages.
 * Clicking extracts current chapter title and text, ingests into RAG, opens widget.
 */

interface ChapterRAGCTAProps {
  chapterTitle?: string;
  onOpenWidget?: () => void;
}

export const ChapterRAGCTA: React.FC<ChapterRAGCTAProps> = ({
  chapterTitle = 'this chapter',
  onOpenWidget
}) => {
  const [isHovered, setIsHovered] = useState(false);

  const handleClick = () => {
    // Extract chapter content from DOM
    const content = document.querySelector('article') || document.querySelector('main');
    
    if (onOpenWidget) {
      onOpenWidget();
    }

    // Optionally scroll to top to show widget
    window.scrollTo({ top: 0, behavior: 'smooth' });
  };

  return (
    <div
      className="mb-6 p-4 bg-gradient-to-r from-blue-50 to-indigo-50 border border-blue-200 rounded-lg"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      <button
        onClick={handleClick}
        className={`
          inline-flex items-center gap-2
          px-4 py-2 rounded-lg
          bg-blue-600 hover:bg-blue-700 text-white
          font-medium text-sm
          transition-all duration-200
          focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2
          ${isHovered ? 'shadow-lg' : 'shadow'}
        `}
        aria-label={`Ask RAG assistant about ${chapterTitle}`}
      >
        <svg
          className="w-4 h-4"
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M8.228 9c.549-1.165 2.03-2 3.772-2 2.21 0 4 1.343 4 3 0 1.4-1.278 2.575-3.006 2.907-.542.104-.994.54-.994 1.093m0 3h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
          />
        </svg>
        Ask about {chapterTitle}
      </button>
      <p className="text-xs text-gray-600 mt-2">
        💡 Have questions? Use our RAG assistant to get instant answers with source citations.
      </p>
    </div>
  );
};

export default ChapterRAGCTA;
