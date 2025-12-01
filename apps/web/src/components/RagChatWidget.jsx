import React, { useState, useRef, useEffect } from 'react';

/**
 * RagChatWidget - Collapsible chat panel for RAG-powered Q&A
 * 
 * Features:
 * - Bottom-right anchored collapsible panel
 * - Query backend RAG endpoints for source passages
 * - Generate LLM answers using only retrieved context (RAG)
 * - Show source citations with links
 * - Keyboard accessible (Escape to close)
 * - Mobile responsive
 * - Session ingestion of current chapter text
 */

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    source_id: string;
    title: string;
    text: string;
    score: number;
    url?: string;
  }>;
}

interface RagChatWidgetProps {
  chapterTitle?: string;
  chapterText?: string;
  chapterUrl?: string;
  onIngest?: (doc_id: string) => void;
}

export const RagChatWidget: React.FC<RagChatWidgetProps> = ({
  chapterTitle = 'Chapter',
  chapterText = '',
  chapterUrl = '',
  onIngest
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionIngested, setSessionIngested] = useState(false);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to latest message
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Keyboard navigation (Escape to close)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };
    
    if (isOpen) {
      window.addEventListener('keydown', handleKeyDown);
      return () => window.removeEventListener('keydown', handleKeyDown);
    }
  }, [isOpen]);

  // Auto-ingest current chapter when widget opens and chapter text available
  useEffect(() => {
    if (isOpen && chapterText && !sessionIngested) {
      ingestChapter();
    }
  }, [isOpen]);

  /**
   * Ingest current chapter into RAG system
   */
  const ingestChapter = async () => {
    if (!chapterText) {
      console.warn('No chapter text to ingest');
      return;
    }

    try {
      const docId = `chapter_${Date.now()}`;
      
      const response = await fetch(`${API_BASE_URL}/ingest`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          id: docId,
          title: chapterTitle,
          text: chapterText
        })
      });

      if (response.ok) {
        setSessionIngested(true);
        const data = await response.json();
        console.log('✓ Chapter ingested:', data.message);
        
        if (onIngest) {
          onIngest(docId);
        }

        // Add system message
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: `📚 Ready to answer questions about "${chapterTitle}". Ask away!`
        }]);
      } else {
        console.error('Failed to ingest chapter:', response.statusText);
      }
    } catch (err) {
      console.error('Error ingesting chapter:', err);
    }
  };

  /**
   * Query RAG system for relevant passages
   */
  const queryRAG = async (query: string) => {
    try {
      // include optional session restriction if present on global widget
      const session = (window as any).__ragWidget?.session;

      const body: any = { query, top_k: 3 };
      if (session && session.restrict && session.sessionId) {
        body.restrict_to_session = true;
        body.session_id = session.sessionId;
      }

      const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      });

      if (!response.ok) {
        throw new Error(`Query failed: ${response.statusText}`);
      }

      const data = await response.json();
      return data.results || [];
    } catch (err) {
      console.error('RAG query error:', err);
      setError('Failed to retrieve sources. Please try again.');
      return [];
    }
  };

  /**
   * Generate answer using OpenAI with retrieved context
   */
  const generateAnswer = async (userQuery: string, sources: any[]) => {
    try {
      // Build context from sources
      const context = sources
        .map((s, i) => `[${i + 1}] ${s.title}:\n${s.text}`)
        .join('\n\n');

      const systemPrompt = `You are a helpful assistant answering questions about robotics and AI textbooks. 
Use ONLY the provided source material to answer the question. 
If the answer is not in the sources, say so clearly.
Format your answer with inline citations using [1], [2], etc. format.
Be concise and clear.`;

      const userPrompt = `Context from textbook:\n${context}\n\nQuestion: ${userQuery}`;

      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          messages: [
            { role: 'user', content: userPrompt }
          ],
          model: 'openai',
          system_prompt: systemPrompt
        })
      });

      if (!response.ok) {
        throw new Error(`Chat failed: ${response.statusText}`);
      }

      const data = await response.json();
      return data.content || 'No response generated';
    } catch (err) {
      console.error('Answer generation error:', err);
      setError('Failed to generate answer. Please try again.');
      return 'Sorry, I encountered an error generating an answer.';
    }
  };

  /**
   * Handle user message submission
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    const trimmedInput = inputValue.trim();
    if (!trimmedInput || isLoading) return;

    // Add user message
    const userMessage: Message = {
      role: 'user',
      content: trimmedInput
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Query RAG for sources (honor selection session if set)
      const sources = await queryRAG(trimmedInput);

      if (sources.length === 0) {
        // Fallback if no sources found
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'No relevant sources found. Please try a different question or make sure content is ingested.'
        }]);
        setIsLoading(false);
        return;
      }

      // Generate answer using sources as context
      const answer = await generateAnswer(trimmedInput, sources);

      // Add assistant message with sources
      const assistantMessage: Message = {
        role: 'assistant',
        content: answer,
        sources: sources.map((s, idx) => ({
          ...s,
          url: chapterUrl || `#${s.source_id}`
        }))
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error handling submission:', err);
      setError('An error occurred. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Render source citations
   */
  const renderSources = (sources: any[] | undefined) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="mt-3 pt-3 border-t border-gray-200 text-sm">
        <div className="text-xs font-semibold text-gray-600 mb-2">Sources:</div>
        <div className="space-y-1">
          {sources.map((source, idx) => (
            <div key={idx} className="text-xs">
              <a
                href={source.url || '#'}
                className="text-blue-600 hover:text-blue-800 hover:underline font-medium"
                target={source.url ? '_blank' : undefined}
                rel={source.url ? 'noopener noreferrer' : undefined}
              >
                [{idx + 1}] {source.title}
              </a>
              <span className="text-gray-500 ml-2">
                (confidence: {(source.score * 100).toFixed(0)}%)
              </span>
            </div>
          ))}
        </div>
      </div>
    );
  };

  return (
    <>
      {/* Widget Toggle Button - Bottom Right */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`
          fixed bottom-6 right-6 z-40
          w-14 h-14 rounded-full
          bg-blue-600 hover:bg-blue-700 text-white
          shadow-lg hover:shadow-xl
          flex items-center justify-center
          transition-all duration-200
          focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2
          ${isOpen ? 'hidden md:flex' : 'flex'}
        `}
        title={isOpen ? 'Close' : 'Open RAG Chat'}
        aria-label={isOpen ? 'Close RAG Chat' : 'Open RAG Chat'}
      >
        <svg
          className="w-6 h-6"
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d={isOpen ? 'M6 18L18 6M6 6l12 12' : 'M12 6v6m0 0v6m0-6h6m0 0h6m-6 0h-6'}
          />
        </svg>
      </button>

      {/* Chat Panel - Collapsible */}
      {isOpen && (
        <div
          className={`
            fixed bottom-24 right-6 z-50
            w-full max-w-md
            bg-white rounded-lg shadow-2xl
            border border-gray-200
            flex flex-col
            max-h-96
            md:max-h-full md:max-w-lg
            md:bottom-24 md:right-6
            sm:bottom-20 sm:right-4 sm:max-w-sm
          `}
        >
          {/* Header */}
          <div className="px-4 py-3 border-b border-gray-200 bg-gradient-to-r from-blue-600 to-blue-700 text-white rounded-t-lg">
            <div className="flex items-center justify-between">
              <div>
                <h3 className="font-semibold text-base">RAG Assistant</h3>
                <p className="text-xs text-blue-100">Ask about this chapter</p>
              </div>
              <button
                onClick={() => setIsOpen(false)}
                className="text-blue-100 hover:text-white focus:outline-none"
                aria-label="Close"
              >
                <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                  <path
                    fillRule="evenodd"
                    d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z"
                    clipRule="evenodd"
                  />
                </svg>
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 bg-gray-50">
            {messages.length === 0 ? (
              <div className="text-center text-gray-500 text-sm py-8">
                <p className="mb-2">👋 Hi! I can help you understand this chapter.</p>
                <p>Ask me anything about the content.</p>
              </div>
            ) : (
              messages.map((msg, idx) => (
                <div
                  key={idx}
                  className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}
                >
                  <div
                    className={`
                      max-w-xs px-4 py-3 rounded-lg
                      ${msg.role === 'user'
                        ? 'bg-blue-600 text-white rounded-br-none'
                        : 'bg-white text-gray-800 rounded-bl-none border border-gray-200'
                      }
                    `}
                  >
                    <p className="text-sm leading-relaxed whitespace-pre-wrap">
                      {msg.content}
                    </p>
                    {msg.role === 'assistant' && renderSources(msg.sources)}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="flex justify-start">
                <div className="bg-white text-gray-800 rounded-lg rounded-bl-none border border-gray-200 px-4 py-3">
                  <div className="flex space-x-2">
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" />
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }} />
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.4s' }} />
                  </div>
                </div>
              </div>
            )}
            {error && (
              <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-3 rounded-lg text-sm">
                ⚠️ {error}
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <form onSubmit={handleSubmit} className="border-t border-gray-200 p-4 bg-white rounded-b-lg">
            <div className="flex gap-2">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    handleSubmit(e as any);
                  }
                }}
                placeholder="Ask a question... (Shift+Enter for new line)"
                className={`
                  flex-1 resize-none
                  px-3 py-2 rounded-lg
                  border border-gray-300 focus:border-blue-500
                  focus:outline-none focus:ring-2 focus:ring-blue-200
                  text-sm placeholder-gray-500
                  max-h-24
                `}
                disabled={isLoading}
                rows={2}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
                className={`
                  px-4 py-2 rounded-lg
                  bg-blue-600 hover:bg-blue-700 text-white
                  font-medium text-sm
                  transition-colors duration-200
                  disabled:opacity-50 disabled:cursor-not-allowed
                  focus:outline-none focus:ring-2 focus:ring-blue-500
                `}
                aria-label="Send message"
              >
                {isLoading ? '...' : 'Send'}
              </button>
            </div>
          </form>
        </div>
      )}
    </>
  );
};

export default RagChatWidget;
