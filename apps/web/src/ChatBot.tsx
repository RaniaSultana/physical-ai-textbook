/**
 * ChatBot Component
 * 
 * A React component that provides a chat interface to the Physical AI backend.
 * - Sends user messages to the backend /chat endpoint
 * - Supports Claude and OpenAI model selection
 * - Displays conversation history
 * - Handles WebSocket connections for real-time updates
 */

import React, { useState, useRef, useEffect } from 'react';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  model?: string;
  timestamp: Date;
}

interface ChatBotProps {
  backendUrl?: string;
  systemPrompt?: string;
}

const ChatBot: React.FC<ChatBotProps> = ({
  backendUrl = 'http://localhost:8000',
  systemPrompt = 'You are a helpful robotics assistant for the Physical AI textbook.'
}) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [model, setModel] = useState<'claude' | 'openai'>('claude');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim()) return;

    // Add user message to state
    const userMessage: Message = {
      id: `msg_${Date.now()}`,
      role: 'user',
      content: input,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);
    setError(null);

    try {
      // Call backend /chat endpoint
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          messages: messages
            .concat(userMessage)
            .map(m => ({ role: m.role, content: m.content })),
          model,
          system_prompt: systemPrompt
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage: Message = {
        id: `msg_${Date.now()}`,
        role: 'assistant',
        content: data.content,
        model: data.model,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Unknown error';
      setError(`Failed to send message: ${errorMessage}`);
      console.error('Chat error:', err);
    } finally {
      setLoading(false);
    }
  };

  const clearHistory = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <h2 style={styles.title}>Physical AI ChatBot</h2>
        <select
          value={model}
          onChange={(e) => setModel(e.target.value as 'claude' | 'openai')}
          style={styles.modelSelect}
        >
          <option value="claude">Claude 3.5 Sonnet</option>
          <option value="openai">GPT-4o</option>
        </select>
      </div>

      {error && (
        <div style={styles.errorBanner}>
          ⚠️ {error}
        </div>
      )}

      <div style={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div style={styles.emptyState}>
            <p>Start a conversation about Physical AI and Robotics!</p>
            <p style={styles.hint}>Ask about ROS 2, simulation, perception, or VLA agents.</p>
          </div>
        ) : (
          messages.map((msg) => (
            <div
              key={msg.id}
              style={{
                ...styles.message,
                ...(msg.role === 'user' ? styles.userMessage : styles.assistantMessage)
              }}
            >
              <div style={styles.messageMeta}>
                <strong>{msg.role === 'user' ? 'You' : msg.model || 'Assistant'}</strong>
                <span style={styles.timestamp}>
                  {msg.timestamp.toLocaleTimeString()}
                </span>
              </div>
              <div style={styles.messageContent}>{msg.content}</div>
            </div>
          ))
        )}
        {loading && (
          <div style={{ ...styles.message, ...styles.assistantMessage }}>
            <div style={styles.messageMeta}>
              <strong>Assistant</strong>
            </div>
            <div style={styles.messageContent}>
              <span style={styles.typing}>Typing...</span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={sendMessage} style={styles.form}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question about robotics..."
          style={styles.input}
          disabled={loading}
        />
        <button
          type="submit"
          style={{ ...styles.button, opacity: loading ? 0.6 : 1 }}
          disabled={loading}
        >
          {loading ? 'Sending...' : 'Send'}
        </button>
        {messages.length > 0 && (
          <button
            type="button"
            onClick={clearHistory}
            style={styles.clearButton}
            disabled={loading}
          >
            Clear
          </button>
        )}
      </form>
    </div>
  );
};

const styles: Record<string, React.CSSProperties> = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    height: '100vh',
    maxWidth: '800px',
    margin: '0 auto',
    backgroundColor: '#f5f5f5',
    fontFamily: 'system-ui, -apple-system, sans-serif'
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '16px',
    backgroundColor: '#2c3e50',
    color: 'white',
    borderBottom: '1px solid #ddd'
  },
  title: {
    margin: 0,
    fontSize: '20px'
  },
  modelSelect: {
    padding: '8px 12px',
    borderRadius: '4px',
    border: '1px solid #ccc',
    fontSize: '14px',
    cursor: 'pointer'
  },
  errorBanner: {
    padding: '12px 16px',
    backgroundColor: '#ffe6e6',
    color: '#c33',
    fontSize: '14px',
    borderBottom: '1px solid #ffcccc'
  },
  messagesContainer: {
    flex: 1,
    overflowY: 'auto',
    padding: '16px',
    display: 'flex',
    flexDirection: 'column',
    gap: '12px'
  },
  emptyState: {
    textAlign: 'center',
    color: '#666',
    margin: 'auto',
    padding: '24px'
  },
  hint: {
    fontSize: '14px',
    color: '#999',
    marginTop: '8px'
  },
  message: {
    padding: '12px 16px',
    borderRadius: '8px',
    maxWidth: '70%',
    wordWrap: 'break-word'
  },
  userMessage: {
    alignSelf: 'flex-end',
    backgroundColor: '#007bff',
    color: 'white'
  },
  assistantMessage: {
    alignSelf: 'flex-start',
    backgroundColor: 'white',
    border: '1px solid #ddd'
  },
  messageMeta: {
    display: 'flex',
    justifyContent: 'space-between',
    fontSize: '12px',
    marginBottom: '4px',
    opacity: 0.8
  },
  timestamp: {
    marginLeft: '8px'
  },
  messageContent: {
    fontSize: '14px',
    lineHeight: '1.4'
  },
  typing: {
    animation: 'blink 1.4s infinite'
  },
  form: {
    display: 'flex',
    gap: '8px',
    padding: '16px',
    backgroundColor: 'white',
    borderTop: '1px solid #ddd'
  },
  input: {
    flex: 1,
    padding: '10px 12px',
    borderRadius: '4px',
    border: '1px solid #ddd',
    fontSize: '14px',
    fontFamily: 'inherit'
  },
  button: {
    padding: '10px 20px',
    borderRadius: '4px',
    border: 'none',
    backgroundColor: '#007bff',
    color: 'white',
    cursor: 'pointer',
    fontSize: '14px',
    fontWeight: '500'
  },
  clearButton: {
    padding: '10px 12px',
    borderRadius: '4px',
    border: '1px solid #ccc',
    backgroundColor: '#f0f0f0',
    color: '#333',
    cursor: 'pointer',
    fontSize: '14px'
  }
};

export default ChatBot;
