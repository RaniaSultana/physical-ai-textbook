import React, { useEffect, useState } from 'react';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const SelectionAskButton = ({ onSessionCreated }) => {
  const [visible, setVisible] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelection = () => {
      try {
        const selection = window.getSelection();
        if (!selection) return;
        const text = selection.toString().trim();
        if (text.length > 5) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setPosition({ x: rect.right + window.scrollX, y: rect.top + window.scrollY - 10 });
          setVisible(true);
        } else {
          setVisible(false);
        }
      } catch (e) {
        setVisible(false);
      }
    };

    document.addEventListener('selectionchange', handleSelection);
    return () => document.removeEventListener('selectionchange', handleSelection);
  }, []);

  const handleClick = async () => {
    const selection = window.getSelection();
    if (!selection) return;
    const text = selection.toString().trim();
    if (!text) return;

    const sessionId = `session_${Date.now()}_${Math.floor(Math.random() * 10000)}`;
    const docId = `sel_${Date.now()}_${Math.floor(Math.random() * 10000)}`;

    try {
      const resp = await fetch(`${API_BASE_URL}/ingest`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          id: docId,
          title: 'Selection',
          text,
          session: true,
          session_id: sessionId
        })
      });

      if (resp.ok) {
        if (onSessionCreated) onSessionCreated({ sessionId, docId });
        window.__ragWidget = window.__ragWidget || {};
        window.__ragWidget.session = { sessionId, restrict: true };
        window.__ragWidget.open?.();
      } else {
        console.error('Failed to ingest selection', resp.statusText);
      }
    } catch (err) {
      console.error('Error ingesting selection', err);
    }
  };

  if (!visible) return null;

  return (
    <button
      onClick={handleClick}
      style={{ position: 'absolute', left: position.x + 8, top: position.y - 40, zIndex: 9999 }}
      className="px-3 py-1 bg-green-600 text-white rounded-md shadow-lg"
      aria-label="Ask about selection"
    >
      Ask about selection
    </button>
  );
};

export default SelectionAskButton;
