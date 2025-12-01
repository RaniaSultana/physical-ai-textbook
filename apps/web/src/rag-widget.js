/**
 * RAG Widget Standalone Script
 * 
 * This script can be embedded in any HTML page to add RAG chat functionality.
 * Usage:
 *   <script src="/rag-widget.js"></script>
 */

(function() {
  const API_BASE_URL = window.RAG_CONFIG?.apiUrl || 'http://localhost:8000';
  const WIDGET_ID = 'rag-widget-root';

  // HTML template for widget
  const widgetHTML = `
    <div id="rag-widget" class="fixed bottom-6 right-6 z-50 hidden">
      <!-- Toggle Button -->
      <button 
        id="rag-toggle" 
        data-rag-toggle
        class="fixed bottom-6 right-6 z-40 w-14 h-14 rounded-full bg-blue-600 hover:bg-blue-700 text-white shadow-lg hover:shadow-xl flex items-center justify-center transition-all duration-200 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2"
        title="Open RAG Chat"
      >
        <svg class="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8m0 8l6-4m-6 4l-6-4" />
        </svg>
      </button>

      <!-- Chat Panel -->
      <div id="rag-panel" class="hidden fixed bottom-24 right-6 z-50 w-full max-w-md bg-white rounded-lg shadow-2xl border border-gray-200 flex flex-col max-h-96">
        <!-- Header -->
        <div class="px-4 py-3 border-b border-gray-200 bg-gradient-to-r from-blue-600 to-blue-700 text-white rounded-t-lg">
          <div class="flex items-center justify-between">
            <div>
              <h3 class="font-semibold text-base">RAG Assistant</h3>
              <p class="text-xs text-blue-100">Ask about this page</p>
            </div>
            <button 
              id="rag-close" 
              data-rag-close
              class="text-blue-100 hover:text-white focus:outline-none"
              title="Close"
            >
              <svg class="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                <path fill-rule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clip-rule="evenodd" />
              </svg>
            </button>
          </div>
        </div>

        <!-- Messages -->
        <div id="rag-messages" class="flex-1 overflow-y-auto p-4 space-y-4 bg-gray-50 max-h-60">
          <div class="text-center text-gray-500 text-sm py-8">
            <p class="mb-2">👋 Hi! Ask me anything about this page.</p>
          </div>
        </div>

        <!-- Input -->
        <form id="rag-form" class="border-t border-gray-200 p-4 bg-white rounded-b-lg">
          <div class="flex gap-2">
            <textarea
              id="rag-input"
              placeholder="Ask a question..."
              class="flex-1 resize-none px-3 py-2 rounded-lg border border-gray-300 focus:border-blue-500 focus:outline-none focus:ring-2 focus:ring-blue-200 text-sm placeholder-gray-500 max-h-24"
              rows="2"
            ></textarea>
            <button
              type="submit"
              id="rag-send"
              class="px-4 py-2 rounded-lg bg-blue-600 hover:bg-blue-700 text-white font-medium text-sm transition-colors duration-200 focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              Send
            </button>
          </div>
        </form>
      </div>
    </div>
  `;

  // Initialize widget
  function initWidget() {
    // Inject Tailwind CSS
    if (!document.querySelector('script[src*="tailwindcss"]')) {
      const script = document.createElement('script');
      script.src = 'https://cdn.tailwindcss.com';
      document.head.appendChild(script);
    }

    // Create widget container
    let container = document.getElementById(WIDGET_ID);
    if (!container) {
      container = document.createElement('div');
      container.id = WIDGET_ID;
      document.body.appendChild(container);
    }

    container.innerHTML = widgetHTML;

    // Setup event listeners
    setupEventListeners();
  }

  function setupEventListeners() {
    const toggleBtn = document.getElementById('rag-toggle');
    const closeBtn = document.getElementById('rag-close');
    const panel = document.getElementById('rag-panel');
    const form = document.getElementById('rag-form');
    const input = document.getElementById('rag-input');

    // Toggle panel
    if (toggleBtn) {
      toggleBtn.addEventListener('click', () => {
        panel.classList.toggle('hidden');
        const isOpen = !panel.classList.contains('hidden');
        
        if (isOpen) {
          input.focus();
          ingestCurrentPage();
        }
      });
    }

    // Close button
    if (closeBtn) {
      closeBtn.addEventListener('click', () => {
        panel.classList.add('hidden');
      });
    }

    // Close on Escape
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && !panel.classList.contains('hidden')) {
        panel.classList.add('hidden');
      }
    });

    // Form submission
    if (form) {
      form.addEventListener('submit', async (e) => {
        e.preventDefault();
        const query = input.value.trim();
        if (!query) return;

        await handleQuery(query);
        input.value = '';
      });
    }

    // Submit on Enter (Shift+Enter for newline)
    if (input) {
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          form.dispatchEvent(new Event('submit'));
        }
      });
    }
  }

  async function ingestCurrentPage() {
    try {
      const title = document.title || 'Page';
      const mainContent = document.querySelector('article') || document.querySelector('main') || document.body;
      const text = mainContent.textContent.slice(0, 5000);

      const response = await fetch(`${API_BASE_URL}/ingest`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          id: `page_${Date.now()}`,
          title,
          text
        })
      });

      if (response.ok) {
        console.log('✓ Page ingested into RAG');
      }
    } catch (err) {
      console.error('Error ingesting page:', err);
    }
  }

  async function handleQuery(query) {
    const messagesDiv = document.getElementById('rag-messages');
    const sendBtn = document.getElementById('rag-send');

    // Add user message
    const userMsg = document.createElement('div');
    userMsg.className = 'flex justify-end';
    userMsg.innerHTML = `
      <div class="max-w-xs px-4 py-3 rounded-lg bg-blue-600 text-white rounded-br-none">
        <p class="text-sm">${escapeHtml(query)}</p>
      </div>
    `;
    messagesDiv.appendChild(userMsg);

    sendBtn.disabled = true;
    sendBtn.textContent = '...';

    try {
      // Query RAG
      const queryResponse = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query, top_k: 3 })
      });

      const queryData = await queryResponse.json();
      const sources = queryData.results || [];

      if (sources.length === 0) {
        const assistantMsg = document.createElement('div');
        assistantMsg.className = 'flex justify-start';
        assistantMsg.innerHTML = `
          <div class="max-w-xs px-4 py-3 rounded-lg bg-white text-gray-800 rounded-bl-none border border-gray-200">
            <p class="text-sm">No relevant sources found. Please try a different question.</p>
          </div>
        `;
        messagesDiv.appendChild(assistantMsg);
      } else {
        // Generate answer using chat endpoint
        const context = sources
          .map((s, i) => \`[\${i + 1}] \${s.title}:\n\${s.text}\`)
          .join('\n\n');

        const chatResponse = await fetch(`${API_BASE_URL}/chat`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            messages: [
              { role: 'user', content: \`Context:\n\${context}\n\nQuestion: \${query}\` }
            ],
            model: 'openai'
          })
        });

        const chatData = await chatResponse.json();
        const answer = chatData.content || 'No response generated';

        const assistantMsg = document.createElement('div');
        assistantMsg.className = 'flex justify-start';
        assistantMsg.innerHTML = \`
          <div class="max-w-xs px-4 py-3 rounded-lg bg-white text-gray-800 rounded-bl-none border border-gray-200">
            <p class="text-sm leading-relaxed">\${escapeHtml(answer)}</p>
            <div class="mt-3 pt-3 border-t border-gray-200 text-xs">
              <div class="font-semibold text-gray-600 mb-2">Sources:</div>
              \${sources.map((s, i) => \`
                <div class="text-xs mb-1">
                  <span class="text-blue-600 font-medium">[${i + 1}] \${escapeHtml(s.title)}</span>
                </div>
              \`).join('')}
            </div>
          </div>
        \`;
        messagesDiv.appendChild(assistantMsg);
      }

      messagesDiv.scrollTop = messagesDiv.scrollHeight;
    } catch (err) {
      console.error('Error querying:', err);
      const errorMsg = document.createElement('div');
      errorMsg.className = 'flex justify-start';
      errorMsg.innerHTML = \`
        <div class="max-w-xs px-4 py-3 rounded-lg bg-red-50 text-red-700 border border-red-200">
          <p class="text-sm">Error: \${escapeHtml(err.message)}</p>
        </div>
      \`;
      messagesDiv.appendChild(errorMsg);
    } finally {
      sendBtn.disabled = false;
      sendBtn.textContent = 'Send';
    }
  }

  function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initWidget);
  } else {
    initWidget();
  }

  // Expose global API
  window.__ragWidget = {
    open() {
      document.getElementById('rag-panel')?.classList.remove('hidden');
    },
    close() {
      document.getElementById('rag-panel')?.classList.add('hidden');
    },
    toggle() {
      document.getElementById('rag-panel')?.classList.toggle('hidden');
    }
  };
})();
