const fs = require('fs');
const path = require('path');

const outDir = path.resolve(__dirname, '..', 'build');
if (!fs.existsSync(outDir)) fs.mkdirSync(outDir, { recursive: true });

const docsSrc = path.resolve(__dirname, '..', '..', 'docs-source');
let docsIndex = '';
if (fs.existsSync(docsSrc)) {
  const files = fs.readdirSync(docsSrc).filter(f => f.endsWith('.md'));
  docsIndex = files.map(f => `- <a href="${f.replace('.md', '.html')}">${f}</a>`).join('\n');
}

// Copy RAG widget script
const widgetSrc = path.resolve(__dirname, '..', 'src', 'rag-widget.js');
const widgetDest = path.join(outDir, 'rag-widget.js');
if (fs.existsSync(widgetSrc)) {
  fs.copyFileSync(widgetSrc, widgetDest);
  console.log('Copied RAG widget script');
}

const html = `<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Physical AI Textbook (scaffold)</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body {
      font-family: system-ui, -apple-system, Roboto, 'Segoe UI', sans-serif;
      padding: 24px;
    }
    .rag-widget-container {
      position: fixed;
      bottom: 24px;
      right: 24px;
      z-index: 40;
    }
    @keyframes bounce {
      0%, 100% { transform: translateY(0); }
      50% { transform: translateY(-8px); }
    }
    .animate-bounce { animation: bounce 1s infinite; }
  </style>
</head>
<body>
  <h1>Physical AI Textbook (scaffold)</h1>
  <p>This is a minimal scaffold that stands in for a Docusaurus site until the environment can run create-docusaurus.</p>
  <h2>Docs</h2>
  <ul>
    ${docsIndex}
  </ul>
  
  <!-- RAG Widget Container -->
  <div id="rag-widget-root" class="rag-widget-container"></div>
  
  <!-- RAG Widget Script -->
  <script src="/rag-widget.js"></script>
  
  <script>
    // Global RAG widget control
    window.__ragWidget = {
      open: function() {
        console.log('RAG Widget: Opening widget');
        const btn = document.querySelector('[data-rag-toggle]');
        if (btn) btn.click();
      },
      close: function() {
        console.log('RAG Widget: Closing widget');
        const closeBtn = document.querySelector('[data-rag-close]');
        if (closeBtn) closeBtn.click();
      },
      toggle: function() {
        console.log('RAG Widget: Toggling widget');
        const btn = document.querySelector('[data-rag-toggle]');
        if (btn) btn.click();
      }
    };
  </script>
</body>
</html>`;

fs.writeFileSync(path.join(outDir, 'index.html'), html);
console.log('✓ Built static scaffold to', outDir);
console.log('✓ RAG widget available at /rag-widget.js');

