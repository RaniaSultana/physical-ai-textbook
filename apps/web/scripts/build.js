const fs = require('fs');
const path = require('path');

const outDir = path.resolve(__dirname, '..', 'build');
if (!fs.existsSync(outDir)) fs.mkdirSync(outDir, { recursive: true });

const docsSrc = path.resolve(__dirname, '..', '..', 'docs-source');
let docsIndex = '';
if (fs.existsSync(docsSrc)) {
  const files = fs.readdirSync(docsSrc).filter(f => f.endsWith('.md'));
  docsIndex = files.map(f => `- <a href=\"${f.replace('.md', '.html')}\">${f}</a>`).join('\n');
}

const html = `<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Physical AI Textbook (scaffold)</title>
  <style>body{font-family:system-ui, -apple-system, Roboto, 'Segoe UI', sans-serif; padding:24px;}</style>
</head>
<body>
  <h1>Physical AI Textbook (scaffold)</h1>
  <p>This is a minimal scaffold that stands in for a Docusaurus site until the environment can run `create-docusaurus`.</p>
  <h2>Docs</h2>
  <ul>
    ${docsIndex}
  </ul>
</body>
</html>`;

fs.writeFileSync(path.join(outDir, 'index.html'), html);
console.log('Built static scaffold to', outDir);
