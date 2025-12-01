# Physical AI Textbook (scaffold)

This repository is a scaffold for the "Textbook for Teaching Physical AI & Humanoid Robotics" project.

- `apps/web` — Docusaurus v2 site scaffold (placeholder until `create-docusaurus` runs locally)
- `packages/backend` — FastAPI backend (placeholder)
- `packages/agents` — Agent definitions (placeholder)
- `infra` — deployment scripts (placeholder)
- `docs-source` — markdown source for book content

Quick start (macOS / zsh):

```bash
cd /Users/apple/Documents/copilot-hackathon/physical-ai-textbook
# If you want a true Docusaurus site, run locally (fix npm permission first):
# sudo chown -R $(id -u):$(id -g) ~/.npm
# then run: npx create-docusaurus@latest apps/web classic

# Build the scaffold web site (no external installs required):
yarn web:build

# Serve it locally:
yarn web:start
```
