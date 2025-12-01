#!/bin/bash
# Deploy book to GitHub Pages
# Usage: ./infra/deploy-gh-pages.sh

set -e

REPO_URL="https://github.com/RaniaSultana/physical-ai-textbook.git"
BUILD_DIR="apps/web/build"
PAGES_BRANCH="gh-pages"

echo "🚀 Deploying to GitHub Pages..."

# Build the site
echo "📦 Building web site..."
yarn web:build

# Create gh-pages branch if it doesn't exist
if ! git show-ref --quiet refs/heads/${PAGES_BRANCH}; then
    echo "Creating ${PAGES_BRANCH} branch..."
    git checkout --orphan ${PAGES_BRANCH}
    git reset --hard
    git commit --allow-empty -m "Initial gh-pages commit"
fi

# Stash current changes
git stash

# Checkout gh-pages
git checkout ${PAGES_BRANCH}

# Copy build output to root
cp -r ${BUILD_DIR}/* .

# Add and commit
git add .
if ! git diff-index --quiet --cached HEAD --; then
    git commit -m "chore: deploy to GitHub Pages [skip ci]"
fi

# Push to gh-pages
git push -u origin ${PAGES_BRANCH}

# Switch back to main
git checkout main
git stash pop || true

echo "✅ Deployed to GitHub Pages!"
echo "📖 View at: https://ranisultana.github.io/physical-ai-textbook/"
