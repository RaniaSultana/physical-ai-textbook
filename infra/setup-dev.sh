#!/bin/bash
# Local development setup script
# Usage: ./infra/setup-dev.sh

set -e

echo "🔧 Setting up local development environment..."

# Copy env template
if [ ! -f .env ]; then
    echo "📝 Creating .env from template..."
    cp .env.example .env
    echo "⚠️  Edit .env with your API keys before running the app"
fi

# Install dependencies
echo "📦 Installing Node dependencies..."
yarn install

echo "📦 Installing Python backend dependencies..."
cd packages/backend
pip install -r requirements.txt
cd ../..

# Create test data directory
mkdir -p data/{synthetic_dataset,real_world_test}

echo "✅ Development environment setup complete!"
echo ""
echo "Next steps:"
echo "1. Edit .env with your API keys (CLAUDE_CODE_API_KEY, OPENAI_GEMINI_KEY, etc.)"
echo "2. Run 'yarn dev' to start web + backend"
echo "3. Visit http://localhost:3000 for the web site"
echo "4. Backend API at http://localhost:8000/docs"
