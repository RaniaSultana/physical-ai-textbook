#!/bin/bash
# Deploy backend to Fly.io or Railway
# Usage: ./infra/deploy-backend.sh [fly|railway|docker]

set -e

TARGET=${1:-fly}
DOCKER_IMAGE="ranisultana/physical-ai-backend:latest"

echo "🚀 Deploying backend (${TARGET})..."

# Build Docker image
echo "📦 Building Docker image..."
docker build -t ${DOCKER_IMAGE} packages/backend/

case $TARGET in
  fly)
    echo "Deploying to Fly.io..."
    # Requires: flyctl installed and authenticated
    # fly auth login
    flyctl deploy --remote-only
    echo "✅ Deployed to Fly.io"
    echo "🌐 Backend URL: https://physical-ai-backend.fly.dev"
    ;;
  railway)
    echo "Deploying to Railway..."
    # Requires: railway CLI installed
    # railway login
    railway up
    echo "✅ Deployed to Railway"
    ;;
  docker)
    echo "Pushing Docker image to registry..."
    # Requires: docker login
    docker push ${DOCKER_IMAGE}
    echo "✅ Image pushed: ${DOCKER_IMAGE}"
    ;;
  *)
    echo "Unknown target: $TARGET"
    echo "Usage: deploy-backend.sh [fly|railway|docker]"
    exit 1
    ;;
esac

echo "✅ Backend deployment complete!"
