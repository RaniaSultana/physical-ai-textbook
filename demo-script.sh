#!/bin/bash
# Demo script: Show off the Physical AI Textbook in <90 seconds
# This script demonstrates:
# - Backend API endpoints
# - Chat component integration
# - Agent framework basics
# 
# Usage: ./demo-script.sh

set -e

BACKEND_URL="http://localhost:8000"
TIMEOUT=5

echo "🤖 Physical AI Textbook - 90 Second Demo"
echo "========================================="
echo ""

# Check if backend is running
echo "📋 Checking backend..."
if ! curl -s "${BACKEND_URL}/" > /dev/null; then
    echo "⚠️  Backend not running. Starting it..."
    yarn backend:start &
    BACKEND_PID=$!
    sleep 3
fi

echo "✅ Backend is running at ${BACKEND_URL}"
echo ""

# Demo 1: Health check
echo "1️⃣  Health Check"
echo "   GET ${BACKEND_URL}/"
RESPONSE=$(curl -s "${BACKEND_URL}/")
echo "   Response: $(echo $RESPONSE | jq -r '.status')"
echo ""

# Demo 2: Chat with Claude
echo "2️⃣  Chat with Claude"
CHAT_REQUEST='{
  "messages": [{"role": "user", "content": "What is ROS 2?"}],
  "model": "claude",
  "system_prompt": "You are a helpful robotics assistant."
}'
echo "   POST ${BACKEND_URL}/chat"
echo "   Query: \"What is ROS 2?\""

# Try to get a real response, but don't fail if API key is missing
CHAT_RESPONSE=$(curl -s -X POST "${BACKEND_URL}/chat" \
  -H "Content-Type: application/json" \
  -d "$CHAT_REQUEST" || echo '{"content":"(Claude API key not configured)","model":"claude"}')

ANSWER=$(echo $CHAT_RESPONSE | jq -r '.content')
MODEL=$(echo $CHAT_RESPONSE | jq -r '.model')
echo "   Model: $MODEL"
echo "   Response: ${ANSWER:0:80}..."
echo ""

# Demo 3: RAG Search
echo "3️⃣  RAG Vector Search"
RAG_REQUEST='{
  "query": "How do I use digital twins in robotics?",
  "top_k": 3
}'
echo "   POST ${BACKEND_URL}/rag-search"
echo "   Query: \"How do I use digital twins?\""

RAG_RESPONSE=$(curl -s -X POST "${BACKEND_URL}/rag-search" \
  -H "Content-Type: application/json" \
  -d "$RAG_REQUEST")

RESULT_COUNT=$(echo $RAG_RESPONSE | jq -r '.count')
echo "   Results found: $RESULT_COUNT"
if [ "$RESULT_COUNT" -gt 0 ]; then
    FIRST_RESULT=$(echo $RAG_RESPONSE | jq -r '.results[0].text')
    echo "   First result: ${FIRST_RESULT:0:60}..."
fi
echo ""

# Demo 4: Show docs/modules
echo "4️⃣  Textbook Content"
echo "   Available Modules:"
echo "   - Module 1: Robotic Nervous System (ROS 2)"
echo "   - Module 2: Digital Twin & Simulation"
echo "   - Module 3: NVIDIA Isaac & Perception"
echo "   - Module 4: Vision & Language Agents (VLA)"
echo ""

# Demo 5: Code example
echo "5️⃣  ROS 2 Hello Node Example"
echo "   Location: examples/ros2/hello_node.py"
echo "   "
head -15 examples/ros2/hello_node.py | tail -10
echo "   ..."
echo ""

echo "========================================="
echo "✅ Demo complete!"
echo ""
echo "📚 Explore the full textbook:"
echo "   • Web: http://localhost:3000"
echo "   • Docs: http://localhost:3000/docs"
echo "   • API: http://localhost:8000/docs"
echo ""
echo "🚀 Next steps:"
echo "   1. Set API keys in .env"
echo "   2. Try the chat component at /chat"
echo "   3. Integrate agents with your robot system"
echo ""

# Cleanup
if [ ! -z "$BACKEND_PID" ]; then
    kill $BACKEND_PID 2>/dev/null || true
fi
