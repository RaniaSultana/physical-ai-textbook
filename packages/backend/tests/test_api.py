"""Test suite for backend API."""

import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_root_endpoint():
    """Test the root health check endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["name"] == "Physical AI Textbook Backend"
    assert data["status"] == "running"

def test_chat_endpoint_invalid_model():
    """Test chat endpoint with invalid model."""
    response = client.post("/chat", json={
        "messages": [{"role": "user", "content": "Hello"}],
        "model": "invalid"
    })
    assert response.status_code == 400

def test_embedding_endpoint_not_configured():
    """Test embedding endpoint when OpenAI is not configured."""
    response = client.post("/embed", json={
        "text": "Hello world"
    })
    # Should return 503 if not configured
    assert response.status_code == 503 or response.status_code == 200

def test_rag_search_endpoint():
    """Test RAG search endpoint."""
    response = client.post("/rag-search", json={
        "query": "What is ROS 2?",
        "top_k": 5
    })
    assert response.status_code == 200
    data = response.json()
    assert "query" in data
    assert "results" in data
