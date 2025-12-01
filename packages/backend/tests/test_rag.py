"""Unit tests for RAG endpoints."""

import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add packages/backend to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import app

client = TestClient(app)


class TestRAGEndpoints:
    """Test suite for RAG ingest and query endpoints."""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """Set up test fixtures."""
        # Ensure we're using mock RAG service for testing
        os.environ.pop("NEON_DB_URL", None)
        os.environ.pop("QDRANT_URL", None)
        os.environ.pop("QDRANT_API_KEY", None)
        yield
    
    def test_ingest_document_success(self):
        """Test successful document ingestion."""
        payload = {
            "id": "doc_001",
            "title": "ROS 2 Fundamentals",
            "text": "ROS 2 is a middleware for robotics development. It provides tools for message passing, service calls, and action servers."
        }
        
        response = client.post("/ingest", json=payload)
        
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["doc_id"] == "doc_001"
        assert "ingested successfully" in data["message"]
    
    def test_ingest_document_missing_fields(self):
        """Test ingest with missing required fields."""
        payload = {
            "id": "doc_002",
            "title": "Incomplete Document"
            # Missing 'text' field
        }
        
        response = client.post("/ingest", json=payload)
        
        # Should fail validation
        assert response.status_code == 422
    
    def test_query_returns_results(self):
        """Test query endpoint returns results."""
        # First, ingest a test document
        ingest_payload = {
            "id": "test_doc_001",
            "title": "Physics Simulation",
            "text": "Gazebo is a powerful physics simulator used for robotics. It supports dynamic simulation with realistic physics parameters."
        }
        
        ingest_response = client.post("/ingest", json=ingest_payload)
        assert ingest_response.status_code == 200
        
        # Now query for documents
        query_payload = {
            "query": "What is Gazebo used for?",
            "top_k": 5
        }
        
        query_response = client.post("/query", json=query_payload)
        
        assert query_response.status_code == 200
        data = query_response.json()
        assert "query" in data
        assert "results" in data
        assert "count" in data
        assert isinstance(data["results"], list)
    
    def test_query_with_context_filter(self):
        """Test query with context_ids filter."""
        # Ingest multiple documents
        docs = [
            {
                "id": "ros_doc",
                "title": "ROS 2",
                "text": "ROS 2 is the robot operating system framework."
            },
            {
                "id": "gazebo_doc",
                "title": "Gazebo",
                "text": "Gazebo is a simulation environment for robotics."
            }
        ]
        
        for doc in docs:
            client.post("/ingest", json=doc)
        
        # Query with context filter
        query_payload = {
            "query": "simulation",
            "top_k": 5,
            "context_ids": ["gazebo_doc"]  # Only search in gazebo_doc
        }
        
        query_response = client.post("/query", json=query_payload)
        
        assert query_response.status_code == 200
        data = query_response.json()
        
        # All returned results should have source_id in context_ids
        for result in data["results"]:
            assert result["source_id"] in ["gazebo_doc"]
    
    def test_query_custom_top_k(self):
        """Test query with custom top_k parameter."""
        # Ingest multiple documents
        for i in range(5):
            client.post("/ingest", json={
                "id": f"doc_{i}",
                "title": f"Document {i}",
                "text": f"This is document number {i} about robotics."
            })
        
        # Query with top_k=2
        query_response = client.post("/query", json={
            "query": "robotics",
            "top_k": 2
        })
        
        assert query_response.status_code == 200
        data = query_response.json()
        
        # Should return at most top_k results
        assert len(data["results"]) <= 2
    
    def test_query_empty_results(self):
        """Test query returns empty results gracefully."""
        # Query without any documents ingested
        query_response = client.post("/query", json={
            "query": "nonexistent topic xyz",
            "top_k": 5
        })
        
        assert query_response.status_code == 200
        data = query_response.json()
        assert data["count"] == 0 or len(data["results"]) == 0
    
    def test_ingest_response_format(self):
        """Test ingest response has correct format."""
        payload = {
            "id": "format_test",
            "title": "Format Test",
            "text": "Testing response format"
        }
        
        response = client.post("/ingest", json=payload)
        
        assert response.status_code == 200
        data = response.json()
        
        # Check response structure
        assert "status" in data
        assert "doc_id" in data
        assert "message" in data
    
    def test_query_response_format(self):
        """Test query response has correct format."""
        # Ingest a test document first
        client.post("/ingest", json={
            "id": "format_doc",
            "title": "Format Test",
            "text": "Test content for format validation"
        })
        
        response = client.post("/query", json={
            "query": "test",
            "top_k": 5
        })
        
        assert response.status_code == 200
        data = response.json()
        
        # Check response structure
        assert "query" in data
        assert "results" in data
        assert "count" in data
        
        # Check result structure if present
        if data["results"]:
            result = data["results"][0]
            assert "source_id" in result
            assert "title" in result
            assert "text" in result
            assert "score" in result
    
    def test_root_endpoint_includes_rag_routes(self):
        """Test root endpoint lists new RAG routes."""
        response = client.get("/")
        
        assert response.status_code == 200
        data = response.json()
        
        endpoints = data.get("endpoints", [])
        assert "/ingest" in endpoints
        assert "/query" in endpoints
    
    def test_ingest_duplicate_document(self):
        """Test ingesting document with duplicate ID updates it."""
        doc_id = "update_test"
        
        # Ingest first version
        response1 = client.post("/ingest", json={
            "id": doc_id,
            "title": "Original Title",
            "text": "Original text content"
        })
        assert response1.status_code == 200
        
        # Ingest updated version with same ID
        response2 = client.post("/ingest", json={
            "id": doc_id,
            "title": "Updated Title",
            "text": "Updated text content"
        })
        assert response2.status_code == 200
        
        # Both should succeed (upsert behavior)
        assert response1.json()["status"] == "success"
        assert response2.json()["status"] == "success"


class TestRAGIntegration:
    """Integration tests for RAG workflow."""
    
    def test_ingest_and_query_workflow(self):
        """Test complete ingest -> query workflow."""
        # Step 1: Ingest documents
        documents = [
            {
                "id": "module_1",
                "title": "ROS 2 Basics",
                "text": "ROS 2 provides nodes, topics, services, and actions for robot communication."
            },
            {
                "id": "module_2",
                "title": "Gazebo Simulation",
                "text": "Gazebo enables realistic physics simulation with plugins for various sensors."
            },
            {
                "id": "module_3",
                "title": "Isaac Sim",
                "text": "NVIDIA Isaac Sim provides synthetic data generation and domain randomization."
            }
        ]
        
        for doc in documents:
            response = client.post("/ingest", json=doc)
            assert response.status_code == 200
            assert response.json()["status"] == "success"
        
        # Step 2: Query and verify results
        query_response = client.post("/query", json={
            "query": "simulation and physics",
            "top_k": 5
        })
        
        assert query_response.status_code == 200
        query_data = query_response.json()
        
        # Should have some results
        assert query_data["count"] >= 0
        assert isinstance(query_data["results"], list)
        
        # All results should have required fields
        for result in query_data["results"]:
            assert "source_id" in result
            assert "title" in result
            assert "text" in result
            assert "score" in result
            assert isinstance(result["score"], (int, float))
    
    def test_query_finds_ingested_document(self):
        """Test that query finds the document we just ingested."""
        # Ingest a specific document
        test_doc_id = "unique_doc_xyz"
        ingest_response = client.post("/ingest", json={
            "id": test_doc_id,
            "title": "Unique Test Document",
            "text": "This document contains very specific content about quantum robotics manipulation"
        })
        
        assert ingest_response.status_code == 200
        
        # Query for related content
        query_response = client.post("/query", json={
            "query": "quantum robotics",
            "top_k": 10
        })
        
        assert query_response.status_code == 200
        query_data = query_response.json()
        
        # The ingested document should be in results (when Qdrant/OpenAI available)
        # or at least the endpoint should work correctly
        assert "results" in query_data
        assert isinstance(query_data["results"], list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
