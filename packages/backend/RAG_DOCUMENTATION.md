"""
Documentation for RAG (Retrieval-Augmented Generation) Microservice

This document outlines the new RAG endpoints and integration with Qdrant and Neon Postgres.
"""

# RAG ENDPOINTS
# =============

# POST /ingest
# Request body:
# {
#   "id": "doc_001",           # Unique document ID
#   "title": "Document Title", # Document title
#   "text": "Document content" # Full text content
# }
# Response: 200 OK
# {
#   "status": "success",
#   "doc_id": "doc_001",
#   "message": "Document 'Document Title' ingested successfully"
# }

# POST /query
# Request body:
# {
#   "query": "search query",           # Query string
#   "top_k": 5,                        # Number of results (default: 5)
#   "context_ids": ["doc_001", "doc_002"]  # Optional: filter by source IDs
# }
# Response: 200 OK
# {
#   "query": "search query",
#   "results": [
#     {
#       "source_id": "doc_001",
#       "title": "Document Title",
#       "text": "Relevant passage from document...",
#       "score": 0.95
#     }
#   ],
#   "count": 1
# }

# DATABASE SCHEMA
# ===============

# Neon Postgres: sources table
# CREATE TABLE sources (
#   id TEXT PRIMARY KEY,
#   title TEXT NOT NULL,
#   text TEXT NOT NULL,
#   created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
# );

# Qdrant: documents collection
# - Vector size: 1536 (from text-embedding-3-small)
# - Distance metric: COSINE
# - Metadata: source_id, title, text (truncated)

# WORKFLOW
# ========

# 1. Ingest Document:
#    - POST /ingest with {id, title, text}
#    - Generate embedding using OpenAI text-embedding-3-small
#    - Store document metadata in Postgres sources table
#    - Upsert vector + metadata to Qdrant

# 2. Query Documents:
#    - POST /query with {query, top_k, optional context_ids}
#    - Generate embedding for query using OpenAI
#    - Search Qdrant for top-k similar vectors (COSINE distance)
#    - Return results with source_id, title, text, and similarity score
#    - Optional: filter results by context_ids if provided

# ENVIRONMENT VARIABLES
# ======================

# OPENAI_GEMINI_KEY: OpenAI API key (required for embeddings)
# QDRANT_URL: Qdrant instance URL (e.g., http://localhost:6333)
# QDRANT_API_KEY: Qdrant API key (optional, depends on deployment)
# NEON_DB_URL: Neon Postgres connection string (optional, for persistence)

# FEATURES
# ========

# ✓ Document ingestion with automatic embedding generation
# ✓ Vector search with Qdrant (COSINE similarity)
# ✓ Document metadata storage in Postgres
# ✓ Configurable top-k results
# ✓ Optional filtering by document source IDs
# ✓ Graceful fallback when external services unavailable
# ✓ Comprehensive error handling and logging
# ✓ Upsert behavior (update if document ID already exists)

# TESTING
# =======

# Test classes in test_rag.py:
# - TestRAGEndpoints: Unit tests for /ingest and /query endpoints
# - TestRAGIntegration: Integration tests for complete workflow

# Test scenarios:
# 1. Ingest document with all fields → 200 OK
# 2. Ingest with missing fields → 422 Validation Error
# 3. Query returns results in correct format
# 4. Query with context_ids filter works correctly
# 5. Query with custom top_k parameter respected
# 6. Duplicate document IDs update existing documents
# 7. Empty query results handled gracefully
# 8. Complete ingest → query workflow functions end-to-end

# FILES
# =====

# New/Modified files:
# - packages/backend/main.py (updated with /ingest and /query endpoints)
# - packages/backend/rag_service.py (new: RAGService class)
# - packages/backend/init_db.py (new: database initialization)
# - packages/backend/tests/test_rag.py (new: comprehensive tests)
# - packages/backend/requirements.txt (updated with psycopg2-binary, httpx)
