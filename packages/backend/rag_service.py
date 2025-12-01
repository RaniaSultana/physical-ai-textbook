"""RAG service: embeddings, vector storage (Qdrant), and document management."""

import os
import logging
from typing import Optional, List, Dict, Any
import psycopg2
from psycopg2 import sql
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams, PointStruct

logger = logging.getLogger(__name__)


class RAGService:
    """Manages document ingestion, embeddings, and vector search."""
    
    def __init__(self):
        """Initialize RAG service with Qdrant and Postgres."""
        self.openai_key = os.getenv("OPENAI_GEMINI_KEY", "")
        self.neon_db_url = os.getenv("NEON_DB_URL", "")
        self.qdrant_url = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY", "")
        
        self.embedding_model = "text-embedding-3-small"
        self.embedding_dim = 1536  # Dimension for text-embedding-3-small
        self.collection_name = "documents"
        
        if self.openai_key:
            openai.api_key = self.openai_key
        
        self.qdrant_client = None
        self.db_conn = None
        
        self._init_qdrant()
        self._init_postgres()
    
    def _init_qdrant(self) -> bool:
        """Initialize Qdrant client and collection."""
        if not self.qdrant_url:
            logger.warning("QDRANT_URL not set; Qdrant features disabled")
            return False
        
        try:
            self.qdrant_client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key if self.qdrant_api_key else None
            )
            
            # Check if collection exists
            try:
                self.qdrant_client.get_collection(self.collection_name)
                logger.info(f"✓ Connected to existing Qdrant collection '{self.collection_name}'")
            except Exception:
                # Create collection if it doesn't exist
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.embedding_dim,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"✓ Created Qdrant collection '{self.collection_name}'")
            
            return True
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant: {e}")
            return False
    
    def _init_postgres(self) -> bool:
        """Initialize Postgres connection and create sources table."""
        if not self.neon_db_url:
            logger.warning("NEON_DB_URL not set; Postgres features disabled")
            return False
        
        try:
            self.db_conn = psycopg2.connect(self.neon_db_url)
            cursor = self.db_conn.cursor()
            
            # Create sources table if it doesn't exist
            create_table_sql = """
            CREATE TABLE IF NOT EXISTS sources (
                id TEXT PRIMARY KEY,
                title TEXT NOT NULL,
                text TEXT NOT NULL,
                created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
            );
            """
            
            cursor.execute(create_table_sql)
            self.db_conn.commit()
            cursor.close()
            
            logger.info("✓ Connected to Neon Postgres; sources table ready")
            return True
        except psycopg2.OperationalError as e:
            logger.error(f"Failed to connect to Postgres: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize Postgres: {e}")
            return False
    
    def _get_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for text using OpenAI."""
        if not self.openai_key:
            logger.error("OpenAI key not configured; cannot generate embeddings")
            return None
        
        try:
            response = openai.Embedding.create(
                input=text,
                model=self.embedding_model
            )
            return response["data"][0]["embedding"]
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            return None
    
    def ingest(self, doc_id: str, title: str, text: str) -> bool:
        """
        Ingest a document into RAG system.
        
        1. Store metadata in Postgres
        2. Generate embedding
        3. Upsert vector to Qdrant with metadata
        
        Args:
            doc_id: Unique document ID
            title: Document title
            text: Document content
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Generate embedding
            embedding = self._get_embedding(text)
            if embedding is None:
                logger.error(f"Failed to embed document {doc_id}")
                return False
            
            # Store in Postgres
            if self.db_conn:
                cursor = self.db_conn.cursor()
                insert_sql = """
                INSERT INTO sources (id, title, text) VALUES (%s, %s, %s)
                ON CONFLICT (id) DO UPDATE SET title = EXCLUDED.title, text = EXCLUDED.text;
                """
                cursor.execute(insert_sql, (doc_id, title, text))
                self.db_conn.commit()
                cursor.close()
                logger.info(f"✓ Stored document {doc_id} in Postgres")
            
            # Upsert to Qdrant
            if self.qdrant_client:
                # Create point with embedding and metadata
                point = PointStruct(
                    id=hash(doc_id) % (2**31),  # Use hash for numeric ID
                    vector=embedding,
                    payload={
                        "source_id": doc_id,
                        "title": title,
                        "text": text[:500]  # Store truncated text in metadata
                    }
                )
                
                self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=[point]
                )
                logger.info(f"✓ Upserted document {doc_id} to Qdrant")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to ingest document {doc_id}: {e}")
            return False
    
    def query(
        self,
        query_text: str,
        top_k: int = 5,
        context_ids: Optional[List[str]] = None
    ) -> List[Dict[str, Any]]:
        """
        Query RAG system and return top-k passages.
        
        Args:
            query_text: Query string
            top_k: Number of results to return
            context_ids: Optional list of source IDs to filter by
        
        Returns:
            List of result dicts with keys: source_id, title, text, score
        """
        results = []
        
        try:
            # Generate query embedding
            query_embedding = self._get_embedding(query_text)
            if query_embedding is None:
                logger.error("Failed to embed query")
                return results
            
            # Search Qdrant
            if self.qdrant_client:
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k,
                    with_payload=True
                )
                
                # Process results
                for hit in search_results:
                    source_id = hit.payload.get("source_id")
                    
                    # Filter by context_ids if provided
                    if context_ids and source_id not in context_ids:
                        continue
                    
                    results.append({
                        "source_id": source_id,
                        "title": hit.payload.get("title"),
                        "text": hit.payload.get("text"),
                        "score": hit.score
                    })
                
                logger.info(f"✓ Query returned {len(results)} results from Qdrant")
            
            # If no results and Postgres available, return all matching source_ids
            if not results and self.db_conn:
                cursor = self.db_conn.cursor()
                select_sql = "SELECT id, title, text FROM sources LIMIT %s;"
                cursor.execute(select_sql, (top_k,))
                rows = cursor.fetchall()
                cursor.close()
                
                for row in rows:
                    results.append({
                        "source_id": row[0],
                        "title": row[1],
                        "text": row[2][:500],  # Truncate for response
                        "score": 0.5  # Default score for non-vector results
                    })
            
            return results
        
        except Exception as e:
            logger.error(f"Failed to query RAG: {e}")
            return results
    
    def close(self):
        """Close database connections."""
        if self.db_conn:
            self.db_conn.close()
            logger.info("Closed Postgres connection")


# Singleton instance
_rag_service: Optional[RAGService] = None


def get_rag_service() -> RAGService:
    """Get or create RAG service singleton."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
