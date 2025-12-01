import os
import json
from typing import Optional, List
from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import anthropic
import openai
import logging
from rag_service import get_rag_service

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Physical AI Backend", version="0.1.0")

# Enable CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize LLM clients
CLAUDE_API_KEY = os.getenv("CLAUDE_CODE_API_KEY", "")
OPENAI_API_KEY = os.getenv("OPENAI_GEMINI_KEY", "")

if CLAUDE_API_KEY:
    claude_client = anthropic.Anthropic(api_key=CLAUDE_API_KEY)
else:
    claude_client = None
    logger.warning("CLAUDE_CODE_API_KEY not set; Claude features disabled")

if OPENAI_API_KEY:
    openai.api_key = OPENAI_API_KEY
else:
    logger.warning("OPENAI_GEMINI_KEY not set; OpenAI features disabled")

# ============== Data Models ==============

class ChatMessage(BaseModel):
    """A single chat message."""
    role: str  # "user" or "assistant"
    content: str

class ChatRequest(BaseModel):
    """Request body for chat endpoint."""
    messages: list[ChatMessage]
    model: str = "claude"  # "claude" or "openai"
    system_prompt: Optional[str] = None

class ChatResponse(BaseModel):
    """Response from chat endpoint."""
    role: str = "assistant"
    content: str
    model: str

class EmbeddingRequest(BaseModel):
    """Request for embedding generation."""
    text: str

class EmbeddingResponse(BaseModel):
    """Response with embeddings."""
    embedding: list[float]
    dimension: int

class RAGSearchRequest(BaseModel):
    """Request for RAG vector search."""
    query: str
    top_k: int = 5

class RAGSearchResult(BaseModel):
    """A single RAG search result."""
    id: str
    text: str
    score: float
    metadata: Optional[dict] = None

class IngestRequest(BaseModel):
    """Request to ingest a document into RAG."""
    id: str
    title: str
    text: str
    session: Optional[bool] = False
    session_id: Optional[str] = None

class IngestResponse(BaseModel):
    """Response from document ingestion."""
    status: str
    doc_id: str
    message: str

class QueryRequest(BaseModel):
    """Request to query RAG system."""
    query: str
    top_k: int = 5
    context_ids: Optional[List[str]] = None
    restrict_to_session: Optional[bool] = False
    session_id: Optional[str] = None

class QueryResult(BaseModel):
    """Single result from RAG query."""
    source_id: str
    title: str
    text: str
    score: float

class QueryResponse(BaseModel):
    """Response from RAG query."""
    query: str
    results: List[QueryResult]
    count: int

# ============== API Endpoints ==============

@app.get("/")
async def root():
    """Health check endpoint."""
    return {
        "name": "Physical AI Textbook Backend",
        "version": "0.1.0",
        "status": "running",
        "endpoints": ["/chat", "/embed", "/rag-search", "/ingest", "/query", "/ws"]
    }

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint using Claude or OpenAI.
    Accepts a list of messages and returns an assistant response.
    """
    if request.model == "claude":
        if not claude_client:
            raise HTTPException(status_code=503, detail="Claude not configured")
        
        # Convert messages to Claude format
        messages = [
            {"role": msg.role, "content": msg.content}
            for msg in request.messages
        ]
        
        # Call Claude API
        response = claude_client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1024,
            system=request.system_prompt or "You are a helpful robotics assistant.",
            messages=messages
        )
        
        return ChatResponse(
            role="assistant",
            content=response.content[0].text,
            model="claude"
        )
    
    elif request.model == "openai":
        if not OPENAI_API_KEY:
            raise HTTPException(status_code=503, detail="OpenAI not configured")
        
        # Convert messages to OpenAI format
        messages = [
            {"role": msg.role, "content": msg.content}
            for msg in request.messages
        ]
        
        # Call OpenAI API
        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            max_tokens=1024,
            system=request.system_prompt or "You are a helpful robotics assistant."
        )
        
        return ChatResponse(
            role="assistant",
            content=response.choices[0].message.content,
            model="openai"
        )
    
    else:
        raise HTTPException(status_code=400, detail="Invalid model")

@app.post("/embed", response_model=EmbeddingResponse)
async def embed(request: EmbeddingRequest) -> EmbeddingResponse:
    """
    Generate embeddings for text using OpenAI.
    Used for semantic search and RAG indexing.
    """
    if not OPENAI_API_KEY:
        raise HTTPException(status_code=503, detail="OpenAI not configured")
    
    # Call OpenAI embeddings API
    response = openai.Embedding.create(
        input=request.text,
        model="text-embedding-3-small"
    )
    
    embedding = response["data"][0]["embedding"]
    
    return EmbeddingResponse(
        embedding=embedding,
        dimension=len(embedding)
    )

@app.post("/rag-search")
async def rag_search(request: RAGSearchRequest) -> dict:
    """
    Placeholder for RAG vector search.
    In a full implementation, this would:
    1. Generate embeddings for the query
    2. Search a vector database (Qdrant)
    3. Return top-k results
    
    For now, returns mock data.
    """
    # TODO: Integrate with Qdrant Cloud
    QDRANT_URL = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
    
    if not QDRANT_URL or not QDRANT_API_KEY:
        logger.warning("Qdrant not configured; returning mock RAG results")
        # Return mock results for demo
        return {
            "query": request.query,
            "results": [
                {
                    "id": "doc_1",
                    "text": "ROS 2 is a middleware for robotics...",
                    "score": 0.95,
                    "metadata": {"module": "1", "section": "Introduction"}
                },
                {
                    "id": "doc_2",
                    "text": "Gazebo is a physics simulator...",
                    "score": 0.88,
                    "metadata": {"module": "2", "section": "Simulation"}
                },
            ],
            "count": 2
        }
    
    # TODO: Real Qdrant integration
    return {"query": request.query, "results": [], "count": 0}

@app.post("/ingest", response_model=IngestResponse)
async def ingest(request: IngestRequest) -> IngestResponse:
    """
    Ingest a document into the RAG system.
    
    Stores document metadata in Postgres and embeddings in Qdrant.
    
    Args:
        id: Unique document identifier
        title: Document title
        text: Document content
    
    Returns:
        Status response with document ID and message
    """
    rag_service = get_rag_service()
    
    success = rag_service.ingest(
        doc_id=request.id,
        title=request.title,
        text=request.text,
        session=request.session,
        session_id=request.session_id
    )
    
    if success:
        return IngestResponse(
            status="success",
            doc_id=request.id,
            message=f"Document '{request.title}' ingested successfully"
        )
    else:
        raise HTTPException(
            status_code=400,
            detail=f"Failed to ingest document '{request.id}'"
        )

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest) -> QueryResponse:
    """
    Query the RAG system for similar documents.
    
    Generates an embedding for the query and returns top-k most similar passages.
    
    Args:
        query: Query string
        top_k: Number of results to return (default 5)
        context_ids: Optional list of source IDs to filter results
    
    Returns:
        Query results with source metadata and similarity scores
    """
    rag_service = get_rag_service()
    
    results = rag_service.query(
        query_text=request.query,
        top_k=request.top_k,
        context_ids=request.context_ids,
        restrict_to_session=bool(request.restrict_to_session),
        session_id=request.session_id
    )
    
    # Convert to response model
    query_results = [
        QueryResult(
            source_id=r["source_id"],
            title=r["title"],
            text=r["text"],
            score=r["score"]
        )
        for r in results
    ]
    
    return QueryResponse(
        query=request.query,
        results=query_results,
        count=len(query_results)
    )

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time chat.
    Clients connect and send/receive messages in real-time.
    """
    await websocket.accept()
    logger.info("WebSocket client connected")
    
    try:
        while True:
            # Receive message from client
            data = await websocket.receive_text()
            message = json.loads(data)
            
            # Echo back with acknowledgment
            response = {
                "status": "received",
                "message": message.get("content", ""),
                "timestamp": str(__import__("datetime").datetime.now())
            }
            
            await websocket.send_json(response)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        await websocket.close()

# ============== Startup/Shutdown ==============

@app.on_event("startup")
async def startup_event():
    """Initialize on app startup."""
    logger.info("Backend started successfully")
    logger.info(f"Claude available: {bool(claude_client)}")
    logger.info(f"OpenAI available: {bool(OPENAI_API_KEY)}")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on app shutdown."""
    logger.info("Backend shutting down")
