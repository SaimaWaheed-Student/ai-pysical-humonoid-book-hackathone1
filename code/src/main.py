from fastapi import FastAPI, Depends, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
import os
import uuid
from typing import Optional
from database.database import get_async_db
from models import (
    QueryRequest, QueryResponse, SelectedTextQueryRequest,
    BookCreate, BookResponse, ChapterCreate, ChapterResponse,
    DocumentChunkCreate, DocumentChunkResponse, ConversationCreate,
    ConversationResponse, MessageCreate, MessageResponse, UserSelectionCreate,
    UserSelectionResponse
)
from services.rag_service import RAGService, DocumentProcessingService, QdrantService

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for a Retrieval-Augmented Generation chatbot embedded in book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure this properly for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.post("/api/chat/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Handle a user query about book content
    """
    try:
        # Perform RAG query
        result = await RAGService.perform_rag_query(
            question=request.question,
            book_id=request.book_id,
            selected_text=request.selected_text,
            conversation_id=request.conversation_id
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/api/chat/selected-text", response_model=QueryResponse)
async def query_selected_text_endpoint(request: SelectedTextQueryRequest):
    """
    Handle a query about selected text
    """
    try:
        # Perform RAG query with selected text context
        result = await RAGService.perform_rag_query(
            question=request.question,
            book_id=request.book_id,
            selected_text=request.selected_text,
            conversation_id=request.conversation_id
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

# Ingestion endpoints
@app.post("/api/ingest/upload")
async def upload_book_endpoint():
    """
    Upload book content (placeholder - implement file upload)
    """
    # This endpoint would require file upload implementation
    # For now, returning a placeholder response
    return {"message": "Upload endpoint - implement file upload logic", "job_id": str(uuid.uuid4())}

@app.post("/api/ingest/process")
async def process_book_endpoint(background_tasks: BackgroundTasks):
    """
    Process uploaded book content (placeholder - implement document processing)
    """
    # This endpoint would process the uploaded book
    # For now, returning a placeholder response
    return {"message": "Processing started", "chunks_created": 0, "embeddings_generated": 0}

@app.get("/api/ingest/status/{job_id}")
async def get_ingestion_status_endpoint(job_id: str):
    """
    Check processing status
    """
    # This endpoint would return the status of the ingestion job
    # For now, returning a placeholder response
    return {"job_id": job_id, "status": "completed", "progress": 100}

# Conversation endpoints
@app.post("/api/conversations", response_model=ConversationResponse)
async def create_conversation_endpoint(conversation: ConversationCreate, db: AsyncSession = Depends(get_async_db)):
    """
    Create a new conversation
    """
    # Implementation for creating conversations
    conversation_id = str(uuid.uuid4())
    return ConversationResponse(
        id=conversation_id,
        user_id=conversation.user_id,
        book_id=conversation.book_id,
        title=conversation.title or "New Conversation",
        created_at=None,
        updated_at=None
    )

@app.get("/api/conversations/{conversation_id}")
async def get_conversation_endpoint(conversation_id: str, db: AsyncSession = Depends(get_async_db)):
    """
    Get conversation details
    """
    # Implementation for getting conversations
    return {"conversation_id": conversation_id, "messages": []}

@app.get("/api/chat/history/{conversation_id}")
async def get_chat_history_endpoint(conversation_id: str, db: AsyncSession = Depends(get_async_db)):
    """
    Get chat history for a conversation
    """
    # Implementation for retrieving chat history
    return {"conversation_id": conversation_id, "messages": []}

# Book metadata endpoints
@app.get("/api/books/{book_id}/structure")
async def get_book_structure_endpoint(book_id: str):
    """
    Get book structure (chapters, sections, etc.)
    """
    # Implementation for returning book structure
    return {"book_id": book_id, "chapters": [], "sections": []}

@app.get("/api/books/{book_id}/search")
async def search_book_endpoint(book_id: str, query: str):
    """
    Full-text search in book content
    """
    # Implementation for searching book content
    return {"book_id": book_id, "query": query, "results": []}

# Health check endpoint
@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": str(uuid.uuid4())}

# Run the application with uvicorn if this file is executed directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )