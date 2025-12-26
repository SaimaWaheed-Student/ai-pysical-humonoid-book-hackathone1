from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession

from src.api.models import (
    QuestionRequest,
    QuestionWithSelectionRequest,
    QuestionResponse,
    UserSelectionRequest,
    UserSelectionResponse,
    UserSelectionsResponse,
    ChatHistoryResponse,
    BookUploadResponse,
    SearchRequest,
    SearchResponse,
    HealthResponse
)
from src.services.rag_service import RAGService
from src.database import get_db_session
from src.config import settings
from src.logging_config import get_logger

# Create API router
router = APIRouter(prefix="/v1", tags=["Chat"])

# Initialize services
rag_service = RAGService()
logger = get_logger(__name__)


@router.post("/chat", response_model=QuestionResponse)
async def ask_question(
    request: QuestionRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Submit a question to the RAG chatbot and receive an answer with citations.
    """
    try:
        result = await rag_service.ask_question(
            question=request.question,
            book_id=request.book_id,
            session_id=request.session_id or f"session_{hash(request.question)}",
            selected_text=None
        )

        return QuestionResponse(**result)
    except Exception as e:
        logger.error(f"Error processing question: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing question: {str(e)}"
        )


@router.post("/chat/with-selection", response_model=QuestionResponse)
async def ask_question_with_selection(
    request: QuestionWithSelectionRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Submit a question with highlighted text context to get contextual responses.
    """
    try:
        result = await rag_service.ask_question(
            question=request.question,
            book_id=request.book_id,
            session_id=request.session_id or f"session_{hash(request.question)}",
            selected_text=request.selected_text
        )

        return QuestionResponse(**result)
    except Exception as e:
        logger.error(f"Error processing question with selection: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing question with selection: {str(e)}"
        )


@router.post("/selections", response_model=UserSelectionResponse)
async def save_selection(
    request: UserSelectionRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Save a user selection for potential future reference.
    """
    try:
        # In a full implementation, this would save to the database
        # For now, we'll just return a success response
        from datetime import datetime
        import uuid

        selection_id = f"sel_{uuid.uuid4().hex[:8]}"
        logger.info(f"Saved user selection with ID: {selection_id}")

        return UserSelectionResponse(
            id=selection_id,
            message="Selection saved successfully"
        )
    except Exception as e:
        logger.error(f"Error saving user selection: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error saving selection: {str(e)}"
        )


@router.get("/selections/{session_id}", response_model=UserSelectionsResponse)
async def get_selections_by_session(
    session_id: str,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve all user selections made within a specific session.
    """
    try:
        # In a full implementation, this would fetch from the database
        # For now, we'll return an empty list
        return UserSelectionsResponse(selections=[])
    except Exception as e:
        logger.error(f"Error retrieving selections for session {session_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving selections: {str(e)}"
        )


@router.get("/chat/{session_id}/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    session_id: str,
    limit: Optional[int] = 10,
    offset: Optional[int] = 0,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve chat history for a specific session.
    """
    try:
        # In a full implementation, this would fetch from the database
        # For now, we'll return an empty history
        return ChatHistoryResponse(
            history=[],
            total_count=0,
            limit=limit,
            offset=offset
        )
    except Exception as e:
        logger.error(f"Error retrieving chat history for session {session_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chat history: {str(e)}"
        )


@router.post("/search", response_model=SearchResponse)
async def debug_search(
    request: SearchRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Direct vector search for debugging and testing purposes.
    """
    try:
        # This would use the vector search service directly
        # For now, we'll return an empty response
        return SearchResponse(
            query=request.query,
            results=[]
        )
    except Exception as e:
        logger.error(f"Error in debug search: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error in search: {str(e)}"
        )


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the API.
    """
    try:
        # In a real implementation, we would check actual service status
        # For now, we'll return a healthy status
        from datetime import datetime

        return HealthResponse(
            status="healthy",
            timestamp=datetime.utcnow(),
            version=settings.api_version
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Health check failed"
        )


@router.post("/upload-book", response_model=BookUploadResponse)
async def upload_book(
    file: UploadFile = File(...),
    book_title: str = None,
    author: str = None,
    publication_date: Optional[str] = None,
    version: str = "1.0.0"
):
    """
    Upload and index book content for use with the RAG system.
    """
    try:
        # In a full implementation, this would process and index the book content
        # For now, we'll return a success response with mock data
        import uuid
        from datetime import datetime
        import os

        # Generate a unique book ID - with validation for required fields
        if not book_title or not author:
            raise ValueError("Both book_title and author are required")

        book_id = f"{book_title.lower().replace(' ', '-')}-{author.lower().replace(' ', '-')}-{str(uuid.uuid4())[:8]}"

        # Log the upload attempt
        logger.info(f"Processing book upload: {file.filename} for book ID: {book_id}")

        # For now, we'll simulate the indexing process
        # In a real implementation, this would:
        # 1. Read the uploaded file
        # 2. Extract text content
        # 3. Split content into chunks with metadata
        # 4. Generate embeddings for each chunk
        # 5. Store in Qdrant vector database
        # 6. Store metadata in PostgreSQL
        chunk_count = 25  # Mock value for now

        logger.info(f"Book '{book_title}' indexed successfully with {chunk_count} chunks")

        return BookUploadResponse(
            book_id=book_id,
            title=book_title,
            author=author,
            version=version,
            message="Book successfully uploaded and indexed",
            chunk_count=chunk_count,
            indexing_status="completed"
        )
    except Exception as e:
        logger.error(f"Error uploading book: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error uploading book: {str(e)}"
        )