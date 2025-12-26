from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class QuestionRequest(BaseModel):
    question: str = Field(..., description="The question to ask the chatbot", example="What is the definition of RAG systems?")
    book_id: str = Field(..., description="Identifier of the book to query", example="the-rag-guide-2024")
    session_id: Optional[str] = Field(None, description="Session identifier to maintain context", example="sess_abc123")
    include_citations: bool = Field(True, description="Whether to include detailed citations (default: true)")


class QuestionWithSelectionRequest(QuestionRequest):
    selected_text: str = Field(..., description="Text highlighted by the user to provide context", example="RAG systems combine retrieval with generation mechanisms")


class Source(BaseModel):
    chapter: int = Field(..., description="Chapter number where the information is found", example=3)
    page: int = Field(..., description="Page number where the information is found", example=45)
    section: Optional[str] = Field(None, description="Section title where the information is found", example="RAG Fundamentals")
    quote: str = Field(..., description="Exact text quoted from the book", example="RAG systems combine retrieval with generation to improve response accuracy.")
    relevance: float = Field(..., description="Relevance score of this source to the question", example=0.95, ge=0.0, le=1.0)


class QuestionResponse(BaseModel):
    answer: str = Field(..., description="The answer to the question with citations", example="RAG systems combine retrieval and generation mechanisms to provide accurate answers based on book content [Ch. 3, p. 45].")
    confidence: float = Field(..., description="Confidence score for the answer", example=0.92, ge=0.0, le=1.0)
    sources: List[Source] = Field(..., description="List of sources used to generate the answer")
    user_context_used: bool = Field(..., description="Whether user selection was used in generating response", example=True)
    user_selection_reference: Optional[str] = Field(None, description="Reference to how user selection was used", example="Based on your highlighted passage")
    follow_up_questions: List[str] = Field(..., description="Suggested follow-up questions", example=["How does chunking work in RAG systems?", "What are the performance benefits of RAG?"])
    citations_validated: bool = Field(..., description="Whether citations were validated", example=True)


class UserSelectionRequest(BaseModel):
    session_id: str = Field(..., description="Session identifier", example="sess_abc123")
    selected_text: str = Field(..., description="The exact text that was highlighted", example="RAG systems combine retrieval with generation mechanisms")
    book_id: str = Field(..., description="Identifier of the book where text was selected", example="the-rag-guide-2024")
    chapter: int = Field(..., description="Chapter number where selection was made", example=3)
    page: int = Field(..., description="Page number where selection was made", example=45)
    start_offset: int = Field(..., description="Character offset where selection starts", example=1234)
    end_offset: int = Field(..., description="Character offset where selection ends", example=1300)


class UserSelectionResponse(BaseModel):
    id: str = Field(..., description="Unique identifier for the saved selection", example="sel_789")
    message: str = Field(..., description="Confirmation message", example="Selection saved successfully")


class UserSelection(BaseModel):
    id: str = Field(..., description="Unique identifier for the selection", example="sel_789")
    selected_text: str = Field(..., description="The text that was selected", example="RAG systems combine retrieval with generation mechanisms")
    chapter: int = Field(..., description="Chapter where selection was made", example=3)
    page: int = Field(..., description="Page where selection was made", example=45)
    created_at: datetime = Field(..., description="When the selection was made", example="2024-01-15T10:30:00Z")


class UserSelectionsResponse(BaseModel):
    selections: List[UserSelection]


class ChatHistoryItem(BaseModel):
    query_id: str = Field(..., description="Unique identifier for the query", example="qry_123")
    query: str = Field(..., description="The original question", example="What is the definition of RAG systems?")
    response: QuestionResponse
    timestamp: datetime = Field(..., description="When the query was made", example="2024-01-15T10:30:00Z")


class ChatHistoryResponse(BaseModel):
    history: List[ChatHistoryItem]
    total_count: int = Field(..., description="Total number of chat items in the session", example=15)
    limit: int = Field(..., description="Number of items returned in this response", example=10)
    offset: int = Field(..., description="Number of items skipped", example=0)


class BookUploadResponse(BaseModel):
    book_id: str = Field(..., description="Unique identifier for the uploaded book", example="the-rag-guide-2024-v2")
    title: str = Field(..., description="Title of the uploaded book", example="Understanding RAG Systems")
    author: str = Field(..., description="Author of the uploaded book", example="Jane Smith")
    version: str = Field(..., description="Version of the book content", example="1.0.0")
    message: str = Field(..., description="Confirmation message", example="Book successfully uploaded and indexed")
    chunk_count: int = Field(..., description="Number of chunks the book was split into", example=150)
    indexing_status: str = Field(..., description="Current status of the indexing process", example="completed")


class SearchRequest(BaseModel):
    query: str = Field(..., description="The search query string", example="What is RAG?")
    book_id: str = Field(..., description="Identifier of the book to search in", example="the-rag-guide-2024")
    top_k: int = Field(5, description="Number of results to return", ge=1, le=100)


class ChunkMetadata(BaseModel):
    id: Optional[str] = Field(None, description="Unique identifier for the chunk", example="ch3_chunk145")
    book_id: str = Field(..., description="Identifier of the book", example="the-rag-guide-2024")
    chapter: int = Field(..., description="Chapter number", example=3)
    chapter_title: Optional[str] = Field(None, description="Title of the chapter", example="Retrieval Mechanisms")
    section: Optional[int] = Field(None, description="Section number", example=2)
    section_title: Optional[str] = Field(None, description="Title of the section", example="Vector Search Optimization")
    page: int = Field(..., description="Page number", example=45)
    page_end: Optional[int] = Field(None, description="Ending page number (if chunk spans multiple pages)", example=46)
    tokens: Optional[int] = Field(None, description="Number of tokens in the chunk", example=412)
    version: Optional[str] = Field(None, description="Version of the book content", example="1.0")


class SearchResult(BaseModel):
    content: str = Field(..., description="The content of the matched chunk", example="RAG (Retrieval-Augmented Generation) systems combine...")
    metadata: ChunkMetadata
    score: float = Field(..., description="Similarity score for the match", example=0.87, ge=0.0, le=1.0)


class SearchResponse(BaseModel):
    query: str = Field(..., description="The original search query", example="What is RAG?")
    results: List[SearchResult]
    query_embedding_time_ms: Optional[int] = Field(None, description="Time taken to generate query embedding in milliseconds", example=120)
    search_time_ms: Optional[int] = Field(None, description="Time taken for vector search in milliseconds", example=85)


class HealthResponse(BaseModel):
    status: str = Field(..., description="Overall health status", example="healthy")
    timestamp: datetime = Field(..., description="When the health check was performed", example="2024-01-15T10:30:00Z")
    services: Optional[Dict[str, str]] = Field(None, description="Health status of individual services", example={"database": "available", "vector_store": "available", "external_apis": "available"})
    version: str = Field(..., description="Current API version", example="1.0.0")


class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error code", example="VALIDATION_ERROR")
    message: str = Field(..., description="Human-readable error message", example="The request was malformed or contained invalid data")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details (optional)", example={"field": "question", "reason": "Question is required and cannot be empty"})


class NotFoundResponse(BaseModel):
    message: str = Field(..., description="Explanation that the content was not found", example="This topic is not covered in Understanding RAG Systems")
    suggestions: List[str] = Field(..., description="Suggested alternative topics", example=["RAG Fundamentals", "Retrieval Mechanisms", "Generation Strategies"])