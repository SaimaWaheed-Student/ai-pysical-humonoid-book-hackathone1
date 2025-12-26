from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

class BookCreate(BaseModel):
    title: str
    author: Optional[str] = None
    total_pages: Optional[int] = None

class BookResponse(BaseModel):
    id: str
    title: str
    author: Optional[str]
    total_pages: Optional[int]
    created_at: datetime

    class Config:
        orm_mode = True

class ChapterCreate(BaseModel):
    book_id: str
    chapter_number: int
    title: str
    start_page: int
    end_page: int

class ChapterResponse(BaseModel):
    id: str
    book_id: str
    chapter_number: int
    title: str
    start_page: int
    end_page: int
    created_at: datetime

    class Config:
        orm_mode = True

class DocumentChunkCreate(BaseModel):
    book_id: str
    chapter_id: str
    chunk_index: int
    content: str
    page_number: int
    qdrant_point_id: str
    token_count: int

class DocumentChunkResponse(BaseModel):
    id: str
    book_id: str
    chapter_id: str
    chunk_index: int
    content: str
    page_number: int
    qdrant_point_id: str
    token_count: int
    created_at: datetime

    class Config:
        orm_mode = True

class ConversationCreate(BaseModel):
    user_id: Optional[str] = None
    book_id: str
    title: Optional[str] = None

class ConversationResponse(BaseModel):
    id: str
    user_id: Optional[str]
    book_id: str
    title: Optional[str]
    created_at: datetime
    updated_at: datetime

    class Config:
        orm_mode = True

class MessageCreate(BaseModel):
    conversation_id: str
    role: str
    content: str
    sources: Optional[Dict[str, Any]] = None

class MessageResponse(BaseModel):
    id: str
    conversation_id: str
    role: str
    content: str
    sources: Optional[Dict[str, Any]]
    created_at: datetime

    class Config:
        orm_mode = True

class UserSelectionCreate(BaseModel):
    conversation_id: str
    selected_text: str
    selection_metadata: Optional[Dict[str, Any]] = None

class UserSelectionResponse(BaseModel):
    id: str
    conversation_id: str
    selected_text: str
    selection_metadata: Optional[Dict[str, Any]]
    created_at: datetime

    class Config:
        orm_mode = True

class QueryRequest(BaseModel):
    question: str
    book_id: str
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = None
    filters: Optional[Dict[str, Any]] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    conversation_id: str

class SelectedTextQueryRequest(BaseModel):
    selected_text: str
    question: str
    conversation_id: Optional[str] = None
    book_id: str

class Source(BaseModel):
    chunk_id: str
    content: str
    page: int
    chapter: Optional[str] = None

class ChunkMetadata(BaseModel):
    chunk_id: str
    book_id: str
    chapter_id: str
    content: str
    page_number: int
    qdrant_point_id: str
    token_count: int
    created_at: datetime