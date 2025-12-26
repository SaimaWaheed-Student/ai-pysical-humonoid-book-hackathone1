from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, Float, JSON
from sqlalchemy.sql import func
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime

Base = declarative_base()


class BookChunk(Base):
    """
    Represents segments of book content stored in the vector database with associated metadata.
    """
    __tablename__ = "book_chunks"

    id = Column(String, primary_key=True, index=True)
    book_id = Column(String, index=True)
    chapter = Column(Integer)
    chapter_title = Column(String)
    section = Column(Integer)
    section_title = Column(String)
    page = Column(Integer)
    page_end = Column(Integer)
    content = Column(Text)
    # Note: embedding is stored in vector database (Qdrant), not in SQL
    tokens = Column(Integer)
    language = Column(String, default="en")
    timestamp = Column(DateTime, default=func.now())
    version = Column(String)
    embedding_model = Column(String)
    is_header = Column(Boolean, default=False)
    is_code = Column(Boolean, default=False)
    is_quote = Column(Boolean, default=False)
    importance_score = Column(Float)


class Query(Base):
    """
    Represents a user's question to the system with metadata for tracking and analysis.
    """
    __tablename__ = "queries"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    session_id = Column(String, index=True)
    user_id = Column(String, index=True, nullable=True)  # Optional, only if user authenticated
    question = Column(Text)
    timestamp = Column(DateTime, default=func.now(), index=True)
    book_id = Column(String, index=True)
    selected_text_id = Column(String, nullable=True)  # Reference to UserSelection
    response_id = Column(String)  # Reference to Response
    confidence_score = Column(Float)
    retrieval_time_ms = Column(Integer)
    generation_time_ms = Column(Integer)
    total_time_ms = Column(Integer)
    is_hallucination_detected = Column(Boolean, default=False)
    feedback_rating = Column(Integer, nullable=True)  # 1-5 scale


class Response(Base):
    """
    The system's answer to a user query, containing citation metadata and generation details.
    """
    __tablename__ = "responses"

    id = Column(String, primary_key=True, index=True)
    query_id = Column(String, index=True)  # Reference to Query
    answer = Column(Text)
    confidence = Column(Float)
    sources = Column(JSON)  # List of source chunks used with citation details
    user_context_used = Column(Boolean, default=False)
    user_selection_reference = Column(Text)  # Reference text indicating how user selection was used
    follow_up_questions = Column(JSON)  # Suggested follow-up questions
    citations_validated = Column(Boolean, default=False)
    generation_model = Column(String)
    generation_timestamp = Column(DateTime, default=func.now())


class UserSelection(Base):
    """
    Captures text highlighted by users with complete metadata to support the User Empowerment principle.
    """
    __tablename__ = "user_selections"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    session_id = Column(String, index=True)
    user_id = Column(String, index=True, nullable=True)  # Optional if user not authenticated
    selected_text = Column(Text)
    chapter = Column(Integer)
    page = Column(Integer)
    start_offset = Column(Integer)
    end_offset = Column(Integer)
    created_at = Column(DateTime, default=func.now())
    is_used_in_query = Column(Boolean, default=False)
    query_context_used = Column(Text)  # Context of how the selection was used in queries
    book_id = Column(String, index=True)


class Session(Base):
    """
    Represents a user session to maintain conversation context and support privacy requirements.
    """
    __tablename__ = "sessions"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True, nullable=True)  # Optional if user not authenticated
    book_id = Column(String, index=True)
    created_at = Column(DateTime, default=func.now())
    last_activity_at = Column(DateTime, default=func.now(), onupdate=func.now())
    session_duration = Column(Integer, nullable=True)  # Duration in seconds
    ended_at = Column(DateTime, nullable=True)  # When the session ended, null if active
    device_info = Column(JSON, nullable=True)  # Device information if opted-in
    location = Column(String, nullable=True)  # Location information if opted-in


class AuditLog(Base):
    """
    Immutable audit log for tracking all system actions to support the Integrity principle.
    """
    __tablename__ = "audit_log"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    timestamp = Column(DateTime, default=func.now(), index=True)  # UTC timestamp
    user_id = Column(String, index=True, nullable=True)  # User ID for user-initiated actions
    action = Column(String, index=True)  # Type of action (QUERY, SELECTION, UPDATE, ERROR, etc.)
    resource = Column(String, index=True)  # Resource involved (book_id, collection_name, etc.)
    details = Column(JSON)  # Full context of the action in JSON format
    status = Column(String)  # Result of the action (success, failure)
    error_message = Column(Text, nullable=True)  # Error details if action failed
    session_id = Column(String, index=True)  # Session in which action occurred


class BookContentVersion(Base):
    """
    Track different versions of book content as required by the Integrity principle.
    """
    __tablename__ = "book_content_versions"

    id = Column(String, primary_key=True, index=True)
    book_id = Column(String, index=True)
    version_number = Column(String)  # Semantic version (e.g., "1.0.0")
    title = Column(String)
    author = Column(String)
    publication_date = Column(DateTime)
    indexed_at = Column(DateTime, default=func.now())
    chunk_count = Column(Integer)  # Number of chunks in this version
    status = Column(String)  # Status: active, archived, deprecated
    index_collection_name = Column(String)  # Qdrant collection name for this version
    changelog = Column(Text)  # Description of changes from previous version
    is_current = Column(Boolean, default=False)  # Whether this is the current active version