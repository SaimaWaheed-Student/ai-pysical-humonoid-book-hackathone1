# Data Model: RAG Chatbot Development

## Overview
This document defines the data models for the RAG Chatbot Development project, focusing on the entities specified in the feature specification and designed to comply with the RAG Chatbot Governance Framework principles.

## Entity: BookChunk
**Purpose**: Represents segments of book content stored in the vector database with associated metadata.

**Fields**:
- `id` (string): Unique identifier for the chunk (e.g., "ch3_chunk145")
- `book_id` (string): Identifier for the book this chunk belongs to
- `chapter` (int): Chapter number containing this chunk
- `chapter_title` (string): Title of the chapter
- `section` (int): Section number within the chapter
- `section_title` (string): Title of the section
- `page` (int): Starting page number of this chunk
- `page_end` (int): Ending page number of this chunk
- `content` (text): The actual text content of the chunk (256-512 tokens)
- `embedding` (vector): 1024-dimensional embedding vector for semantic search
- `tokens` (int): Number of tokens in the content
- `language` (string): Language code (e.g., "en")
- `timestamp` (datetime): When the chunk was indexed
- `version` (string): Version of the book content (e.g., "1.0")
- `embedding_model` (string): Model used for embedding (e.g., "cohere-embed-3-english-v3.0")
- `is_header` (boolean): Whether this chunk represents a header
- `is_code` (boolean): Whether this chunk contains code snippets
- `is_quote` (boolean): Whether this chunk represents a direct quote
- `importance_score` (float): Score representing the importance of this content (0.0-1.0)

**Relationships**: None (stored in vector database)

**Compliance**: Supports Integrity Principle (version control) and Transparency Principle (citation metadata).

## Entity: Query
**Purpose**: Represents a user's question to the system with metadata for tracking and analysis.

**Fields**:
- `id` (serial/UUID): Unique identifier for the query
- `session_id` (string): Identifier grouping related queries from the same session
- `user_id` (string, optional): Identifier for the user (if authenticated)
- `question` (text): The text of the user's question
- `timestamp` (datetime): When the query was submitted
- `book_id` (string): Identifier of the book being queried
- `selected_text_id` (string, optional): Reference to user-selected text used in this query
- `response_id` (string): Reference to the response generated for this query
- `confidence_score` (float): Confidence score of the response (0.0-1.0)
- `retrieval_time_ms` (int): Time taken for the retrieval process in milliseconds
- `generation_time_ms` (int): Time taken for the generation process in milliseconds
- `total_time_ms` (int): Total processing time in milliseconds
- `is_hallucination_detected` (boolean): Flag if hallucination was detected during response
- `feedback_rating` (int, optional): User rating of the response (1-5)

**Relationships**:
- One-to-one with Response (via response_id)
- One-to-many with UserSelection (via selected_text_id, optional)

**Compliance**: Supports Privacy Principle (opt-in data collection, retention policies) and Accuracy Principle (confidence tracking).

## Entity: Response
**Purpose**: The system's answer to a user query, containing citation metadata and generation details.

**Fields**:
- `id` (string): Unique identifier for the response
- `query_id` (string): Reference to the original query
- `answer` (text): The generated answer text
- `confidence` (float): Overall confidence in the response (0.0-1.0)
- `sources` (JSON): List of source chunks used in generation with citation details:
  ```json
  [
    {
      "chapter": 3,
      "page": 45,
      "section": "RAG Fundamentals",
      "quote": "Exact text from book",
      "relevance": 0.95
    }
  ]
  ```
- `user_context_used` (boolean): Whether user selection was used in generation
- `user_selection_reference` (text): Reference text indicating how user selection was used (e.g., "Based on your highlight")
- `follow_up_questions` (JSON): Suggested follow-up questions as an array of strings
- `citations_validated` (boolean): Whether citations were validated before sending
- `generation_model` (string): Model used for generation (e.g., "gpt-4o")
- `generation_timestamp` (datetime): When the response was generated

**Relationships**:
- Many-to-one with Query (via query_id)
- Many-to-many with BookChunk (via sources)

**Compliance**: Supports Transparency Principle (citations) and Accuracy Principle (validation).

## Entity: UserSelection
**Purpose**: Captures text highlighted by users with complete metadata to support the User Empowerment principle.

**Fields**:
- `id` (serial): Primary key for the selection
- `session_id` (string): Reference to the session where selection was made
- `user_id` (string, optional): Reference to the user who made the selection (if authenticated)
- `selected_text` (text): The exact text that was highlighted by the user
- `chapter` (int): Chapter number where the selection was made
- `page` (int): Page number where the selection was made
- `start_offset` (int): Character offset where selection starts in the text
- `end_offset` (int): Character offset where selection ends in the text
- `created_at` (datetime): When the selection was made
- `is_used_in_query` (boolean): Whether this selection has been used in a query (default: false)
- `query_context_used` (text): Context of how the selection was used in queries
- `book_id` (string): Reference to the book where the selection was made

**Relationships**:
- One-to-many with Query (via selected_text_id in Query table)
- One-to-one with Session (via session_id)

**Compliance**: Supports User Empowerment Principle (selection tracking) and Privacy Principle (opt-in storage).

## Entity: Session
**Purpose**: Represents a user session to maintain conversation context and support privacy requirements.

**Fields**:
- `id` (string): Unique identifier for the session
- `user_id` (string, optional): Reference to the user (if authenticated)
- `book_id` (string): Identifier of the book associated with this session
- `created_at` (datetime): When the session was created
- `last_activity_at` (datetime): When the last activity occurred in this session
- `session_duration` (int, optional): Duration of the session in seconds
- `ended_at` (datetime, optional): When the session ended (null if active)
- `device_info` (JSON, optional): Information about the device used (if opted-in)
- `location` (string, optional): Location information (if opted-in)

**Relationships**:
- One-to-many with Query (via session_id)
- One-to-many with UserSelection (via session_id)

**Compliance**: Supports Privacy Principle (opt-in data) and Integrity Principle (audit trail).

## Entity: AuditLog
**Purpose**: Immutable audit log for tracking all system actions to support the Integrity principle.

**Fields**:
- `id` (serial): Primary key for the log entry
- `timestamp` (datetime): When the action occurred (UTC)
- `user_id` (string, nullable): User identifier for user-initiated actions
- `action` (string): Type of action (QUERY, SELECTION, UPDATE, ERROR, etc.)
- `resource` (string): Resource involved (book_id, collection_name, etc.)
- `details` (JSON): Full context of the action in JSON format
- `status` (string): Result of the action (success, failure)
- `error_message` (text, nullable): Error details if the action failed
- `session_id` (string): Session in which the action occurred

**Example details**:
```json
{
  "timestamp": "2024-01-15T10:30:45.123Z",
  "session_id": "sess_abc123",
  "action": "QUERY",
  "query": "What is RAG?",
  "confidence": 0.87,
  "chunks_returned": 5,
  "latency_ms": 1247,
  "status": "success",
  "user_selected_text": true
}
```

**Relationships**: References user and session tables (foreign keys)

**Compliance**: Supports Integrity Principle (immutable audit trail) and Transparency Principle (action logging).

## Entity: BookContentVersion
**Purpose**: Track different versions of book content as required by the Integrity principle.

**Fields**:
- `id` (string): Unique identifier for this version
- `book_id` (string): Identifier of the book
- `version_number` (string): Semantic version (e.g., "1.0.0")
- `title` (string): Title of the book
- `author` (string): Author of the book
- `publication_date` (date): Original publication date
- `indexed_at` (datetime): When this version was indexed
- `chunk_count` (int): Number of chunks in this version
- `status` (string): Status of this version (active, archived, deprecated)
- `index_collection_name` (string): Name of the Qdrant collection for this version
- `changelog` (text): Description of changes from previous version
- `is_current` (boolean): Whether this is the current active version

**Relationships**: One-to-many with BookChunk (for a specific version)

**Compliance**: Supports Integrity Principle (version control and rollback).

## Database Relationships Diagram
```
Session (1) -- (M) Query (1) -- (1) Response (M) -- (M) BookChunk
    |
    M
UserSelection (M) -- (1) Session
    |
    M
Query (via selected_text_id)

Session/Multiple entities -- (M) AuditLog
BookContentVersion (1) -- (M) BookChunk
```

## Indexing Strategy
1. **Query table**: Index on `session_id`, `timestamp`, `book_id`
2. **UserSelection table**: Index on `session_id`, `user_id`, `created_at`, `book_id`
3. **Session table**: Index on `user_id`, `book_id`, `created_at`
4. **AuditLog table**: Index on `timestamp`, `user_id`, `action`, `session_id`

## Compliance Summary
- **Accuracy Principle**: Confidence tracking in Query/Response; citation validation
- **Transparency Principle**: Citation metadata in BookChunk and Response
- **User Empowerment Principle**: Complete UserSelection tracking with context
- **Performance Principle**: Proper indexing for efficient queries
- **Privacy Principle**: Separation of optional/opt-in data; retention tracking
- **Integrity Principle**: Version control through BookContentVersion; audit logging