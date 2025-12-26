# Integrated RAG Chatbot for Book Content

This project implements a Retrieval-Augmented Generation (RAG) chatbot that answers user questions about book content, including queries based on user-selected text.

## Features

- Question answering based on book content
- User text selection with context injection
- Source attribution with page numbers
- Conversation history management
- API endpoints for integration

## Technology Stack

- **Backend**: FastAPI (Python)
- **AI/ML**: OpenAI API for embeddings and generation
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon Serverless Postgres
- **Frontend**: React with TypeScript (to be implemented)

## Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Create a `.env` file with your API keys (based on `.env.example`)
5. Run the application:
   ```bash
   uvicorn src.main:app --reload
   ```

## Environment Variables

Required environment variables:

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your Qdrant cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon Postgres connection string

## API Endpoints

- `GET /` - Health check
- `POST /api/chat/query` - Ask a question about book content
- `POST /api/chat/selected-text` - Ask about selected text
- `POST /api/conversations` - Create a conversation
- `GET /api/chat/history/{conversation_id}` - Get conversation history
- `GET /api/books/{book_id}/structure` - Get book structure
- `GET /health` - Health check

## Running with Docker

```bash
docker-compose up --build
```

The API will be available at `http://localhost:8000`.

## Architecture

The application consists of:

1. **Models**: Pydantic models for data validation
2. **Database**: SQLAlchemy models and session management
3. **Services**: Business logic for RAG processing, document handling, etc.
4. **API**: FastAPI routes and endpoints

## RAG Process

1. User submits a question
2. Question is converted to an embedding using OpenAI API
3. Qdrant is searched for relevant document chunks
4. Retrieved context is passed to OpenAI API with the question
5. Response is formatted with source attribution
6. Conversation is stored in the database

## Frontend Integration

The API is designed to be consumed by a React frontend with components for:

- Book viewer with text selection capability
- Chat interface
- Source attribution display
- Conversation history

The frontend implementation is planned in the next phase.