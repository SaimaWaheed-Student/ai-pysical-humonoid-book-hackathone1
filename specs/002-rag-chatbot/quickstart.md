# Quickstart Guide: RAG Chatbot Development

## Overview
This quickstart guide will help you get the RAG Chatbot system up and running quickly. The system enables users to ask questions about book content and receive accurate answers with proper citations.

## Prerequisites
Before starting, ensure you have:
- Python 3.11+ installed
- Docker and Docker Compose (for containerization)
- Access to the following APIs:
  - OpenAI API key (for response generation)
  - Cohere API key (for embeddings and reranking)
  - Qdrant Cloud account (for vector storage)
  - Neon Postgres account (for metadata storage)

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd rag-chatbot
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=postgresql://username:password@ep-xxxxxx.us-east-1.aws.neon.tech/dbname?sslmode=require
DEBUG=true
LOG_LEVEL=info
```

## Running the Application

### Option 1: Local Development
```bash
# Run the development server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Option 2: Docker
```bash
# Build and run with Docker
docker-compose up --build
```

The API will be available at `http://localhost:8000`.

## Basic Usage

### 1. Index a Book
To use the RAG chatbot, first index a book:

```bash
curl -X POST "http://localhost:8000/upload-book" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/your/book.pdf" \
  -F "book_title=Your Book Title" \
  -F "author=Author Name"
```

### 2. Ask a Question
Once the book is indexed, ask questions:

```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main concept of this book?",
    "book_id": "your-book-id",
    "session_id": "session-123"
  }'
```

### 3. Ask with User Selection
To ask a question with highlighted text:

```bash
curl -X POST "http://localhost:8000/chat/with-selection" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Can you elaborate on this concept?",
    "selected_text": "The main concept is...",
    "book_id": "your-book-id",
    "session_id": "session-123"
  }'
```

### 4. Save User Selections
To save a user's highlighted text:

```bash
curl -X POST "http://localhost:8000/selections" \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "session-123",
    "selected_text": "Important text that user highlighted",
    "book_id": "your-book-id",
    "chapter": 3,
    "page": 45,
    "start_offset": 100,
    "end_offset": 150
  }'
```

## API Endpoints Overview

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/chat` | Ask a question about book content |
| POST | `/chat/with-selection` | Ask a question with user-selected text context |
| POST | `/upload-book` | Index a new book for the chatbot |
| GET | `/chat/{session_id}/history` | Retrieve chat history for a session |
| POST | `/selections` | Save a user selection |
| GET | `/selections/{session_id}` | Retrieve selections for a session |
| GET | `/health` | Health check endpoint |
| POST | `/search` | Debug vector search |

## Configuration

### Performance Settings
The system is configured for optimal performance:

- **Latency targets**: p95 < 1.5s (monitored automatically)
- **Chunk size**: 256-512 tokens for balance between context and specificity
- **Overlap**: 50-100 tokens between consecutive chunks to preserve continuity
- **Confidence threshold**: 0.7 minimum for direct responses

### Compliance Settings
The system adheres to all RAG Chatbot Governance Framework principles:

- **Accuracy**: 0% hallucination tolerance with citation validation
- **Transparency**: All responses include Chapter X, Page Y citations
- **User Empowerment**: User selections prioritized in retrieval
- **Privacy**: Opt-in data collection with GDPR/CCPA compliance
- **Integrity**: Immutable book content with version control

## Testing

### Run Unit Tests
```bash
pytest tests/unit/
```

### Run Integration Tests
```bash
pytest tests/integration/
```

### Run Accuracy Tests
```bash
pytest tests/accuracy/
```

## Monitoring

The system includes built-in monitoring capabilities:

- **Health endpoint**: `/health` returns system status
- **Metrics**: Access Prometheus-style metrics at `/metrics`
- **Logging**: Structured logs in JSON format to stdout
- **Performance**: Real-time latency monitoring dashboard

## Troubleshooting

### Common Issues

**Issue**: API requests timing out
**Solution**: Check that all required API keys are set correctly in environment variables

**Issue**: "Book not found" errors
**Solution**: Verify the book has been successfully indexed using the `/upload-book` endpoint

**Issue**: Poor response quality
**Solution**: Check that the book content was properly chunked and indexed by reviewing the indexing logs

**Issue**: High latency responses
**Solution**: Check external API availability (OpenAI, Cohere) and Qdrant connection

### Debug Mode
Run with `DEBUG=true` in the environment to enable detailed logging and debugging information.

## Next Steps

1. **Customize**: Adjust the system configuration for your specific book content
2. **Deploy**: Use the provided Docker configuration for production deployment
3. **Monitor**: Set up monitoring and alerting for your production instance
4. **Scale**: Configure horizontal scaling based on your expected traffic