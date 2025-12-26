"""
Comprehensive test suite for the RAG Chatbot
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from src.main import app
from src.services.rag_service import RAGService
from src.api.models import QuestionRequest, QuestionWithSelectionRequest
import os


def test_health_endpoint():
    """
    Test the health endpoint
    """
    with TestClient(app) as client:
        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "version" in data


def test_chat_endpoint():
    """
    Test the chat endpoint
    """
    with TestClient(app) as client:
        # Test basic chat functionality
        chat_data = {
            "question": "What is this book about?",
            "book_id": "test-book-id",
            "session_id": "test-session-123"
        }

        response = client.post("/v1/chat", json=chat_data)
        assert response.status_code == 200

        data = response.json()
        assert "answer" in data
        assert "confidence" in data
        assert "sources" in data
        assert isinstance(data["sources"], list)


def test_chat_with_selection_endpoint():
    """
    Test the chat with selection endpoint
    """
    with TestClient(app) as client:
        # Test chat with selection functionality
        chat_data = {
            "question": "Can you elaborate on this concept?",
            "selected_text": "Sample selected text for testing",
            "book_id": "test-book-id",
            "session_id": "test-session-123"
        }

        response = client.post("/v1/chat/with-selection", json=chat_data)
        assert response.status_code == 200

        data = response.json()
        assert "answer" in data
        assert "confidence" in data
        assert "sources" in data
        assert data["user_context_used"] is True


def test_selections_endpoint():
    """
    Test the selections endpoint
    """
    with TestClient(app) as client:
        # Test saving a selection
        selection_data = {
            "session_id": "test-session-123",
            "selected_text": "Sample selected text for testing",
            "book_id": "test-book-id",
            "chapter": 1,
            "page": 10,
            "start_offset": 0,
            "end_offset": 50
        }

        response = client.post("/v1/selections", json=selection_data)
        assert response.status_code == 200

        data = response.json()
        assert "id" in data
        assert "message" in data


def test_search_endpoint():
    """
    Test the search endpoint
    """
    with TestClient(app) as client:
        # Test search functionality
        search_data = {
            "query": "What is this book about?",
            "book_id": "test-book-id"
        }

        response = client.post("/v1/search", json=search_data)
        assert response.status_code == 200

        data = response.json()
        assert "query" in data
        assert "results" in data
        assert data["query"] == search_data["query"]


def test_rag_service():
    """
    Test the RAG service directly
    """
    rag_service = RAGService()

    # Test asking a question
    result = asyncio.run(
        rag_service.ask_question(
            question="What is this book about?",
            book_id="test-book-id",
            session_id="test-session-123"
        )
    )

    assert "answer" in result
    assert "confidence" in result
    assert "sources" in result


def test_upload_book_endpoint():
    """
    Test the upload book endpoint with a mock file
    """
    with TestClient(app) as client:
        # Create a simple text file for upload testing
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as temp_file:
            temp_file.write("This is a test book to verify the upload functionality of our RAG chatbot.")
            temp_file_path = temp_file.name

        try:
            with open(temp_file_path, 'rb') as file:
                # Send the file and form data correctly
                response = client.post(
                    "/v1/upload-book",
                    files={"file": (os.path.basename(temp_file_path), file, "text/plain")},
                    data={
                        "book_title": "Test Book",
                        "author": "Test Author",
                        "publication_date": "2023-01-01",
                        "version": "1.0.0"
                    }
                )

                # The test should expect a 200 response now that we fixed the error
                assert response.status_code == 200

                data = response.json()
                assert "book_id" in data
                assert "message" in data
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)


def run_all_tests():
    """
    Run all tests
    """
    print("Running RAG Chatbot tests...")

    test_health_endpoint()
    print("PASSED: Health endpoint test passed")

    test_chat_endpoint()
    print("PASSED: Chat endpoint test passed")

    test_chat_with_selection_endpoint()
    print("PASSED: Chat with selection endpoint test passed")

    test_selections_endpoint()
    print("PASSED: Selections endpoint test passed")

    test_search_endpoint()
    print("PASSED: Search endpoint test passed")

    test_rag_service()
    print("PASSED: RAG service test passed")

    test_upload_book_endpoint()
    print("PASSED: Upload book endpoint test passed")

    print("\nSUCCESS: All RAG Chatbot tests passed successfully!")


if __name__ == "__main__":
    run_all_tests()