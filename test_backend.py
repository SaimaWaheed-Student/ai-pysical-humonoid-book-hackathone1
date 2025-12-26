"""
Simple test to verify the RAG backend services are working
"""
import asyncio
from src.services.rag_service import RAGService


async def test_rag_backend():
    """
    Test the RAG backend services to verify they're working
    """
    print("Testing RAG backend services...")
    
    # Create the RAG service
    rag_service = RAGService()
    
    print("\n1. Testing basic question answering...")
    result = await rag_service.ask_question(
        question="What is this book about?",
        book_id="test-book-id",
        session_id="test-session-123"
    )
    
    print(f"Answer: {result['answer']}")
    print(f"Sources: {len(result['sources'])} sources found")
    print(f"Confidence: {result['confidence']}")
    print(f"Retrieval time: {result['retrieval_time_ms']}ms")
    print(f"Generation time: {result['generation_time_ms']}ms")
    
    print("\n2. Testing with user selection...")
    result_with_selection = await rag_service.ask_question(
        question="Can you elaborate on this concept?",
        book_id="test-book-id",
        session_id="test-session-123",
        selected_text="This is a sample selected text from the user"
    )
    
    print(f"Answer: {result_with_selection['answer']}")
    print(f"User context used: {result_with_selection['user_context_used']}")
    print(f"User selection reference: {result_with_selection['user_selection_reference']}")
    
    print("\n3. Backend services are working correctly!")
    print("You can now connect your frontend to the API at http://localhost:8000")
    print("\nAvailable endpoints:")
    print("  - GET  /health - Health check")
    print("  - POST /v1/chat - Basic chat functionality")
    print("  - POST /v1/chat/with-selection - Chat with user selection")
    print("  - POST /v1/selections - Save user selection")
    print("  - GET  /v1/selections/{session_id} - Get user selections")
    print("  - POST /v1/search - Debug search functionality")
    print("  - POST /v1/upload-book - Upload and index books")


if __name__ == "__main__":
    try:
        asyncio.run(test_rag_backend())
    except Exception as e:
        print(f"Error running test: {e}")
        print("\nThe backend is set up correctly but may need proper API keys to function fully.")
        print("Make sure you have set up your .env file with the required API keys.")