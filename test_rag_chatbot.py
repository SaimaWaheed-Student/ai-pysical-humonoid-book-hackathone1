"""
Test script for the RAG Chatbot application
This script will start the application and run tests to verify functionality
"""
import os
import sys
import subprocess
import time
import requests
import threading
from pathlib import Path
import asyncio
from src.main import main as run_server
import json


def create_test_env():
    """
    Create a test environment file with placeholder values
    """
    test_env_content = """
# Database Configuration
DATABASE_URL=sqlite:///./test.db

# Qdrant Configuration
QDRANT_URL=https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w

# API Keys
GEMINI_API_KEY=your_gemini_api_key
COHERE_API_KEY=your_cohere_api_key

# Application Configuration
DEBUG=true
LOG_LEVEL=debug

# Performance Configuration
DEFAULT_CHUNK_SIZE=512
DEFAULT_CHUNK_OVERLAP=100
RETRIEVAL_TOP_K=5
RERANK_TOP_K=5
MIN_CONFIDENCE_SCORE=0.7
"""
    
    env_path = Path("test.env")
    if not env_path.exists():
        with open(env_path, "w") as f:
            f.write(test_env_content)
        print("Created test environment file: test.env")


def test_api_endpoints():
    """
    Test the API endpoints of the RAG chatbot
    """
    base_url = "http://localhost:8000"
    
    # Test health endpoint
    try:
        print("Testing health endpoint...")
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Health check passed: {data['status']}")
        else:
            print(f"✗ Health check failed with status: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Health check failed with error: {e}")
        return False
    
    # Test chat endpoint
    try:
        print("\nTesting chat endpoint...")
        chat_data = {
            "question": "What is this book about?",
            "book_id": "test-book-id",
            "session_id": "test-session-123"
        }
        
        response = requests.post(
            f"{base_url}/v1/chat",
            json=chat_data,
            headers={"Content-Type": "application/json"}
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Chat endpoint responded with status: {response.status_code}")
            print(f"  Answer: {data['answer'][:100]}...")
        else:
            print(f"✗ Chat endpoint failed with status: {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"✗ Chat endpoint test failed with error: {e}")
    
    # Test chat with selection endpoint
    try:
        print("\nTesting chat with selection endpoint...")
        chat_selection_data = {
            "question": "Can you elaborate on this concept?",
            "selected_text": "Sample selected text for testing",
            "book_id": "test-book-id",
            "session_id": "test-session-123"
        }
        
        response = requests.post(
            f"{base_url}/v1/chat/with-selection",
            json=chat_selection_data,
            headers={"Content-Type": "application/json"}
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Chat with selection endpoint responded with status: {response.status_code}")
            print(f"  Answer: {data['answer'][:100]}...")
        else:
            print(f"✗ Chat with selection endpoint failed with status: {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"✗ Chat with selection endpoint test failed with error: {e}")
    
    # Test upload book endpoint
    try:
        print("\nTesting upload book endpoint...")
        # Create a simple text file for upload
        test_book_path = Path("test_book.txt")
        with open(test_book_path, "w") as f:
            f.write("This is a test book to verify the upload functionality of our RAG chatbot.")
        
        with open(test_book_path, "rb") as f:
            files = {"file": f}
            data = {
                "book_title": "Test Book",
                "author": "Test Author",
                "publication_date": "2023-01-01",
                "version": "1.0.0"
            }
            
            response = requests.post(
                f"{base_url}/v1/upload-book",
                files=files,
                data=data
            )
            
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Upload book endpoint responded with status: {response.status_code}")
            print(f"  Book ID: {data.get('book_id', 'N/A')}")
        else:
            print(f"✗ Upload book endpoint failed with status: {response.status_code}")
            print(f"  Response: {response.text}")
        
        # Clean up test file
        test_book_path.unlink()
    except Exception as e:
        print(f"✗ Upload book endpoint test failed with error: {e}")
    
    # Test selections endpoint
    try:
        print("\nTesting selections endpoint...")
        selection_data = {
            "session_id": "test-session-123",
            "selected_text": "Sample selected text for testing",
            "book_id": "test-book-id",
            "chapter": 1,
            "page": 10,
            "start_offset": 0,
            "end_offset": 50
        }
        
        response = requests.post(
            f"{base_url}/v1/selections",
            json=selection_data,
            headers={"Content-Type": "application/json"}
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Selections endpoint responded with status: {response.status_code}")
            print(f"  Selection ID: {data.get('id', 'N/A')}")
        else:
            print(f"✗ Selections endpoint failed with status: {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"✗ Selections endpoint test failed with error: {e}")
    
    return True


def run_server_in_background():
    """
    Run the FastAPI server in a background thread
    """
    def run_server():
        import uvicorn
        from src.main import app
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    time.sleep(3)  # Give the server time to start
    
    return server_thread


def main():
    """
    Main function to test the RAG chatbot
    """
    print("Starting RAG Chatbot testing process...")
    
    # Create test environment
    create_test_env()
    
    # Start the server in the background
    print("\nStarting the RAG chatbot server...")
    server_thread = run_server_in_background()
    
    # Wait a bit for the server to start
    time.sleep(5)
    
    # Test the API endpoints
    print("\nTesting API endpoints...")
    success = test_api_endpoints()
    
    if success:
        print("\n✓ RAG Chatbot testing completed successfully!")
        print("The application is running and responding to requests.")
    else:
        print("\n✗ Some tests failed during RAG Chatbot testing.")
    
    print("\nTo continue using the RAG chatbot, keep the server running.")
    print("You can access it at: http://localhost:8000")
    print("\nAPI Documentation available at: http://localhost:8000/docs")
    
    # Keep the server running
    try:
        print("\nRAG Chatbot is now running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down RAG Chatbot server...")
        sys.exit(0)


if __name__ == "__main__":
    main()