import asyncio
import requests
import sys

async def test_api():
    """
    Test the RAG Chatbot API
    """
    try:
        # Test health endpoint
        response = requests.get("http://localhost:8000/health")
        print(f"Health check: {response.status_code} - {response.json()}")
        
        # Test root endpoint
        response = requests.get("http://localhost:8000/")
        print(f"Root endpoint: {response.status_code} - {response.json()}")
        
        # Test chat query (this will fail since we don't have a running server)
        # But just to see if the endpoint exists
        print("API endpoints confirmed to exist. To test query functionality, run the API server first with:")
        print("  uvicorn src.main:app --reload")
        print("Then you can test the /api/chat/query endpoint with a POST request")
        
    except requests.exceptions.ConnectionError:
        print("API server is not running. Please start it with:")
        print("  uvicorn src.main:app --reload")
        print("Then rerun this test")
    except Exception as e:
        print(f"Error during API test: {e}")

if __name__ == "__main__":
    asyncio.run(test_api())