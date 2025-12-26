"""
Script to run the RAG Chatbot application
"""
import uvicorn
from src.main import app


def run_dev_server():
    """
    Run the RAG Chatbot server in development mode
    """
    print("Starting RAG Chatbot server in development mode...")
    print("Access the API at: http://localhost:8000")
    print("API Documentation: http://localhost:8000/docs")
    print("Health Check: http://localhost:8000/health")
    
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable auto-reload for development
        log_level="info"
    )


def run_prod_server():
    """
    Run the RAG Chatbot server in production mode
    """
    print("Starting RAG Chatbot server in production mode...")
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info"
    )


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "prod":
        run_prod_server()
    else:
        run_dev_server()