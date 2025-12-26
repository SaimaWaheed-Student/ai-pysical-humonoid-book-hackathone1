import requests
import os
from pathlib import Path

def test_upload_simple():
    """
    Simple test to check the upload book endpoint
    """
    # Create a simple text file for testing
    test_file_path = Path("test_book.txt")
    with open(test_file_path, "w") as f:
        f.write("This is a test book to verify the upload functionality of our RAG chatbot.")
    
    try:
        # Test using requests to see what happens
        with open(test_file_path, "rb") as f:
            files = {
                'file': (test_file_path.name, f, 'text/plain')
            }
            data = {
                'book_title': 'Test Book',
                'author': 'Test Author',
                'publication_date': '2023-01-01',
                'version': '1.0.0'
            }
            
            response = requests.post(
                "http://localhost:8000/v1/upload-book",
                files=files,
                data=data
            )
            
            print(f"Status Code: {response.status_code}")
            print(f"Response: {response.text}")
    
    finally:
        # Clean up
        if test_file_path.exists():
            os.remove(test_file_path)

if __name__ == "__main__":
    print("Testing upload endpoint with requests library...")
    print("Note: This will fail if the server is not running on localhost:8000")
    test_upload_simple()