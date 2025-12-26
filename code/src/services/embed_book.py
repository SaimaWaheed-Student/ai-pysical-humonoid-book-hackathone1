import os
import sys
import asyncio
from typing import List

# Add the src directory to the path so we can import from services
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Temporarily handle database imports to avoid errors
# Set environment variable to use SQLite instead of PostgreSQL
os.environ['DATABASE_URL'] = 'sqlite+aiosqlite:///./test.db'

from services.rag_service import DocumentProcessingService, EmbeddingService, QdrantService


async def embed_book_text(file_path: str, book_id: str, chunk_size: int = 800, overlap: int = 200):
    """
    Process a book file, chunk it, generate embeddings, and store in Qdrant
    """
    # Determine file type and extract text
    if file_path.lower().endswith('.pdf'):
        text = DocumentProcessingService.extract_text_from_pdf(file_path)
    elif file_path.lower().endswith('.docx'):
        text = DocumentProcessingService.extract_text_from_docx(file_path)
    else:
        with open(file_path, 'r', encoding='utf-8') as file:
            text = file.read()

    # Chunk the text
    chunks = DocumentProcessingService.chunk_text(text, chunk_size, overlap)

    # Generate embeddings for all chunks at once (more efficient)
    embeddings = EmbeddingService.generate_embeddings(chunks)

    # Prepare payloads for each chunk
    payloads = []
    for i, chunk in enumerate(chunks):
        payload = {
            "content": chunk,
            "book_id": book_id,
            "chunk_id": f"{book_id}_chunk_{i}",
            "page_number": i  # This is a simple way to assign page numbers, in actuality you might have better page tracking
        }
        payloads.append(payload)

    # Store in Qdrant
    collection_name = "book_embeddings"
    chunk_ids = await QdrantService.store_vectors(
        collection_name=collection_name,
        vectors=embeddings,
        payloads=payloads
    )

    print(f"Successfully embedded {len(chunks)} chunks from {file_path} into Qdrant")
    return chunk_ids


def main():
    """
    Example usage of the embed_book_text function
    """
    import sys

    if len(sys.argv) < 3:
        print("Usage: python embed_book.py <file_path> <book_id>")
        print("Example: python embed_book.py my_book.pdf my_book_id")
        return

    file_path = sys.argv[1]
    book_id = sys.argv[2]

    if not os.path.exists(file_path):
        print(f"File {file_path} does not exist!")
        return

    # Run the embedding process
    loop = asyncio.get_event_loop()
    chunk_ids = loop.run_until_complete(embed_book_text(file_path, book_id))

    print(f"Book {book_id} has been successfully embedded with {len(chunk_ids)} chunks.")


if __name__ == "__main__":
    main()