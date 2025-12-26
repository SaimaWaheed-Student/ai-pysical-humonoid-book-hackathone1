import os
import sys
import asyncio
from typing import List

# Add the src directory to the path so we can import from services
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Import required components directly
from sentence_transformers import SentenceTransformer
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from uuid import uuid4


class EmbeddingService:
    @staticmethod
    def generate_embedding(text: str) -> List[float]:
        """
        Generate embedding for text using sentence transformer model
        """
        try:
            # Initialize sentence transformer model for embeddings
            embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
            embedding = embedding_model.encode([text])
            return embedding[0].tolist()  # Convert numpy array to list
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise

    @staticmethod
    def generate_embeddings(texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        try:
            # Initialize sentence transformer model for embeddings
            embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
            embeddings = embedding_model.encode(texts)
            return [embedding.tolist() for embedding in embeddings]  # Convert numpy arrays to lists
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            raise


class QdrantService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL", "https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333"),
            api_key=os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w"),
        )

    def create_collection_if_not_exists(self, collection_name: str):
        """
        Create a Qdrant collection if it doesn't exist
        """
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(collection_name=collection_name)
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Sentence transformer embedding dimension (all-MiniLM-L6-v2)
                    distance=models.Distance.COSINE
                )
            )

    async def store_vectors(
        self,
        collection_name: str,
        vectors: List[List[float]],
        payloads: List[dict]
    ) -> List[str]:
        """
        Store vectors in Qdrant with associated payloads
        """
        # Create collection if it doesn't exist
        self.create_collection_if_not_exists(collection_name)

        # Generate unique IDs for points
        point_ids = [str(uuid4()) for _ in vectors]

        # Create points to upload
        points = [
            models.PointStruct(
                id=point_id,
                vector=vector,
                payload=payload
            )
            for point_id, vector, payload in zip(point_ids, vectors, payloads)
        ]

        # Upload points to Qdrant
        self.qdrant_client.upload_points(
            collection_name=collection_name,
            points=points
        )

        return point_ids


def chunk_text(text: str, chunk_size: int = 800, overlap: int = 200) -> List[str]:
    """
    Split text into chunks with overlap, respecting sentence boundaries
    """
    import re
    
    # Split into sentences
    sentences = re.split(r'(?<=[.!?]) +', text)
    
    chunks = []
    current_chunk = ""
    
    for sentence in sentences:
        # Check if adding the sentence would exceed the chunk size
        if len(current_chunk) + len(sentence) <= chunk_size:
            current_chunk += sentence + " "
        else:
            # If the chunk is substantial, save it
            if len(current_chunk) > 100:  # Minimum substantial chunk size
                chunks.append(current_chunk.strip())
            
            # Start a new chunk, potentially with overlap if the sentence is long
            if len(sentence) > chunk_size:
                # If the sentence is itself longer than the chunk size, split it
                for i in range(0, len(sentence), chunk_size - overlap):
                    chunk = sentence[i:i + chunk_size - overlap]
                    if len(chunk) > 20:  # Minimum meaningful chunk size
                        chunks.append(chunk)
                current_chunk = ""
            else:
                current_chunk = sentence + " "

    # Add the last chunk if it's substantial
    if current_chunk.strip() and len(current_chunk) > 100:
        chunks.append(current_chunk.strip())

    return chunks


async def embed_book_text(file_path: str, book_id: str, chunk_size: int = 800, overlap: int = 200):
    """
    Process a book file, chunk it, generate embeddings, and store in Qdrant
    """
    # Determine file type and extract text
    if file_path.lower().endswith('.pdf'):
        import PyPDF2
        with open(file_path, 'rb') as file:
            reader = PyPDF2.PdfReader(file)
            text = ""
            for page in reader.pages:
                text += page.extract_text()
    elif file_path.lower().endswith('.docx'):
        import docx
        doc = docx.Document(file_path)
        text = "\n".join([paragraph.text for paragraph in doc.paragraphs])
    else:
        with open(file_path, 'r', encoding='utf-8') as file:
            text = file.read()

    # Chunk the text
    chunks = chunk_text(text, chunk_size, overlap)

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
    qdrant_service = QdrantService()
    collection_name = "book_embeddings"
    chunk_ids = await qdrant_service.store_vectors(
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