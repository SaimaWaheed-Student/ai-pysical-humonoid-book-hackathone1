import os
import sys
import asyncio
from typing import List
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
from qdrant_client import QdrantClient
from qdrant_client.http import models
from uuid import uuid4
import re


class EmbeddingService:
    def __init__(self):
        self.vectorizer = TfidfVectorizer(max_features=384)  # Fixed dimension for TF-IDF
        self.fitted = False
        self.texts = []  # To keep track of all texts fitted to the vectorizer

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using TF-IDF
        """
        try:
            all_texts = self.texts + [text]

            # Fit the vectorizer on all texts if not fitted yet, or re-fit when adding new texts
            if not self.fitted:
                vectorized = self.vectorizer.fit_transform(all_texts)
                self.fitted = True
            else:
                vectorized = self.vectorizer.transform(all_texts)

            # Get the embedding for the specific text (last in the list)
            embedding = vectorized[-1].toarray()[0]
            return embedding.tolist()
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        try:
            all_texts = self.texts + texts

            # Fit the vectorizer on all texts if not fitted yet, or re-fit when adding new texts
            if not self.fitted:
                vectorized = self.vectorizer.fit_transform(all_texts)
                self.fitted = True
            else:
                vectorized = self.vectorizer.transform(all_texts)

            # Get embeddings for the specific texts (the new ones)
            start_idx = len(self.texts)
            embeddings = vectorized[start_idx:].toarray()

            # Update texts list
            self.texts = all_texts

            return [embedding.tolist() for embedding in embeddings]
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

    def create_collection_if_not_exists(self, collection_name: str, dimension: int = 384):
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
                    size=dimension,  # TF-IDF embedding dimension
                    distance=models.Distance.COSINE
                ),
                # Add payload schema to enable filtering by book_id
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                    indexing_threshold=20000
                )
            )
            # Create payload index for book_id after collection creation
            self.qdrant_client.create_payload_index(
                collection_name=collection_name,
                field_name="book_id",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

    def store_vectors(
        self,
        collection_name: str,
        vectors: List[List[float]],
        payloads: List[dict]
    ) -> List[str]:
        """
        Store vectors in Qdrant with associated payloads
        """
        # Determine the actual dimension of the first vector
        actual_dimension = len(vectors[0]) if vectors else 384

        # Create collection if it doesn't exist
        self.create_collection_if_not_exists(collection_name, actual_dimension)

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
        self.qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        return point_ids


def chunk_text(text: str, chunk_size: int = 800, overlap: int = 200) -> List[str]:
    """
    Split text into chunks with overlap, respecting sentence boundaries
    """
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
    embedding_service = EmbeddingService()
    embeddings = embedding_service.generate_embeddings(chunks)

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
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    chunk_ids = loop.run_until_complete(embed_book_text(file_path, book_id))

    print(f"Book {book_id} has been successfully embedded with {len(chunk_ids)} chunks.")


if __name__ == "__main__":
    main()