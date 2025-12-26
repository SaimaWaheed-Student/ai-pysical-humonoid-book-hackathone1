import asyncio
import os
from typing import List, Optional, Dict, Any
from uuid import uuid4
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from database.database import SessionLocal
from models import QueryRequest, QueryResponse, Source, MessageCreate
import PyPDF2
import docx
import requests
import re
from sentence_transformers import SentenceTransformer

# Initialize Google Generative AI client
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-pro')

# Initialize sentence transformer model for embeddings
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333"),
    api_key=os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w"),
)

class EmbeddingService:
    @staticmethod
    def generate_embedding(text: str) -> List[float]:
        """
        Generate embedding for text using sentence transformer model
        """
        try:
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
            embeddings = embedding_model.encode(texts)
            return [embedding.tolist() for embedding in embeddings]  # Convert numpy arrays to lists
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            raise


class DocumentProcessingService:
    @staticmethod
    def extract_text_from_pdf(file_path: str) -> str:
        """
        Extract text from a PDF file
        """
        with open(file_path, 'rb') as file:
            reader = PyPDF2.PdfReader(file)
            text = ""
            for page in reader.pages:
                text += page.extract_text()
        return text

    @staticmethod
    def extract_text_from_docx(file_path: str) -> str:
        """
        Extract text from a DOCX file
        """
        doc = docx.Document(file_path)
        text = "\n".join([paragraph.text for paragraph in doc.paragraphs])
        return text

    @staticmethod
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


class QdrantService:
    @staticmethod
    def create_collection_if_not_exists(collection_name: str):
        """
        Create a Qdrant collection if it doesn't exist
        """
        try:
            # Check if collection exists
            qdrant_client.get_collection(collection_name=collection_name)
        except:
            # Create collection if it doesn't exist
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Sentence transformer embedding dimension (all-MiniLM-L6-v2)
                    distance=models.Distance.COSINE
                )
            )

    @staticmethod
    async def store_vectors(
        collection_name: str,
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]]
    ) -> List[str]:
        """
        Store vectors in Qdrant with associated payloads
        """
        # Create collection if it doesn't exist
        QdrantService.create_collection_if_not_exists(collection_name)

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
        qdrant_client.upload_points(
            collection_name=collection_name,
            points=points
        )

        return point_ids

    @staticmethod
    async def search(
        collection_name: str,
        query_vector: List[float],
        limit: int = 10,
        book_id: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant
        """
        # Prepare filters if needed
        filters = None
        if book_id:
            filters = models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=book_id)
                    )
                ]
            )

        # Perform search
        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            query_filter=filters,
            limit=limit
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "payload": result.payload,
                "score": result.score
            })

        return results


class RAGService:
    @staticmethod
    async def perform_rag_query(
        question: str,
        book_id: str,
        selected_text: Optional[str] = None,
        conversation_id: Optional[str] = None
    ) -> QueryResponse:
        """
        Perform a RAG query with the given question
        """
        # Generate embedding for the question
        query_embedding = EmbeddingService.generate_embedding(question)

        # Search in Qdrant
        search_results = await QdrantService.search(
            collection_name="book_embeddings",
            query_vector=query_embedding,
            limit=10,
            book_id=book_id
        )

        # Prepare context for the LLM
        context = ""
        sources = []

        if selected_text:
            context = f"User-selected text: {selected_text}\n\nRelated content from book:\n"
        else:
            context = "Relevant content from book:\n"

        # Add the most relevant chunks to the context
        for result in search_results[:5]:  # Use top 5 results
            payload = result["payload"]
            content = payload.get("content", "")
            page_number = payload.get("page_number", "Unknown")
            chunk_id = payload.get("chunk_id", "")

            context += f"\n[Page {page_number}]: {content}\n"

            sources.append({
                "chunk_id": chunk_id,
                "content": content[:200] + "..." if len(content) > 200 else content,  # Truncate for display
                "page": page_number
            })

        # Prepare the prompt for Gemini
        prompt = f"""You are a helpful assistant answering questions about a book.
        Use the following context to answer the user's question.
        Context: {context}

        If the answer cannot be found in the context, say so.
        Always cite page numbers when referencing content.
        Be concise but comprehensive in your response.

        Question: {question}"""

        try:
            # Call Google Gemini API
            response = model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=500,
                    temperature=0.3,
                )
            )

            answer = response.text

            # Generate a conversation ID if not provided
            if not conversation_id:
                conversation_id = str(uuid4())

            return QueryResponse(
                answer=answer,
                sources=sources,
                conversation_id=conversation_id
            )

        except Exception as e:
            print(f"Error calling Gemini API: {e}")
            return QueryResponse(
                answer="Sorry, I encountered an error processing your request.",
                sources=[],
                conversation_id=conversation_id or str(uuid4())
            )