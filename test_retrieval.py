import os
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
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


class QdrantService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL", "https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333"),
            api_key=os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w"),
        )

    def search(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 10,
        book_id: str = None
    ) -> List[dict]:
        """
        Search for similar vectors in Qdrant
        """
        # Prepare filters if needed
        query_filter = None
        if book_id:
            query_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=book_id)
                    )
                ]
            )

        # Perform search
        search_results = self.qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector,
            query_filter=query_filter,
            limit=limit
        )

        # Format results
        results = []
        for result in search_results.points:
            results.append({
                "id": result.id,
                "payload": result.payload,
                "score": result.score
            })

        return results


def search_book_content(query: str, book_id: str = None):
    """
    Search for content in the embedded book
    """
    # Generate embedding for the query
    embedding_service = EmbeddingService()
    query_embedding = embedding_service.generate_embedding(query)

    # Search in Qdrant
    qdrant_service = QdrantService()
    search_results = qdrant_service.search(
        collection_name="book_embeddings",
        query_vector=query_embedding,
        limit=5,
        book_id=book_id
    )

    return search_results


def main():
    """
    Test querying the embedded book content
    """
    # Test search queries
    queries = [
        "What is machine learning?",
        "Supervised learning types",
        "Introduction to deep learning"
    ]

    book_id = "sample_book_id"

    print("=" * 60)
    print("Testing retrieval of embedded book content")
    print("=" * 60)

    for query in queries:
        print(f"\nQuery: {query}")
        print("-" * 40)

        results = search_book_content(query, book_id)

        if results:
            for i, result in enumerate(results):
                content = result["payload"]["content"]
                page_number = result["payload"]["page_number"]
                score = result["score"]

                print(f"Result {i+1} (Score: {score:.3f}, Page: {page_number}):")
                print(f"  {content[:200]}...")
                print()
        else:
            print("No results found for this query.")


if __name__ == "__main__":
    main()