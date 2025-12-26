import os
from qdrant_client import QdrantClient
from qdrant_client.http import models


def recreate_collection():
    """
    Recreate the book_embeddings collection with proper payload indexing
    """
    client = QdrantClient(
        url=os.getenv("QDRANT_URL", "https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333"),
        api_key=os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w"),
    )
    
    # Delete the collection if it exists
    try:
        client.delete_collection("book_embeddings")
        print("Deleted existing 'book_embeddings' collection")
    except:
        print("Collection 'book_embeddings' did not exist, proceeding with creation")
    
    # Create the collection with proper payload schema
    client.create_collection(
        collection_name="book_embeddings",
        vectors_config=models.VectorParams(
            size=384,  # TF-IDF embedding dimension
            distance=models.Distance.COSINE
        ),
        optimizers_config=models.OptimizersConfigDiff(
            memmap_threshold=20000,
            indexing_threshold=20000
        )
    )
    
    # Create payload index for book_id field
    client.create_payload_index(
        collection_name="book_embeddings",
        field_name="book_id",
        field_schema=models.PayloadSchemaType.KEYWORD
    )
    
    print("Successfully created 'book_embeddings' collection with proper indexing")


if __name__ == "__main__":
    recreate_collection()