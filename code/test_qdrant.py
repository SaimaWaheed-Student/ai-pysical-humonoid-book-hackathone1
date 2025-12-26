from qdrant_client import QdrantClient

# Initialize Qdrant client with provided credentials
qdrant_client = QdrantClient(
    url="https://2245ed77-ab7e-4e97-b6fb-8f67366580af.europe-west3-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.uoilSNw-Pvn8Jj51KvLLirCqG9mET2P4-UYyMs2V54w",
)

# Test the connection
try:
    collections = qdrant_client.get_collections()
    print("Qdrant connection successful!")
    print(f"Available collections: {collections}")
except Exception as e:
    print(f"Error connecting to Qdrant: {e}")