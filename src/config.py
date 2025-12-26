from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # API Configuration
    app_name: str = "RAG Chatbot API"
    api_version: str = "v1"
    debug: bool = os.getenv("DEBUG", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # Database Configuration
    database_url: str = os.getenv("DATABASE_URL", "sqlite+aiosqlite:///./test.db")  # Using SQLite for development

    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")  # Default local Qdrant
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")

    # API Keys
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "fake-key-for-development")  # Using Gemini instead of OpenAI
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "fake-key-for-development")

    # Performance Configuration
    default_chunk_size: int = int(os.getenv("DEFAULT_CHUNK_SIZE", "512"))
    default_chunk_overlap: int = int(os.getenv("DEFAULT_CHUNK_OVERLAP", "100"))
    retrieval_top_k: int = int(os.getenv("RETRIEVAL_TOP_K", "5"))
    rerank_top_k: int = int(os.getenv("RERANK_TOP_K", "5"))
    min_confidence_score: float = float(os.getenv("MIN_CONFIDENCE_SCORE", "0.7"))

    class Config:
        env_file = ".env"


settings = Settings()