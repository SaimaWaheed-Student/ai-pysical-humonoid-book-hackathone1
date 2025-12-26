from contextlib import asynccontextmanager
from fastapi import FastAPI
from src.config import settings
from src.logging_config import get_logger
from src.api.routes import router as api_router

# Setup logging
logger = get_logger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting up RAG Chatbot API")
    # Initialize any resources here if needed

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API")
    # Clean up any resources here if needed


# Create the FastAPI app instance
app = FastAPI(
    title=settings.app_name,
    version=settings.api_version,
    debug=settings.debug,
    lifespan=lifespan
)

# Include API routes
app.include_router(api_router)

@app.get("/health")
async def health_check():
    logger.info("Health check endpoint accessed")
    return {
        "status": "healthy",
        "version": settings.api_version
    }

# Additional API routes will be added in subsequent tasks


def main():
    """Entry point for running the application directly"""
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level=settings.log_level.lower()
    )


if __name__ == "__main__":
    main()