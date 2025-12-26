from typing import List, Dict, Any
from src.config import settings
from .gemini_service import GeminiService


class ResponseGenerationService:
    """
    Service for generating responses using Google's Gemini API.
    """

    def __init__(self):
        self.gemini_service = GeminiService()

    async def generate_response(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        user_selection: str = None
    ) -> Dict[str, Any]:
        """
        Generate a response based on the query and context chunks using Gemini.

        Args:
            query: The user's question
            context_chunks: List of relevant context chunks with metadata
            user_selection: Optional user-selected text to prioritize

        Returns:
            Generated response with citations and metadata
        """
        return await self.gemini_service.generate_response(
            query=query,
            context_chunks=context_chunks,
            user_selection=user_selection
        )