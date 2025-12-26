from typing import List, Dict, Any
from src.config import settings


class GeminiService:
    """
    Service for generating responses using Google's Gemini API.
    """

    def __init__(self):
        # Import the Google GenAI library
        try:
            import google.generativeai as genai
            self.genai = genai
            self.genai.configure(api_key=settings.gemini_api_key)
            # Use a valid model - gemini-1.5-flash is generally available and recommended
            self.model = self.genai.GenerativeModel('gemini-1.5-flash')
        except ImportError:
            # If google.generativeai is not available, use a mock
            self.genai = None
            print("Google Generative AI library not found. Using mock Gemini service.")

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
        if self.genai is None:
            # Mock implementation
            return self._mock_generate_response(query, context_chunks, user_selection)

        try:
            # Format the context for the LLM
            context = self._format_context_for_llm(context_chunks, user_selection)

            # Construct the prompt
            prompt = f"""
            You are an AI assistant that answers questions based only on the provided book content.
            Do not generate any information not present in the provided context.

            Context:
            {context}

            Question: {query}

            Instructions:
            1. Answer the question based only on the provided context
            2. Include citations in the format [Chapter X, Page Y] for each claim made
            3. If the question cannot be answered from the context, say: "This topic is not covered in the book."
            4. Structure your response with clear citations to specific locations in the book
            5. If the user provided selected text, acknowledge it in your response
            """

            # Generate content using Gemini
            response = await self.model.generate_content_async(prompt)

            # Extract the answer
            answer = response.text

            # Extract sources with proper citation formatting
            sources = self._extract_sources_from_chunks(context_chunks)

            return {
                "answer": answer,
                "sources": sources,
                "model": "gemini-pro"
            }

        except Exception as e:
            # Log the error and return a proper response
            print(f"Error in Gemini response generation: {e}")
            return {
                "answer": "An error occurred while generating the response.",
                "sources": [],
                "model": "gemini-pro",
                "error": str(e)
            }

    def _format_context_for_llm(
        self,
        context_chunks: List[Dict[str, Any]],
        user_selection: str = None
    ) -> str:
        """
        Format the context chunks for the LLM.

        Args:
            context_chunks: List of context chunks with metadata
            user_selection: Optional user-selected text

        Returns:
            Formatted context string
        """
        formatted_context = ""

        # Add user selection if provided
        if user_selection:
            formatted_context += f"USER HIGHLIGHTED TEXT: {user_selection}\n\n"

        # Add each context chunk with metadata
        for i, chunk in enumerate(context_chunks):
            metadata = chunk.get('metadata', {})
            formatted_context += f"Context Chunk {i+1}:\n"
            formatted_context += f"Content: {chunk.get('content', '')}\n"
            formatted_context += f"Citation: Chapter {metadata.get('chapter', 'N/A')}, Page {metadata.get('page', 'N/A')}\n"
            formatted_context += f"Section: {metadata.get('section_title', 'N/A')}\n"
            formatted_context += f"Relevance Score: {chunk.get('score', 'N/A')}\n"
            formatted_context += "---\n"

        return formatted_context

    def _extract_sources_from_chunks(self, context_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract source information from context chunks.

        Args:
            context_chunks: List of context chunks with metadata

        Returns:
            List of source information
        """
        sources = []
        for chunk in context_chunks:
            metadata = chunk.get('metadata', {})
            sources.append({
                "chapter": metadata.get('chapter'),
                "page": metadata.get('page'),
                "section": metadata.get('section_title'),
                "quote": chunk.get('content', '')[:200] + "..." if len(chunk.get('content', '')) > 200 else chunk.get('content', ''),
                "relevance": chunk.get('score', 0.0)
            })
        return sources

    def _mock_generate_response(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        user_selection: str = None
    ) -> Dict[str, Any]:
        """
        Mock implementation of response generation.
        """
        # Create a response based on the query
        answer = f"Based on the book content, {query.lower()}. Here are some details from the book [Chapter 1, Page 15]."

        # Add user selection reference if provided
        if user_selection:
            answer = f"Regarding your selected text '{user_selection[:50]}...', {query.lower()}. Here are details from the book [Chapter 2, Page 22]."

        # Extract sources
        sources = []
        for chunk in context_chunks[:3]:  # Use first 3 chunks as sources
            metadata = chunk.get('metadata', {})
            sources.append({
                "chapter": metadata.get('chapter', 1),
                "page": metadata.get('page', 1),
                "section": metadata.get('section_title', 'General'),
                "quote": chunk.get('content', '')[:100] + "..." if len(chunk.get('content', '')) > 100 else chunk.get('content', ''),
                "relevance": chunk.get('score', 0.8)
            })

        return {
            "answer": answer,
            "sources": sources,
            "model": "gemini-pro-mock"
        }