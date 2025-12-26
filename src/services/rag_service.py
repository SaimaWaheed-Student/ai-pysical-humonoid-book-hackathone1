from typing import Dict, Any, List
from src.services.retrieval_service import RetrievalService
from src.services.response_generation_service import ResponseGenerationService
from src.services.citation_validation_service import CitationValidationService
from src.config import settings
from src.logging_config import get_logger


class RAGService:
    """
    Main RAG orchestration service that coordinates all components.
    """
    
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.response_generation_service = ResponseGenerationService()
        self.citation_validation_service = CitationValidationService()
        self.logger = get_logger(__name__)
        
    async def ask_question(
        self,
        question: str,
        book_id: str,
        session_id: str,
        selected_text: str = None
    ) -> Dict[str, Any]:
        """
        Main method to process a question through the RAG pipeline.
        
        Args:
            question: The user's question
            book_id: The book ID to search in
            session_id: Session identifier for tracking
            selected_text: Optional user-selected text for context
            
        Returns:
            Dictionary with answer, sources, confidence, and validation results
        """
        start_time = __import__('time').time()
        
        try:
            # Step 1: Retrieve relevant chunks
            self.logger.info(f"Starting retrieval for question: {question[:50]}...")
            retrieved_chunks = await self.retrieval_service.retrieve(
                query=question,
                book_id=book_id,
                user_selection=selected_text
            )
            
            retrieval_time = __import__('time').time() - start_time
            
            if not retrieved_chunks:
                self.logger.warning(f"No relevant chunks found for question: {question}")
                return {
                    "answer": "This topic is not covered in the book.",
                    "confidence": 0.0,
                    "sources": [],
                    "user_context_used": bool(selected_text),
                    "user_selection_reference": "No relevant content found in book" if selected_text else None,
                    "follow_up_questions": [],
                    "citations_validated": True,
                    "retrieval_time_ms": int(retrieval_time * 1000),
                    "generation_time_ms": 0,
                    "total_time_ms": int(retrieval_time * 1000)
                }
            
            # Step 2: Generate response
            generation_start_time = __import__('time').time()
            self.logger.info(f"Generating response using {len(retrieved_chunks)} chunks")
            
            response_data = await self.response_generation_service.generate_response(
                query=question,
                context_chunks=retrieved_chunks,
                user_selection=selected_text
            )
            
            generation_time = __import__('time').time() - generation_start_time
            total_time = __import__('time').time() - start_time
            
            # Step 3: Validate citations
            self.logger.info("Validating citations in response")
            citation_validation = self.citation_validation_service.validate_citations(
                response=response_data["answer"],
                sources=response_data["sources"]
            )
            
            # Step 4: Validate content accuracy
            content_accuracy = self.citation_validation_service.validate_content_accuracy(
                response=response_data["answer"],
                sources=retrieved_chunks
            )
            
            # Step 5: Calculate confidence based on validation results
            # For now, using a simple approach - in production this would be more sophisticated
            confidence = min(1.0, settings.min_confidence_score + (len(response_data["sources"]) * 0.1))
            
            # If citation validation fails, reduce confidence significantly
            if not citation_validation["valid"]:
                confidence = 0.1  # Low confidence due to validation failure
            
            # Prepare the response
            result = {
                "answer": response_data["answer"],
                "confidence": confidence,
                "sources": response_data["sources"],
                "user_context_used": bool(selected_text),
                "user_selection_reference": f"Based on your highlighted passage: {selected_text[:50]}..." if selected_text else None,
                "follow_up_questions": [],  # Would be generated in real implementation
                "citations_validated": citation_validation["valid"],
                "retrieval_time_ms": int(retrieval_time * 1000),
                "generation_time_ms": int(generation_time * 1000),
                "total_time_ms": int(total_time * 1000),
                "validation_details": {
                    "citation_validation": citation_validation,
                    "content_accuracy": content_accuracy
                }
            }
            
            self.logger.info(f"Question processed successfully in {total_time:.2f}s with confidence {confidence:.2f}")
            return result
            
        except Exception as e:
            self.logger.error(f"Error processing question '{question}': {str(e)}")
            return {
                "answer": "An error occurred while processing your question. Please try again.",
                "confidence": 0.0,
                "sources": [],
                "user_context_used": bool(selected_text),
                "user_selection_reference": "Error occurred during processing" if selected_text else None,
                "follow_up_questions": [],
                "citations_validated": False,
                "retrieval_time_ms": 0,
                "generation_time_ms": 0,
                "total_time_ms": int((__import__('time').time() - start_time) * 1000),
                "error": str(e)
            }