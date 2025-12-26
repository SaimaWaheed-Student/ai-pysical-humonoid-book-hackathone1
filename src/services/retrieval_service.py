from typing import List, Dict, Any
from src.services.embedding_service import EmbeddingService
from src.services.vector_search_service import VectorSearchService
from src.services.rerank_service import RerankService
from src.config import settings


class RetrievalService:
    """
    Orchestrates the complete RAG retrieval pipeline.
    """
    
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.vector_search_service = VectorSearchService()
        self.rerank_service = RerankService()
        
    async def retrieve(
        self,
        query: str,
        book_id: str,
        user_selection: str = None,
        top_k: int = settings.retrieval_top_k,
        rerank_top_k: int = settings.rerank_top_k
    ) -> List[Dict[str, Any]]:
        """
        Execute the complete retrieval pipeline:
        1. Embed the query
        2. Search in vector database
        3. Rerank results
        4. Optionally boost results based on user selection
        
        Args:
            query: The user's question
            book_id: The book ID to search in
            user_selection: Optional user-selected text to boost relevance
            top_k: Number of results to initially retrieve
            rerank_top_k: Number of results after reranking
            
        Returns:
            List of relevant chunks with metadata and relevance scores
        """
        # Step 1: Embed the query
        query_embedding = await self.embedding_service.embed_query(query)
        
        # Step 2: Search in vector database
        search_results = await self.vector_search_service.search(
            query_vector=query_embedding,
            book_id=book_id,
            top_k=top_k * 2  # Get more results for reranking
        )
        
        # Step 3: Prepare documents for reranking
        documents = [result['content'] for result in search_results]
        
        # Step 4: Rerank results
        reranked_results = await self.rerank_service.rerank(
            query=query,
            documents=documents,
            top_k=rerank_top_k
        )
        
        # Step 5: Combine reranked results with original metadata
        final_results = []
        for rerank_result in reranked_results:
            original_result = search_results[rerank_result['index']]
            original_result['score'] = rerank_result['relevance_score']  # Update with reranked score
            final_results.append(original_result)
        
        # Step 6: If user selection exists, potentially boost relevant results
        if user_selection:
            final_results = await self._boost_user_selection_relevance(
                final_results, 
                user_selection, 
                query
            )
        
        return final_results
    
    async def _boost_user_selection_relevance(
        self,
        results: List[Dict[str, Any]], 
        user_selection: str, 
        query: str
    ) -> List[Dict[str, Any]]:
        """
        Boost the relevance of results based on user selection.
        
        Args:
            results: List of search results
            user_selection: User-selected text
            query: Original query
            
        Returns:
            List of results with potentially adjusted scores
        """
        # Calculate embedding similarity between user selection and results
        all_texts = [user_selection] + [result['content'] for result in results]
        embeddings = await self.embedding_service.embed_texts(all_texts)
        
        # Get the user selection embedding (first in list)
        selection_embedding = embeddings[0]
        
        # Compare with each result and boost if similar
        for i, result in enumerate(results):
            result_embedding = embeddings[i + 1]  # +1 due to user selection at index 0
            
            # Calculate cosine similarity (simplified approach)
            # In a real implementation, we would use a proper cosine similarity function
            similarity = self._cosine_similarity(selection_embedding, result_embedding)
            
            # If similarity is above threshold, boost the score
            if similarity > 0.8:  # Arbitrary threshold
                # Boost score by settings.user_selection_boost value (0.3 as per requirements)
                result['score'] = min(1.0, result['score'] + 0.3)
                
        # Re-sort based on the new scores
        results.sort(key=lambda x: x['score'], reverse=True)
        
        return results
    
    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.
        This is a simplified version for demonstration purposes.
        In production, use a proper implementation or library.
        """
        # Using a simplified approach - in practice, you'd use numpy or scipy
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5
        
        if magnitude1 == 0 or magnitude2 == 0:
            return 0
        
        return dot_product / (magnitude1 * magnitude2)