from typing import Dict, Any, List
from src.logging_config import get_logger


class CitationValidationService:
    """
    Service for validating citations to ensure accuracy and compliance with the Constitution.
    """
    
    def __init__(self):
        self.logger = get_logger(__name__)
        
    def validate_citations(
        self, 
        response: str, 
        sources: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate that the response properly cites sources from the retrieved content.
        
        Args:
            response: The generated response
            sources: List of sources used to generate the response
            
        Returns:
            Validation result with success flag and any validation messages
        """
        # Check if response contains citation format as required (Chapter X, Page Y)
        import re
        
        # Look for citation patterns like "Chapter X, Page Y" or "Ch. X, p. Y"
        citation_pattern = r'(Chapter\s+\d+,\s*Page\s+\d+|Ch\.\s*\d+,\s*p\.\s*\d+|Section\s+[\w\d\-\s]+,\s*Page\s+\d+)'
        found_citations = re.findall(citation_pattern, response)
        
        if not found_citations:
            self.logger.warning("No citations found in response")
            return {
                "valid": False,
                "message": "Response does not contain any citations in required format (Chapter X, Page Y)",
                "citations_found": found_citations,
            }
        
        # Verify that cited chapters/pages exist in the sources
        valid_citations = []
        invalid_citations = []
        
        for citation in found_citations:
            # Extract chapter and page numbers
            chapter_match = re.search(r'Chapter\s+(\d+)|Ch\.\s*(\d+)', citation)
            page_match = re.search(r'Page\s+(\d+)|p\.\s*(\d+)', citation)
            
            if chapter_match and page_match:
                chapter_num = int(chapter_match.group(1) or chapter_match.group(2))
                page_num = int(page_match.group(1) or page_match.group(2))
                
                # Check if this chapter/page combination exists in the sources
                source_match = any(
                    source.get('metadata', {}).get('chapter') == chapter_num and
                    source.get('metadata', {}).get('page') == page_num
                    for source in sources
                )
                
                if source_match:
                    valid_citations.append(citation)
                else:
                    invalid_citations.append(citation)
        
        if invalid_citations:
            self.logger.warning(f"Found invalid citations not in sources: {invalid_citations}")
            return {
                "valid": False,
                "message": f"Some citations in response do not match the provided sources: {invalid_citations}",
                "citations_found": found_citations,
                "valid_citations": valid_citations,
                "invalid_citations": invalid_citations,
            }
        
        # If all citations are valid
        self.logger.info(f"Successfully validated {len(valid_citations)} citations in response")
        return {
            "valid": True,
            "message": f"All {len(valid_citations)} citations are properly validated",
            "citations_found": found_citations,
            "valid_citations": valid_citations,
            "invalid_citations": invalid_citations,
        }
    
    def validate_content_accuracy(
        self,
        response: str,
        sources: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate that the content in the response is supported by the sources.
        
        Args:
            response: The generated response
            sources: List of sources used to generate the response
            
        Returns:
            Validation result with accuracy assessment
        """
        # In a full implementation, this would use more sophisticated NLP techniques
        # to compare the claims in the response against the source content.
        # For now, we'll implement a basic keyword overlap check.
        
        # Extract key terms from sources
        source_content = " ".join([source.get('content', '') for source in sources])
        source_words = set(source_content.lower().split())
        
        # Extract key terms from response
        response_words = set(response.lower().split())
        
        # Calculate overlap
        if source_words:
            overlap = len(source_words.intersection(response_words)) / len(source_words)
        else:
            overlap = 0
            
        if overlap < 0.3:  # If less than 30% of source words appear in response
            self.logger.warning(f"Low content overlap between response and sources: {overlap:.2%}")
            return {
                "accurate": False,
                "message": f"Response has low content overlap with sources ({overlap:.2%})",
                "overlap_percentage": overlap
            }
        
        self.logger.info(f"Content accuracy validated with {overlap:.2%} overlap")
        return {
            "accurate": True,
            "message": f"Response content is consistent with sources ({overlap:.2%} overlap)",
            "overlap_percentage": overlap
        }