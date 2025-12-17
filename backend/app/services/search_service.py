import asyncio
from typing import List, Dict, Any, Optional
from ..services.embedding_service import EmbeddingService


class SearchService:
    def __init__(self):
        self.embedding_service = EmbeddingService()

    async def semantic_search(self, query: str, top_k: int = 5, selected_text: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Perform semantic search with optional context from selected text
        """
        # Check if API keys are properly configured in the embedding service
        from ..config import settings
        if not settings.OPENAI_API_KEY or settings.OPENAI_API_KEY == "":
            # Return empty results if API keys are not configured
            return []

        # If selected text is provided, combine it with the query for better context
        search_query = query
        if selected_text:
            search_query = f"Context: {selected_text}\nQuestion: {query}"

        try:
            # Perform semantic search
            results = await self.embedding_service.search_similar_content(search_query, top_k)

            # Enhance results with relevance scoring
            enhanced_results = []
            for result in results:
                enhanced_result = {
                    "content": result["content"],
                    "document_title": result["document_title"],
                    "source_path": result["source_path"],
                    "score": result["score"],
                    "metadata": result["metadata"]
                }
                enhanced_results.append(enhanced_result)

            return enhanced_results
        except Exception as e:
            # Return empty results if there's an error
            print(f"Search error: {str(e)}")
            return []

    async def keyword_search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Perform keyword-based search (to be implemented with full-text search if needed)
        """
        # For now, we'll use semantic search as the primary method
        # This can be extended with Elasticsearch or PostgreSQL full-text search later
        return await self.semantic_search(query, top_k)

    async def hybrid_search(self, query: str, top_k: int = 5, selected_text: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining semantic and keyword approaches
        """
        # For now, using semantic search as the primary method
        # This can be enhanced with true hybrid search later
        return await self.semantic_search(query, top_k, selected_text)

    async def close(self):
        """
        Close connections
        """
        await self.embedding_service.close()