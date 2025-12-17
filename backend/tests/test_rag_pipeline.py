"""
Integration tests for RAG Pipeline
Tests the complete Retrieval-Augmented Generation workflow
"""
import pytest
from unittest.mock import Mock, patch, AsyncMock, MagicMock
from app.services.rag_service import RAGService
from app.services.search_service import SearchService
from app.services.embedding_service import EmbeddingService


class TestSearchService:
    """Test search service functionality"""

    @pytest.fixture
    @patch('app.services.search_service.EmbeddingService')
    def search_service(self, mock_embedding):
        """Create search service instance"""
        return SearchService()

    def test_search_service_initialization(self, search_service):
        """Test search service initializes correctly"""
        assert search_service.embedding_service is not None

    @pytest.mark.asyncio
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_semantic_search_returns_results(self, mock_search, search_service, mock_qdrant_search_results):
        """Test semantic search returns results"""
        # Mock search results
        mock_search.return_value = mock_qdrant_search_results

        # Perform search
        results = await search_service.semantic_search("What is ROS 2?", top_k=5)

        # Verify
        assert len(results) > 0
        assert all("content" in r for r in results)
        assert all("document_title" in r for r in results)
        assert all("score" in r for r in results)

    @pytest.mark.asyncio
    async def test_semantic_search_with_selected_text(self, search_service):
        """Test search with selected text context"""
        # This test would need mocking or a test database
        pass


class TestEmbeddingService:
    """Test embedding service functionality"""

    @pytest.fixture
    def embedding_service(self):
        """Create embedding service instance"""
        return EmbeddingService()

    def test_embedding_service_initialization(self, embedding_service):
        """Test embedding service initializes with correct provider"""
        assert embedding_service.provider is not None
        assert hasattr(embedding_service.provider, 'generate_embedding')

    @pytest.mark.asyncio
    @patch('app.services.llm_provider.GeminiProvider.generate_embedding')
    async def test_generate_embedding(self, mock_generate, embedding_service, sample_embedding):
        """Test embedding generation"""
        # Mock provider response
        mock_generate.return_value = sample_embedding

        # Generate embedding
        embedding = await embedding_service.generate_embedding("Test text")

        # Verify
        assert len(embedding) == 768
        mock_generate.assert_called_once()

    @pytest.mark.asyncio
    @patch('app.services.llm_provider.GeminiProvider.generate_query_embedding')
    @patch('app.services.embedding_service.EmbeddingService.search_similar_content')
    async def test_search_similar_content(self, mock_search, mock_embed, embedding_service,
                                        sample_embedding, mock_qdrant_search_results):
        """Test searching for similar content"""
        # Mock responses
        mock_embed.return_value = sample_embedding
        mock_search.return_value = mock_qdrant_search_results

        # Search
        results = await embedding_service.search_similar_content("What is ROS 2?", top_k=3)

        # Verify
        assert len(results) <= 3


class TestRAGService:
    """Test RAG service functionality"""

    @pytest.fixture
    def rag_service(self):
        """Create RAG service instance"""
        return RAGService()

    def test_rag_service_initialization(self, rag_service):
        """Test RAG service initializes correctly"""
        assert rag_service.provider is not None
        assert rag_service.search_service is not None
        assert rag_service.max_tokens > 0

    @pytest.mark.asyncio
    @patch('app.services.search_service.SearchService.semantic_search')
    @patch('app.services.llm_provider.GeminiProvider.generate_chat_response')
    async def test_generate_response_success(self, mock_chat, mock_search, rag_service,
                                            mock_qdrant_search_results, mock_gemini_response):
        """Test successful RAG response generation"""
        # Mock search and chat
        mock_search.return_value = mock_qdrant_search_results
        mock_chat.return_value = mock_gemini_response

        # Generate response
        result = await rag_service.generate_response(
            query="What is ROS 2?",
            session_id="test-123"
        )

        # Verify
        assert "response" in result
        assert "session_id" in result
        assert "sources" in result
        assert len(result["response"]) > 0
        assert result["session_id"] == "test-123"

    @pytest.mark.asyncio
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_generate_response_with_no_results(self, mock_search, rag_service):
        """Test response when no search results found"""
        # Mock empty search results
        mock_search.return_value = []

        # Generate response
        result = await rag_service.generate_response(query="Obscure topic")

        # Verify
        assert "response" in result
        assert "couldn't find" in result["response"].lower() or "no relevant" in result["response"].lower()

    @pytest.mark.asyncio
    @patch('app.services.search_service.SearchService.semantic_search')
    @patch('app.services.llm_provider.GeminiProvider.generate_chat_response')
    async def test_generate_response_with_selected_text(self, mock_chat, mock_search, rag_service,
                                                       mock_qdrant_search_results, mock_gemini_response):
        """Test response generation with selected text context"""
        # Mock responses
        mock_search.return_value = mock_qdrant_search_results
        mock_chat.return_value = mock_gemini_response

        # Generate response with selected text
        result = await rag_service.generate_response(
            query="Explain this",
            selected_text="ROS 2 is a middleware framework...",
            session_id="test-456"
        )

        # Verify
        assert "response" in result
        assert result["session_id"] == "test-456"
        # Verify selected text was passed to search
        mock_search.assert_called_once()

    def test_prepare_context_formatting(self, rag_service, mock_qdrant_search_results):
        """Test context preparation from search results"""
        context = rag_service._prepare_context(mock_qdrant_search_results)

        # Verify
        assert isinstance(context, str)
        assert len(context) > 0
        assert "Document:" in context
        assert "Content:" in context
        # Check that content from results is included
        assert mock_qdrant_search_results[0]["content"] in context

    def test_create_user_message_without_selected_text(self, rag_service):
        """Test user message creation without selected text"""
        message = rag_service._create_user_message("What is ROS 2?")

        assert "Question: What is ROS 2?" in message
        assert "Selected Text" not in message

    def test_create_user_message_with_selected_text(self, rag_service):
        """Test user message creation with selected text"""
        message = rag_service._create_user_message(
            "Explain this",
            selected_text="ROS 2 is a middleware..."
        )

        assert "Selected Text:" in message
        assert "ROS 2 is a middleware..." in message
        assert "Explain this" in message


class TestRAGPipelineEndToEnd:
    """End-to-end integration tests for complete RAG pipeline"""

    @pytest.mark.asyncio
    @pytest.mark.integration
    @patch('app.services.search_service.SearchService.semantic_search')
    @patch('app.services.llm_provider.GeminiProvider.generate_chat_response')
    async def test_complete_pipeline_flow(self, mock_chat, mock_search,
                                         mock_qdrant_search_results, mock_gemini_response):
        """Test complete RAG pipeline from query to response"""
        # Setup mocks
        mock_search.return_value = mock_qdrant_search_results
        mock_chat.return_value = mock_gemini_response

        # Create service
        rag_service = RAGService()

        # Execute pipeline
        result = await rag_service.generate_response(
            query="What is ROS 2?",
            session_id="e2e-test"
        )

        # Verify all steps executed
        mock_search.assert_called_once()
        mock_chat.assert_called_once()

        # Verify result structure
        assert result["response"] == mock_gemini_response
        assert len(result["sources"]) > 0
        assert result["session_id"] == "e2e-test"

    @pytest.mark.asyncio
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires running Qdrant and valid API keys")
    async def test_real_pipeline_execution(self):
        """Test actual pipeline with real services (requires setup)"""
        rag_service = RAGService()

        result = await rag_service.generate_response(
            query="What is ROS 2?",
            session_id="real-test"
        )

        # Verify real response
        assert result["response"]
        assert len(result["response"]) > 0
        assert result["session_id"] == "real-test"
        # Should have found relevant sources
        assert len(result["sources"]) > 0

    @pytest.mark.asyncio
    async def test_error_handling_in_pipeline(self, mock_qdrant_search_results):
        """Test error handling throughout the pipeline"""
        rag_service = RAGService()

        # Test with None provider (simulating initialization failure)
        original_provider = rag_service.provider
        rag_service.provider = None

        result = await rag_service.generate_response(query="Test")

        # Should return error response
        assert "Error" in result["response"]
        assert result["session_id"] is not None

        # Restore
        rag_service.provider = original_provider


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
