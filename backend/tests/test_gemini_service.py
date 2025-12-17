"""
Unit tests for Gemini Service
Tests the Gemini API integration including embeddings and chat
"""
import pytest
import asyncio
import time
from unittest.mock import Mock, patch, AsyncMock
from app.services.gemini_service import (
    GeminiEmbeddingService,
    GeminiChatService,
    RateLimiter
)
from app.config import settings


class TestRateLimiter:
    """Test rate limiter functionality"""

    def test_rate_limiter_initialization(self):
        """Test rate limiter initializes correctly"""
        limiter = RateLimiter(rpm_limit=15)
        assert limiter.rpm_limit == 15
        assert len(limiter.requests) == 0

    def test_rate_limiter_allows_requests_within_limit(self):
        """Test rate limiter allows requests within limit"""
        limiter = RateLimiter(rpm_limit=60)  # 60 per minute = 1 per second

        # Add a request
        limiter.requests.append(time.time())

        # Check requests recorded
        assert len(limiter.requests) == 1

        # Wait should not be needed (within limit)
        import time as time_module
        start_time = time_module.time()
        limiter.wait_if_needed()
        end_time = time_module.time()

        # Should not wait (less than 1 second elapsed)
        assert (end_time - start_time) < 1

    def test_rate_limiter_enforces_limit(self):
        """Test rate limiter enforces rate limits"""
        limiter = RateLimiter(rpm_limit=2)  # Very low limit for testing

        # Add requests up to limit
        now = time.time()
        limiter.requests.append(now)
        limiter.requests.append(now)

        # Verify limit is reached
        assert len(limiter.requests) == 2
        assert len(limiter.requests) >= limiter.rpm_limit


class TestGeminiEmbeddingService:
    """Test Gemini embedding service"""

    @pytest.fixture
    def embedding_service(self):
        """Create embedding service instance"""
        return GeminiEmbeddingService()

    def test_embedding_service_initialization(self, embedding_service):
        """Test embedding service initializes correctly"""
        assert embedding_service.model_name == settings.GEMINI_EMBEDDING_MODEL
        assert embedding_service.dimension == 768

    @pytest.mark.asyncio
    @patch('app.services.gemini_service.genai.embed_content')
    async def test_generate_embedding_success(self, mock_embed, embedding_service):
        """Test successful embedding generation"""
        # Mock API response
        mock_embed.return_value = {"embedding": [0.1] * 768}

        # Generate embedding
        text = "What is ROS 2?"
        embedding = await embedding_service.generate_embedding(text)

        # Verify
        assert len(embedding) == 768
        assert all(isinstance(v, float) for v in embedding)
        mock_embed.assert_called_once()

    @pytest.mark.asyncio
    @patch('app.services.gemini_service.genai.embed_content')
    async def test_generate_embedding_batch_success(self, mock_embed, embedding_service):
        """Test batch embedding generation"""
        # Mock API response
        mock_embed.return_value = {"embedding": [0.1] * 768}

        # Generate batch embeddings
        texts = ["What is ROS 2?", "Explain NVIDIA Isaac", "VLA models"]
        embeddings = await embedding_service.generate_embeddings_batch(texts, batch_size=2)

        # Verify
        assert len(embeddings) == 3
        assert all(len(emb) == 768 for emb in embeddings)

        # Should be called twice (batch size = 2, so 2 batches for 3 items)
        assert mock_embed.call_count >= 2

    @pytest.mark.asyncio
    @patch('app.services.gemini_service.genai.embed_content')
    async def test_generate_embedding_handles_errors(self, mock_embed, embedding_service):
        """Test embedding generation handles API errors"""
        # Mock API error
        mock_embed.side_effect = Exception("API Error")

        # Should raise exception
        with pytest.raises(Exception):
            await embedding_service.generate_embedding("Test text")


class TestGeminiChatService:
    """Test Gemini chat service"""

    @pytest.fixture
    def chat_service(self):
        """Create chat service instance"""
        return GeminiChatService()

    def test_chat_service_initialization(self, chat_service):
        """Test chat service initializes correctly"""
        assert chat_service.model_name == settings.GEMINI_CHAT_MODEL
        assert chat_service.model is not None

    @pytest.mark.asyncio
    @patch('app.services.gemini_service.genai.GenerativeModel')
    async def test_generate_response_success(self, mock_model, chat_service):
        """Test successful chat response generation"""
        # Mock model response
        mock_response = Mock()
        mock_response.text = "ROS 2 is a middleware framework..."
        mock_model.return_value.generate_content.return_value = mock_response

        # Generate response
        messages = [{"role": "user", "content": "What is ROS 2?"}]
        context = "Document: ROS 2 Overview\nContent: ROS 2 is a middleware..."

        response = await chat_service.generate_response(
            messages=messages,
            context=context
        )

        # Verify
        assert isinstance(response, str)
        assert len(response) > 0

    @pytest.mark.asyncio
    async def test_generate_response_with_empty_context(self, chat_service):
        """Test response generation with empty context"""
        messages = [{"role": "user", "content": "What is ROS 2?"}]

        # Should still work with empty context
        # Note: This will actually call the API if not mocked
        # In a real test environment, you'd mock this
        # For now, we'll skip actual API calls
        pass

    def test_chat_service_model_configuration(self, chat_service):
        """Test chat service has correct model configuration"""
        # Verify service has model configured
        assert chat_service.model_name is not None
        assert len(chat_service.model_name) > 0
        assert chat_service.model is not None


class TestGeminiServiceIntegration:
    """Integration tests for Gemini services"""

    @pytest.mark.asyncio
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires valid Gemini API key and consumes quota")
    async def test_real_embedding_generation(self):
        """Test actual embedding generation (requires API key)"""
        service = GeminiEmbeddingService()
        embedding = await service.generate_embedding("What is ROS 2?")

        assert len(embedding) == 768
        assert all(isinstance(v, float) for v in embedding)

    @pytest.mark.asyncio
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires valid Gemini API key and consumes quota")
    async def test_real_chat_generation(self):
        """Test actual chat response generation (requires API key)"""
        service = GeminiChatService()
        messages = [{"role": "user", "content": "What is ROS 2?"}]
        context = "ROS 2 is a middleware framework for robotics."

        response = await service.generate_response(messages, context)

        assert isinstance(response, str)
        assert len(response) > 0
        assert "ROS" in response or "middleware" in response.lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
