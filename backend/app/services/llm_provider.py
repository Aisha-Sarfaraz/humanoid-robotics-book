"""
LLM Provider Abstraction Layer
Supports multiple LLM providers (Gemini, OpenAI) with automatic fallback
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Iterator, Optional
import logging
from tenacity import (
    retry,
    stop_after_attempt,
    wait_exponential,
    retry_if_exception_type,
)

from app.config import settings

# Import provider-specific services
from app.services.gemini_service import GeminiEmbeddingService, GeminiChatService

# OpenAI imports
from openai import AsyncOpenAI
import openai

logger = logging.getLogger(__name__)


class LLMProvider(ABC):
    """
    Abstract base class for LLM providers
    Defines the interface that all LLM providers must implement
    """

    @abstractmethod
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        pass

    @abstractmethod
    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 10
    ) -> List[List[float]]:
        """Generate embeddings for multiple texts"""
        pass

    @abstractmethod
    async def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for a query (may differ from document embedding)"""
        pass

    @abstractmethod
    async def generate_chat_response(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> str:
        """Generate chat response"""
        pass

    @abstractmethod
    async def generate_chat_response_stream(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> Iterator[str]:
        """Generate streaming chat response"""
        pass

    @abstractmethod
    def get_embedding_dimension(self) -> int:
        """Get the dimension of embeddings from this provider"""
        pass

    @abstractmethod
    def get_provider_name(self) -> str:
        """Get the name of this provider"""
        pass


class GeminiProvider(LLMProvider):
    """
    Gemini LLM Provider
    Uses Google's Gemini API for embeddings and chat
    """

    def __init__(self):
        self.embedding_service = GeminiEmbeddingService()
        self.chat_service = GeminiChatService()
        logger.info("Initialized GeminiProvider")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type(Exception),
    )
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding using Gemini"""
        try:
            return await self.embedding_service.generate_embedding(text)
        except Exception as e:
            logger.error(f"GeminiProvider embedding error: {e}")
            raise

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type(Exception),
    )
    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 10
    ) -> List[List[float]]:
        """Generate batch embeddings using Gemini"""
        try:
            return await self.embedding_service.generate_embeddings_batch(
                texts, batch_size
            )
        except Exception as e:
            logger.error(f"GeminiProvider batch embedding error: {e}")
            raise

    async def generate_query_embedding(self, query: str) -> List[float]:
        """Generate query embedding using Gemini"""
        try:
            return await self.embedding_service.generate_query_embedding(query)
        except Exception as e:
            logger.error(f"GeminiProvider query embedding error: {e}")
            raise

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type(Exception),
    )
    async def generate_chat_response(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> str:
        """Generate chat response using Gemini"""
        try:
            return await self.chat_service.generate_response(
                messages=messages,
                context=context,
                temperature=temperature,
                max_tokens=max_tokens,
            )
        except Exception as e:
            logger.error(f"GeminiProvider chat response error: {e}")
            raise

    async def generate_chat_response_stream(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> Iterator[str]:
        """Generate streaming chat response using Gemini"""
        try:
            async for chunk in self.chat_service.generate_response_stream(
                messages=messages,
                context=context,
                temperature=temperature,
                max_tokens=max_tokens,
            ):
                yield chunk
        except Exception as e:
            logger.error(f"GeminiProvider streaming error: {e}")
            raise

    def get_embedding_dimension(self) -> int:
        """Get Gemini embedding dimension (768)"""
        return 768

    def get_provider_name(self) -> str:
        """Get provider name"""
        return "gemini"


class OpenAIProvider(LLMProvider):
    """
    OpenAI LLM Provider
    Uses OpenAI API for embeddings and chat
    """

    def __init__(self):
        if settings.OPENAI_API_KEY and settings.OPENAI_API_KEY != "":
            self.client = AsyncOpenAI(
                api_key=settings.OPENAI_API_KEY,
                base_url=settings.OPENAI_API_BASE,
                organization=settings.OPENAI_ORGANIZATION,
            )
        else:
            self.client = None
            logger.warning("OpenAI API key not configured")

        self.embedding_model = settings.OPENAI_EMBEDDING_MODEL
        self.chat_model = settings.OPENAI_CHAT_MODEL
        logger.info("Initialized OpenAIProvider")

    def _check_client(self):
        """Check if OpenAI client is initialized"""
        if not self.client:
            raise Exception("OpenAI API key is not configured")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type((openai.RateLimitError, openai.APITimeoutError)),
    )
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding using OpenAI"""
        try:
            self._check_client()
            response = await self.client.embeddings.create(
                input=text, model=self.embedding_model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"OpenAIProvider embedding error: {e}")
            raise

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type((openai.RateLimitError, openai.APITimeoutError)),
    )
    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 10
    ) -> List[List[float]]:
        """Generate batch embeddings using OpenAI"""
        try:
            self._check_client()

            if len(texts) == 0:
                return []

            # Process in batches
            all_embeddings = []
            for i in range(0, len(texts), batch_size):
                batch = texts[i : i + batch_size]
                response = await self.client.embeddings.create(
                    input=batch, model=self.embedding_model
                )
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

            return all_embeddings
        except Exception as e:
            logger.error(f"OpenAIProvider batch embedding error: {e}")
            raise

    async def generate_query_embedding(self, query: str) -> List[float]:
        """Generate query embedding using OpenAI (same as document embedding)"""
        return await self.generate_embedding(query)

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type((openai.RateLimitError, openai.APITimeoutError)),
    )
    async def generate_chat_response(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> str:
        """Generate chat response using OpenAI"""
        try:
            self._check_client()

            # Build messages list
            chat_messages = []

            # Add system message if context provided
            if context:
                chat_messages.append(
                    {
                        "role": "system",
                        "content": f"You are an AI assistant for the Humanoid Robotics Book. Answer questions based on the provided context.\n\nContext:\n{context}",
                    }
                )
            else:
                chat_messages.append(
                    {
                        "role": "system",
                        "content": "You are an AI assistant for the Humanoid Robotics Book. Answer questions based on the provided context. If the context doesn't contain the answer, say so. Always cite the source when possible.",
                    }
                )

            # Add conversation messages
            chat_messages.extend(messages)

            response = await self.client.chat.completions.create(
                model=self.chat_model,
                messages=chat_messages,
                max_tokens=max_tokens,
                temperature=temperature,
            )

            return response.choices[0].message.content
        except Exception as e:
            logger.error(f"OpenAIProvider chat response error: {e}")
            raise

    async def generate_chat_response_stream(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> Iterator[str]:
        """Generate streaming chat response using OpenAI"""
        try:
            self._check_client()

            # Build messages list (same as non-streaming)
            chat_messages = []

            if context:
                chat_messages.append(
                    {
                        "role": "system",
                        "content": f"You are an AI assistant for the Humanoid Robotics Book. Answer questions based on the provided context.\n\nContext:\n{context}",
                    }
                )
            else:
                chat_messages.append(
                    {
                        "role": "system",
                        "content": "You are an AI assistant for the Humanoid Robotics Book. Answer questions based on the provided context. If the context doesn't contain the answer, say so. Always cite the source when possible.",
                    }
                )

            chat_messages.extend(messages)

            stream = await self.client.chat.completions.create(
                model=self.chat_model,
                messages=chat_messages,
                max_tokens=max_tokens,
                temperature=temperature,
                stream=True,
            )

            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error(f"OpenAIProvider streaming error: {e}")
            raise

    def get_embedding_dimension(self) -> int:
        """Get OpenAI embedding dimension (1536 for text-embedding-3-small)"""
        return 1536

    def get_provider_name(self) -> str:
        """Get provider name"""
        return "openai"


class LLMProviderFactory:
    """
    Factory for creating LLM providers
    Supports automatic fallback from Gemini to OpenAI
    """

    _providers: Dict[str, LLMProvider] = {}

    @classmethod
    def get_provider(
        cls, provider_name: Optional[str] = None, enable_fallback: bool = None
    ) -> LLMProvider:
        """
        Get LLM provider instance

        Args:
            provider_name: Provider name ("gemini" or "openai"). If None, uses settings.LLM_PROVIDER
            enable_fallback: Whether to enable fallback. If None, uses settings.ENABLE_FALLBACK

        Returns:
            LLMProvider instance

        Raises:
            Exception: If provider cannot be created
        """
        if provider_name is None:
            provider_name = settings.LLM_PROVIDER.lower()

        if enable_fallback is None:
            enable_fallback = settings.ENABLE_FALLBACK

        # Return cached provider if available
        if provider_name in cls._providers:
            return cls._providers[provider_name]

        # Create new provider
        try:
            if provider_name == "gemini":
                if not settings.GEMINI_API_KEY or settings.GEMINI_API_KEY == "your_gemini_api_key_here_please_replace_this":
                    if enable_fallback:
                        logger.warning(
                            "Gemini API key not configured, falling back to OpenAI"
                        )
                        return cls.get_provider("openai", enable_fallback=False)
                    else:
                        raise Exception("Gemini API key is not configured")

                provider = GeminiProvider()
                cls._providers["gemini"] = provider
                logger.info("Using Gemini as LLM provider")
                return provider

            elif provider_name == "openai":
                if not settings.OPENAI_API_KEY or settings.OPENAI_API_KEY == "":
                    raise Exception("OpenAI API key is not configured")

                provider = OpenAIProvider()
                cls._providers["openai"] = provider
                logger.info("Using OpenAI as LLM provider")
                return provider

            else:
                raise ValueError(
                    f"Unknown provider: {provider_name}. Supported: gemini, openai"
                )

        except Exception as e:
            logger.error(f"Error creating provider {provider_name}: {e}")

            # Try fallback if enabled
            if enable_fallback and provider_name != "openai":
                logger.warning(f"Falling back to OpenAI provider")
                return cls.get_provider("openai", enable_fallback=False)

            raise

    @classmethod
    def clear_cache(cls):
        """Clear provider cache (useful for testing)"""
        cls._providers.clear()
