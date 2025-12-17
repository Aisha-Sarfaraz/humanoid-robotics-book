"""
Gemini API Service Layer
Handles embeddings and chat completions using Google's Gemini API (free tier)
"""
import google.generativeai as genai
from typing import List, Dict, Iterator, Optional
import logging
import asyncio
from functools import wraps
import time

from app.config import settings

# Configure logging
logger = logging.getLogger(__name__)

# Configure Gemini API
if settings.GEMINI_API_KEY:
    genai.configure(api_key=settings.GEMINI_API_KEY)
else:
    logger.warning("GEMINI_API_KEY not set - Gemini services will not function")


class RateLimiter:
    """Simple rate limiter for Gemini API calls"""

    def __init__(self, rpm_limit: int = 15):
        self.rpm_limit = rpm_limit
        self.requests = []

    def wait_if_needed(self):
        """Block if rate limit would be exceeded"""
        now = time.time()
        # Remove requests older than 1 minute
        self.requests = [req_time for req_time in self.requests if now - req_time < 60]

        if len(self.requests) >= self.rpm_limit:
            # Calculate wait time until oldest request expires
            wait_time = 60 - (now - self.requests[0]) + 1
            if wait_time > 0:
                logger.info(f"Rate limit reached. Waiting {wait_time:.1f} seconds...")
                time.sleep(wait_time)
                self.requests = []

        self.requests.append(now)


# Global rate limiters
embedding_rate_limiter = RateLimiter(rpm_limit=settings.GEMINI_RPM_LIMIT)
chat_rate_limiter = RateLimiter(rpm_limit=settings.GEMINI_RPM_LIMIT)


def rate_limit(limiter: RateLimiter):
    """Decorator for rate limiting"""

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            limiter.wait_if_needed()
            return func(*args, **kwargs)

        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            limiter.wait_if_needed()
            return await func(*args, **kwargs)

        return async_wrapper if asyncio.iscoroutinefunction(func) else wrapper

    return decorator


class GeminiEmbeddingService:
    """
    Gemini Embedding Service
    Uses text-embedding-004 model (768 dimensions)
    """

    def __init__(self):
        self.model_name = settings.GEMINI_EMBEDDING_MODEL
        self.dimension = 768  # Gemini text-embedding-004 dimension
        logger.info(f"Initialized GeminiEmbeddingService with model: {self.model_name}")

    @rate_limit(embedding_rate_limiter)
    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text to embed

        Returns:
            List of 768 floats representing the embedding

        Raises:
            Exception: If Gemini API call fails
        """
        try:
            if not text or not text.strip():
                raise ValueError("Text cannot be empty")

            # Use genai.embed_content for embeddings
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="retrieval_document",  # For document embeddings
            )

            embedding = result["embedding"]

            if len(embedding) != self.dimension:
                raise ValueError(
                    f"Expected {self.dimension} dimensions, got {len(embedding)}"
                )

            logger.debug(f"Generated embedding for text of length {len(text)}")
            return embedding

        except Exception as e:
            logger.error(f"Error generating Gemini embedding: {e}")
            raise

    @rate_limit(embedding_rate_limiter)
    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 10
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts

        Args:
            texts: List of texts to embed
            batch_size: Number of texts to process in parallel (respects rate limits)

        Returns:
            List of embeddings (each embedding is a list of 768 floats)

        Raises:
            Exception: If Gemini API call fails
        """
        try:
            embeddings = []

            # Process in batches to respect rate limits
            for i in range(0, len(texts), batch_size):
                batch = texts[i : i + batch_size]

                # Generate embeddings for batch
                batch_embeddings = []
                for text in batch:
                    embedding = await self.generate_embedding(text)
                    batch_embeddings.append(embedding)

                embeddings.extend(batch_embeddings)

                logger.info(
                    f"Generated {len(batch_embeddings)} embeddings (batch {i // batch_size + 1})"
                )

            return embeddings

        except Exception as e:
            logger.error(f"Error in batch embedding generation: {e}")
            raise

    async def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a query (search query)

        Args:
            query: Query text to embed

        Returns:
            List of 768 floats representing the query embedding
        """
        try:
            result = genai.embed_content(
                model=self.model_name,
                content=query,
                task_type="retrieval_query",  # For query embeddings
            )

            embedding = result["embedding"]
            logger.debug(f"Generated query embedding for text of length {len(query)}")
            return embedding

        except Exception as e:
            logger.error(f"Error generating query embedding: {e}")
            raise

    def get_dimension(self) -> int:
        """Get embedding dimension"""
        return self.dimension


class GeminiChatService:
    """
    Gemini Chat Service
    Uses gemini-2.0-flash-exp or gemini-1.5-flash model
    """

    def __init__(self):
        self.model_name = settings.GEMINI_CHAT_MODEL
        self.model = genai.GenerativeModel(self.model_name)
        logger.info(f"Initialized GeminiChatService with model: {self.model_name}")

    @rate_limit(chat_rate_limiter)
    async def generate_response(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> str:
        """
        Generate chat response

        Args:
            messages: List of message dicts with 'role' and 'content'
            context: Additional context to include (for RAG)
            temperature: Sampling temperature (0-1)
            max_tokens: Maximum tokens in response

        Returns:
            Generated response text

        Raises:
            Exception: If Gemini API call fails
        """
        try:
            # Build the prompt from messages
            prompt_parts = []

            # Add context if provided
            if context:
                prompt_parts.append(f"Context:\n{context}\n\n")

            # Convert messages to Gemini format
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")

                if role == "system":
                    prompt_parts.append(f"Instructions: {content}\n\n")
                elif role == "user":
                    prompt_parts.append(f"User: {content}\n\n")
                elif role == "assistant":
                    prompt_parts.append(f"Assistant: {content}\n\n")

            # Combine into final prompt
            full_prompt = "".join(prompt_parts)

            # Generate response
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=max_tokens,
                ),
            )

            # Extract text from response
            response_text = response.text

            logger.info(
                f"Generated response of {len(response_text)} characters"
            )
            return response_text

        except Exception as e:
            logger.error(f"Error generating Gemini chat response: {e}")
            raise

    @rate_limit(chat_rate_limiter)
    async def generate_response_stream(
        self,
        messages: List[Dict[str, str]],
        context: str = "",
        temperature: float = 0.7,
        max_tokens: int = 2000,
    ) -> Iterator[str]:
        """
        Generate streaming chat response

        Args:
            messages: List of message dicts with 'role' and 'content'
            context: Additional context to include (for RAG)
            temperature: Sampling temperature (0-1)
            max_tokens: Maximum tokens in response

        Yields:
            Response text chunks

        Raises:
            Exception: If Gemini API call fails
        """
        try:
            # Build the prompt from messages (same as non-streaming)
            prompt_parts = []

            if context:
                prompt_parts.append(f"Context:\n{context}\n\n")

            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")

                if role == "system":
                    prompt_parts.append(f"Instructions: {content}\n\n")
                elif role == "user":
                    prompt_parts.append(f"User: {content}\n\n")
                elif role == "assistant":
                    prompt_parts.append(f"Assistant: {content}\n\n")

            full_prompt = "".join(prompt_parts)

            # Generate streaming response
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=max_tokens,
                ),
                stream=True,
            )

            # Stream chunks
            for chunk in response:
                if chunk.text:
                    yield chunk.text

            logger.info("Completed streaming response")

        except Exception as e:
            logger.error(f"Error in streaming chat response: {e}")
            raise

    async def count_tokens(self, text: str) -> int:
        """
        Count tokens in text (approximate)

        Args:
            text: Text to count tokens for

        Returns:
            Approximate token count
        """
        try:
            result = self.model.count_tokens(text)
            return result.total_tokens
        except Exception as e:
            logger.warning(f"Error counting tokens: {e}")
            # Fallback: rough estimate (1 token ≈ 4 characters)
            return len(text) // 4
