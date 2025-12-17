import asyncio
import logging
from typing import List, Dict, Any, Optional
from ..config import settings
from ..services.search_service import SearchService
from ..services.llm_provider import LLMProviderFactory
from ..models.session import ChatSession
from ..models.message import ChatMessage
from ..database import get_db
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        # Get LLM provider (Gemini with OpenAI fallback)
        try:
            self.provider = LLMProviderFactory.get_provider(
                provider_name=settings.LLM_PROVIDER,
                enable_fallback=settings.ENABLE_FALLBACK
            )
            logger.info(f"RAGService using provider: {self.provider.get_provider_name()}")
        except Exception as e:
            logger.error(f"Failed to initialize LLM provider: {e}")
            self.provider = None

        self.search_service = SearchService()
        self.max_tokens = settings.MAX_TOKENS_PER_REQUEST

    async def generate_response(self, query: str, session_id: Optional[str] = None, selected_text: Optional[str] = None) -> Dict[str, Any]:
        """
        Generate a response using RAG (Retrieval-Augmented Generation)
        """
        # Check if provider is available
        if not self.provider:
            return {
                "response": "Error: LLM provider is not configured. Please check your API keys in the .env file.",
                "session_id": session_id or self._generate_session_id(),
                "sources": [],
                "context_chunks": []
            }

        try:
            # Perform semantic search to retrieve relevant context
            search_results = await self.search_service.semantic_search(
                query,
                top_k=5,
                selected_text=selected_text
            )

            # Prepare context from search results
            context = self._prepare_context(search_results)

            # Check if we have any relevant context
            if not context.strip():
                return {
                    "response": "I couldn't find any relevant information in the book to answer your question. Please make sure the book content has been properly indexed.",
                    "session_id": session_id or self._generate_session_id(),
                    "sources": [],
                    "context_chunks": []
                }

            # Build messages for the LLM
            messages = [
                {
                    "role": "user",
                    "content": self._create_user_message(query, selected_text)
                }
            ]

            # Generate response using configured LLM provider (Gemini or OpenAI)
            response_text = await self.provider.generate_chat_response(
                messages=messages,
                context=context,
                temperature=0.7,
                max_tokens=self.max_tokens
            )

            # Extract sources
            sources = self._extract_sources(search_results)

            logger.info(f"Generated RAG response using {self.provider.get_provider_name()}")

            return {
                "response": response_text,
                "session_id": session_id or self._generate_session_id(),
                "sources": sources,
                "context_chunks": search_results
            }
        except Exception as e:
            # Handle any errors that occur during processing
            logger.error(f"Error in RAG generation: {e}")
            error_msg = f"An error occurred while processing your request: {str(e)}"
            if "API key" in str(e) or "authentication" in str(e).lower():
                error_msg = f"Error: Invalid or missing {self.provider.get_provider_name() if self.provider else 'LLM'} API key. Please check your API key in the .env file."
            elif "rate limit" in str(e).lower():
                error_msg = "Rate limit exceeded. Please try again later."

            return {
                "response": error_msg,
                "session_id": session_id or self._generate_session_id(),
                "sources": [],
                "context_chunks": []
            }

    def _prepare_context(self, search_results: List[Dict[str, Any]]) -> str:
        """
        Prepare context from search results
        """
        context_parts = []
        for result in search_results:
            content = result.get("content", "")
            title = result.get("document_title", "Unknown")
            source = result.get("source_path", "Unknown")
            context_parts.append(f"Document: {title} ({source})\nContent: {content}\n---")

        return "\n".join(context_parts)

    def _create_user_message(self, query: str, selected_text: Optional[str] = None) -> str:
        """
        Create the user message for the LLM (context is passed separately)
        """
        if selected_text:
            message = f"""The user has selected specific text which provides additional context:

Selected Text: {selected_text}

Question: {query}

Please provide an answer based on the context and cite the relevant sources."""
        else:
            message = f"""Question: {query}

Please provide an answer based on the context and cite the relevant sources."""

        return message

    def _extract_sources(self, search_results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract sources from search results
        """
        sources = []
        for result in search_results:
            source = {
                "title": result.get("document_title", ""),
                "path": result.get("source_path", ""),
                "score": result.get("score", 0.0),
                "content_preview": result.get("content", "")[:200] + "..." if len(result.get("content", "")) > 200 else result.get("content", "")
            }
            sources.append(source)

        return sources

    def _generate_session_id(self) -> str:
        """
        Generate a new session ID
        """
        import uuid
        return str(uuid.uuid4())

    async def close(self):
        """
        Close connections
        """
        await self.search_service.close()