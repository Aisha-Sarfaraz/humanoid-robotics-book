"""
Streaming Service for Server-Sent Events (SSE)
Provides real-time streaming of chat responses
"""
import asyncio
import json
import logging
from typing import AsyncGenerator, Dict, Any, Optional

from app.services.rag_service import RAGService
from app.services.llm_provider import LLMProviderFactory
from app.config import settings

logger = logging.getLogger(__name__)


class StreamingService:
    """Service for streaming chat responses via SSE"""

    def __init__(self):
        self.rag_service = RAGService()
        self.provider = LLMProviderFactory.get_provider(
            provider_name=settings.LLM_PROVIDER,
            enable_fallback=settings.ENABLE_FALLBACK
        )
        logger.info("StreamingService initialized")

    async def stream_chat_response(
        self,
        query: str,
        session_id: Optional[str] = None,
        selected_text: Optional[str] = None
    ) -> AsyncGenerator[str, None]:
        """
        Stream chat response as Server-Sent Events

        Args:
            query: User query
            session_id: Optional session identifier
            selected_text: Optional selected text for context

        Yields:
            SSE formatted messages
        """
        try:
            # Send initial status
            yield self._format_sse("status", {"message": "Searching documents...", "type": "searching"})

            # Get relevant documents using search service
            similar_docs = await self.rag_service.search_service.semantic_search(
                query=query,
                top_k=5,
                selected_text=selected_text
            )

            # Send sources
            sources = [
                {
                    "title": doc.get("document_title", "Unknown"),
                    "source_path": doc.get("source_path", ""),
                    "score": doc.get("score", 0)
                }
                for doc in similar_docs
            ]
            yield self._format_sse("sources", {"sources": sources, "count": len(sources)})

            # Build context from search results
            context = self.rag_service._prepare_context(similar_docs)

            # Send status update
            yield self._format_sse("status", {"message": "Generating response...", "type": "generating"})

            # Build messages (simplified - no history for now)
            user_message = self.rag_service._create_user_message(query, selected_text)
            messages = [{"role": "user", "content": user_message}]

            # Stream response from LLM
            full_response = ""
            async for chunk in self._stream_llm_response(messages, context):
                full_response += chunk
                yield self._format_sse("chunk", {"content": chunk})

            # Send completion
            yield self._format_sse("done", {
                "session_id": session_id,
                "total_length": len(full_response),
                "sources_used": len(sources)
            })

        except Exception as e:
            logger.error(f"Error in streaming response: {e}")
            yield self._format_sse("error", {
                "message": f"An error occurred: {str(e)}",
                "type": "streaming_error"
            })

    async def _stream_llm_response(
        self,
        messages: list,
        context: str
    ) -> AsyncGenerator[str, None]:
        """
        Stream response from LLM provider

        Args:
            messages: Chat history
            context: Retrieved context

        Yields:
            Response chunks
        """
        try:
            # Build full prompt with context
            system_prompt = f"""You are a helpful AI assistant specializing in humanoid robotics and ROS 2.
Use the following context to answer the user's question accurately and concisely.
If the context doesn't contain relevant information, say so clearly.

Context:
{context}

Provide a clear, well-structured answer with proper citations."""

            # For now, generate full response and stream it word by word
            # In production, you'd use actual streaming API if available
            response = await self.provider.generate_chat_response(
                messages=messages,
                context=context,
                temperature=0.7,
                max_tokens=2000
            )

            # Simulate streaming by splitting into words
            words = response.split()
            for i, word in enumerate(words):
                if i == 0:
                    yield word
                else:
                    yield " " + word
                # Small delay for streaming effect
                await asyncio.sleep(0.05)

        except Exception as e:
            logger.error(f"Error streaming LLM response: {e}")
            yield f"\n\n[Error: {str(e)}]"

    def _format_sse(self, event: str, data: Dict[str, Any]) -> str:
        """
        Format message as Server-Sent Event

        Args:
            event: Event type
            data: Event data

        Returns:
            SSE formatted string
        """
        return f"event: {event}\ndata: {json.dumps(data)}\n\n"

    async def close(self):
        """Close connections"""
        await self.rag_service.close()
