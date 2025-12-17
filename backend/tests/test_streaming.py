"""
Tests for streaming service functionality.

This module contains unit and integration tests for the StreamingService
which handles Server-Sent Events (SSE) for real-time chat responses.
"""

import pytest
import json
from unittest.mock import Mock, patch, AsyncMock, MagicMock
from typing import AsyncGenerator

from app.services.streaming_service import StreamingService


class TestStreamingService:
    """Test suite for StreamingService."""

    @pytest.fixture
    def streaming_service(self):
        """Create a StreamingService instance for testing."""
        return StreamingService()

    def test_format_sse_chunk(self, streaming_service):
        """Test SSE format for chunk events."""
        result = streaming_service._format_sse("chunk", {"content": "Hello"})

        assert "event: chunk\n" in result
        assert "data: " in result

        # Parse the JSON data
        data_line = [line for line in result.split('\n') if line.startswith('data: ')][0]
        data = json.loads(data_line[6:])  # Remove "data: " prefix
        assert data["content"] == "Hello"

    def test_format_sse_status(self, streaming_service):
        """Test SSE format for status events."""
        result = streaming_service._format_sse("status", {"message": "Processing..."})

        assert "event: status\n" in result
        data_line = [line for line in result.split('\n') if line.startswith('data: ')][0]
        data = json.loads(data_line[6:])
        assert data["message"] == "Processing..."

    def test_format_sse_sources(self, streaming_service):
        """Test SSE format for sources events."""
        sources = [
            {
                "title": "Module ROS2",
                "path": "docs/chapter-04/module-ros2.md",
                "score": 0.85
            }
        ]
        result = streaming_service._format_sse("sources", {"sources": sources})

        assert "event: sources\n" in result
        data_line = [line for line in result.split('\n') if line.startswith('data: ')][0]
        data = json.loads(data_line[6:])
        assert len(data["sources"]) == 1
        assert data["sources"][0]["title"] == "Module ROS2"

    def test_format_sse_done(self, streaming_service):
        """Test SSE format for done events."""
        result = streaming_service._format_sse("done", {})

        assert "event: done\n" in result
        assert "data: {}\n" in result

    def test_format_sse_error(self, streaming_service):
        """Test SSE format for error events."""
        result = streaming_service._format_sse("error", {"error": "API quota exceeded"})

        assert "event: error\n" in result
        data_line = [line for line in result.split('\n') if line.startswith('data: ')][0]
        data = json.loads(data_line[6:])
        assert data["error"] == "API quota exceeded"


class TestStreamingLLMResponse:
    """Test suite for LLM response streaming."""

    @pytest.fixture
    def streaming_service(self):
        """Create a StreamingService instance for testing."""
        return StreamingService()

    @pytest.mark.asyncio
    async def test_stream_llm_response_gemini(self, streaming_service):
        """Test streaming LLM response with Gemini provider."""
        # Mock the provider's generate_chat_response method
        mock_provider = AsyncMock()
        mock_provider.generate_chat_response = AsyncMock(return_value="Hello from Gemini!")

        # Replace the provider on the streaming service
        streaming_service.provider = mock_provider

        messages = [{"role": "user", "content": "Test query"}]
        context = "Test context"

        chunks = []
        async for chunk in streaming_service._stream_llm_response(messages, context):
            chunks.append(chunk)

        # Should get words from the response
        assert len(chunks) == 3
        assert "Hello" in chunks[0]
        assert "from" in chunks[1]
        assert "Gemini!" in chunks[2]

        # Verify provider was called correctly
        mock_provider.generate_chat_response.assert_called_once_with(
            messages=messages,
            context=context,
            temperature=0.7,
            max_tokens=2000
        )

    @pytest.mark.asyncio
    async def test_stream_llm_response_error_handling(self, streaming_service):
        """Test error handling in LLM response streaming."""
        # Mock the provider to raise an exception
        mock_provider = AsyncMock()
        mock_provider.generate_chat_response = AsyncMock(side_effect=Exception("API Error"))

        # Replace the provider on the streaming service
        streaming_service.provider = mock_provider

        messages = [{"role": "user", "content": "Test query"}]
        context = "Test context"

        # The error should be caught and yielded as an error message
        chunks = []
        async for chunk in streaming_service._stream_llm_response(messages, context):
            chunks.append(chunk)

        # Should get error message
        assert len(chunks) == 1
        assert "[Error:" in chunks[0]
        assert "API Error" in chunks[0]


class TestStreamChatResponse:
    """Test suite for complete chat response streaming."""

    @pytest.fixture
    def streaming_service(self):
        """Create a StreamingService instance for testing."""
        return StreamingService()

    @pytest.fixture
    def mock_search_results(self):
        """Create mock search results."""
        return [
            {
                "content": "ROS 2 is a middleware framework for robotics applications.",
                "document_title": "Module ROS2",
                "source_path": "docs/chapter-04/module-ros2.md",
                "score": 0.85
            },
            {
                "content": "ROS 2 uses DDS for communication between nodes.",
                "document_title": "Module ROS2",
                "source_path": "docs/chapter-04/module-ros2.md",
                "score": 0.78
            }
        ]

    @pytest.mark.asyncio
    @patch('app.services.streaming_service.StreamingService._stream_llm_response')
    @patch('app.services.rag_service.RAGService._prepare_context')
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_stream_chat_response_complete_flow(
        self,
        mock_search,
        mock_prepare_context,
        mock_stream_llm,
        streaming_service,
        mock_search_results
    ):
        """Test complete streaming flow from query to response."""
        # Setup mocks
        mock_search.return_value = mock_search_results
        mock_prepare_context.return_value = "Context about ROS 2"

        async def mock_llm_stream():
            yield "ROS 2 "
            yield "is a "
            yield "robotics framework."

        mock_stream_llm.return_value = mock_llm_stream()

        # Collect all SSE events
        events = []
        async for sse_event in streaming_service.stream_chat_response(
            query="What is ROS 2?",
            session_id="test-session"
        ):
            events.append(sse_event)

        # Verify we got the expected events
        event_types = [self._extract_event_type(event) for event in events]

        # Should have: status (searching), sources, status (generating), chunks, done
        assert "status" in event_types
        assert "sources" in event_types
        assert "chunk" in event_types
        assert "done" in event_types

        # Verify search was called
        mock_search.assert_called_once()
        assert mock_search.call_args[1]["query"] == "What is ROS 2?"
        assert mock_search.call_args[1]["top_k"] == 5

    @pytest.mark.asyncio
    @patch('app.services.streaming_service.StreamingService._stream_llm_response')
    @patch('app.services.rag_service.RAGService._prepare_context')
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_stream_chat_response_with_selected_text(
        self,
        mock_search,
        mock_prepare_context,
        mock_stream_llm,
        streaming_service,
        mock_search_results
    ):
        """Test streaming with selected text context."""
        mock_search.return_value = mock_search_results
        mock_prepare_context.return_value = "Context"

        async def mock_llm_stream():
            yield "Response"

        mock_stream_llm.return_value = mock_llm_stream()

        events = []
        async for sse_event in streaming_service.stream_chat_response(
            query="Explain this",
            session_id="test-session",
            selected_text="ROS 2 uses DDS"
        ):
            events.append(sse_event)

        # Verify selected_text was passed to search
        mock_search.assert_called_once()
        assert mock_search.call_args[1]["selected_text"] == "ROS 2 uses DDS"

    @pytest.mark.asyncio
    @patch('app.services.streaming_service.StreamingService._stream_llm_response')
    @patch('app.services.rag_service.RAGService._prepare_context')
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_stream_chat_response_error_handling(
        self,
        mock_search,
        mock_prepare_context,
        mock_stream_llm,
        streaming_service
    ):
        """Test error handling during streaming."""
        # Simulate search failure
        mock_search.side_effect = Exception("Search failed")

        events = []
        async for sse_event in streaming_service.stream_chat_response(
            query="Test query",
            session_id="test-session"
        ):
            events.append(sse_event)

        # Should get an error event
        event_types = [self._extract_event_type(event) for event in events]
        assert "error" in event_types

        # Check error message
        error_events = [e for e in events if "event: error" in e]
        assert len(error_events) > 0
        assert "Search failed" in error_events[0] or "error" in error_events[0].lower()

    @pytest.mark.asyncio
    @patch('app.services.streaming_service.StreamingService._stream_llm_response')
    @patch('app.services.rag_service.RAGService._prepare_context')
    @patch('app.services.search_service.SearchService.semantic_search')
    async def test_stream_chat_response_no_results(
        self,
        mock_search,
        mock_prepare_context,
        mock_stream_llm,
        streaming_service
    ):
        """Test streaming when no search results are found."""
        mock_search.return_value = []  # No results
        mock_prepare_context.return_value = ""

        async def mock_llm_stream():
            yield "I don't have information about that."

        mock_stream_llm.return_value = mock_llm_stream()

        events = []
        async for sse_event in streaming_service.stream_chat_response(
            query="Unknown topic",
            session_id="test-session"
        ):
            events.append(sse_event)

        # Should still complete successfully
        event_types = [self._extract_event_type(event) for event in events]
        assert "done" in event_types

    def _extract_event_type(self, sse_event: str) -> str:
        """Extract event type from SSE event string."""
        for line in sse_event.split('\n'):
            if line.startswith('event: '):
                return line[7:].strip()
        return ""


class TestStreamingIntegration:
    """Integration tests for streaming service."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    @pytest.mark.skip(reason="Skipping to avoid API quota usage")
    async def test_real_streaming_flow(self):
        """
        Integration test with real services (skipped by default).

        To run this test:
        1. Remove the @pytest.mark.skip decorator
        2. Ensure API keys are set in .env
        3. Run: pytest backend/tests/test_streaming.py::TestStreamingIntegration -v
        """
        streaming_service = StreamingService()

        events = []
        async for sse_event in streaming_service.stream_chat_response(
            query="What is ROS 2?",
            session_id="integration-test"
        ):
            events.append(sse_event)

            # Parse and print events for debugging
            event_type = None
            for line in sse_event.split('\n'):
                if line.startswith('event: '):
                    event_type = line[7:].strip()
                    print(f"Event: {event_type}")

        # Verify we got all expected event types
        event_types = set()
        for event in events:
            for line in event.split('\n'):
                if line.startswith('event: '):
                    event_types.add(line[7:].strip())

        assert "status" in event_types
        assert "sources" in event_types or "chunk" in event_types
        assert "done" in event_types or "error" in event_types
