"""
Pytest configuration and fixtures
"""
import pytest
import asyncio
from typing import Generator
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


@pytest.fixture(scope="session")
def event_loop():
    """Create event loop for async tests"""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def sample_query():
    """Sample user query for testing"""
    return "What is ROS 2?"


@pytest.fixture
def sample_document_content():
    """Sample document content for testing"""
    return """
# ROS 2: Robot Operating System

ROS 2 is a middleware framework for building robot applications.

## Key Features
- Distributed architecture
- Multi-language support (Python, C++)
- Real-time capabilities
- Modular design
"""


@pytest.fixture
def sample_embedding():
    """Sample embedding vector for testing"""
    # 768-dimensional vector (Gemini embeddings)
    return [0.1] * 768


@pytest.fixture
def mock_qdrant_search_results():
    """Mock search results from Qdrant"""
    return [
        {
            "content": "ROS 2 is a middleware framework...",
            "document_title": "Module ROS2",
            "source_path": "docs/chapter-04/module-ros2.md",
            "score": 0.85
        },
        {
            "content": "Robot Operating System provides...",
            "document_title": "Introduction",
            "source_path": "docs/intro.md",
            "score": 0.78
        }
    ]


@pytest.fixture
def mock_gemini_response():
    """Mock response from Gemini API"""
    return """ROS 2 is a middleware framework designed for building modular and
distributed robot applications. It allows various parts of a robot system to
run on different computers and supports multiple programming languages."""


@pytest.fixture
def sample_session_id():
    """Sample session ID for testing"""
    return "test-session-12345"
