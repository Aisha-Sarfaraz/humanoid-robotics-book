"""
Test RAG Pipeline with Gemini API
"""
import asyncio
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

from app.services.rag_service import RAGService
from app.services.embedding_service import EmbeddingService
from app.config import settings


async def test_rag_pipeline():
    """Test the complete RAG pipeline with Gemini"""

    print("=" * 70)
    print("TESTING RAG PIPELINE WITH GEMINI API")
    print("=" * 70)

    # Initialize services
    print("\n1. Initializing services...")
    rag_service = RAGService()
    embedding_service = EmbeddingService()

    print(f"   Provider: {rag_service.provider.get_provider_name()}")
    print(f"   Embedding dimension: {rag_service.provider.get_embedding_dimension()}")
    print(f"   Collection: {settings.QDRANT_COLLECTION_GEMINI}")

    # Test queries
    test_queries = [
        "What is ROS 2 and why is it important for robotics?",
        "Explain NVIDIA Isaac Sim for robot simulation",
        "What are Vision-Language-Action (VLA) models?"
    ]

    for i, query in enumerate(test_queries, 1):
        print(f"\n{i}. Testing Query: '{query}'")
        print("-" * 70)

        try:
            # Test embedding generation
            print("   > Generating query embedding...")
            query_embedding = await embedding_service.generate_embedding(query)
            print(f"   [OK] Query embedding generated (dimension: {len(query_embedding)})")

            # Test vector search
            print("   > Searching for relevant documents...")
            similar_docs = await embedding_service.search_similar_content(query, top_k=3)
            print(f"   [OK] Found {len(similar_docs)} relevant chunks:")

            for j, doc in enumerate(similar_docs, 1):
                title = doc.get('title', 'Unknown')
                score = doc.get('score', 0)
                content_preview = doc.get('content', '')[:80]
                print(f"      {j}. [{score:.3f}] {title}")
                print(f"         Preview: {content_preview}...")

            # Test RAG response generation
            print("   > Generating response with context...")
            response = await rag_service.generate_response(query=query, session_id=f"test-{i}")

            print(f"   [OK] Response generated ({len(response['response'])} chars)")
            print("\n   RESPONSE:")
            print("   " + "-" * 66)
            # Print response with indentation
            for line in response['response'].split('\n'):
                print(f"   {line}")
            print("   " + "-" * 66)

            print(f"   Sources: {len(response.get('sources', []))} documents used")
            print(f"   Session ID: {response.get('session_id', 'N/A')}")

        except Exception as e:
            print(f"   [ERROR] {e}")
            import traceback
            traceback.print_exc()

    print("\n" + "=" * 70)
    print("RAG PIPELINE TEST COMPLETE")
    print("=" * 70)

    # Close connections
    await rag_service.close()
    await embedding_service.close()


if __name__ == "__main__":
    asyncio.run(test_rag_pipeline())
