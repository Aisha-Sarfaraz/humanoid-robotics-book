"""
Simple RAG Test - Single Query
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from app.services.rag_service import RAGService


async def test_simple_query():
    print("=" * 70)
    print("SIMPLE RAG TEST WITH GEMINI-1.5-FLASH")
    print("=" * 70)

    rag_service = RAGService()
    print(f"\nProvider: {rag_service.provider.get_provider_name()}")
    print(f"Embedding dimension: {rag_service.provider.get_embedding_dimension()}")

    query = "What is ROS 2?"
    print(f"\nQuery: {query}")
    print("-" * 70)

    try:
        response = await rag_service.generate_response(query=query, session_id="simple-test")

        print("\nRESPONSE:")
        print("-" * 70)
        print(response['response'])
        print("-" * 70)
        print(f"\nSources: {len(response.get('sources', []))} documents")
        print(f"Session: {response.get('session_id')}")

        print("\n[SUCCESS] RAG pipeline working correctly!")

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()

    finally:
        await rag_service.close()

    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(test_simple_query())
