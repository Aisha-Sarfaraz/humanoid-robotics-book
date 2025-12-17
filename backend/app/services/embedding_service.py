import asyncio
import logging
from typing import List, Dict, Any
from ..config import settings
from ..vector_db import VectorDB
from ..services.llm_provider import LLMProviderFactory
from qdrant_client.http import models

logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self):
        from ..config import settings

        # Get LLM provider (Gemini with OpenAI fallback)
        try:
            self.provider = LLMProviderFactory.get_provider(
                provider_name=settings.LLM_PROVIDER,
                enable_fallback=settings.ENABLE_FALLBACK
            )
            logger.info(f"EmbeddingService using provider: {self.provider.get_provider_name()}")
        except Exception as e:
            logger.error(f"Failed to initialize LLM provider: {e}")
            raise

        self.vector_db = VectorDB()
        self.max_batch_size = settings.MAX_EMBEDDING_BATCH_SIZE

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using configured LLM provider
        """
        try:
            return await self.provider.generate_embedding(text)
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts using configured LLM provider
        """
        if len(texts) == 0:
            return []

        try:
            return await self.provider.generate_embeddings_batch(
                texts,
                batch_size=self.max_batch_size
            )
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

    async def store_document_embeddings(self, document_id: str, chunks: List[Dict[str, Any]]):
        """
        Store document chunks with their embeddings in vector database
        """
        # Prepare texts for embedding
        texts = [chunk["content"] for chunk in chunks]

        # Generate embeddings
        embeddings = await self.generate_embeddings_batch(texts)

        # Create Qdrant points
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = models.PointStruct(
                id=f"{document_id}_chunk_{i}",
                vector=embedding,
                payload={
                    "content": chunk["content"],
                    "document_id": document_id,
                    "title": chunk["metadata"]["title"],
                    "source_path": chunk["metadata"]["source_path"],
                    "chunk_index": chunk["metadata"]["chunk_index"],
                    "total_chunks": chunk["metadata"]["total_chunks"]
                }
            )
            points.append(point)

        # Store in vector database
        await self.vector_db.store_embeddings(points)

    async def search_similar_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content in the vector database
        """
        try:
            # Generate query embedding (optimized for search)
            query_embedding = await self.provider.generate_query_embedding(query)

            # Search in vector database
            results = await self.vector_db.search_similar(query_embedding, top_k)

            return results
        except Exception as e:
            logger.error(f"Error searching similar content: {e}")
            raise

    async def close(self):
        """
        Close connections
        """
        await self.vector_db.close()