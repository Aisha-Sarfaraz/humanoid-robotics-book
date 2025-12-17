import asyncio
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from .config import settings

logger = logging.getLogger(__name__)


class VectorDB:
    def __init__(self):
        # Only initialize client if Qdrant URL is available
        if settings.QDRANT_URL and settings.QDRANT_URL != "":
            self.client = AsyncQdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                prefer_grpc=False  # Using REST API for better compatibility
            )
        else:
            self.client = None

        # Dual collection support
        self.gemini_collection = settings.QDRANT_COLLECTION_GEMINI
        self.openai_collection = settings.QDRANT_COLLECTION_OPENAI

        # Legacy collection name for backward compatibility
        self.collection_name = settings.QDRANT_COLLECTION_NAME

        logger.info(f"VectorDB initialized with collections: Gemini={self.gemini_collection}, OpenAI={self.openai_collection}")

    def _get_collection_by_dimension(self, dimension: int) -> str:
        """
        Determine which collection to use based on vector dimension

        Args:
            dimension: Vector dimension (768 for Gemini, 1536 for OpenAI)

        Returns:
            Collection name
        """
        if dimension == 768:
            return self.gemini_collection
        elif dimension == 1536:
            return self.openai_collection
        else:
            logger.warning(f"Unknown dimension {dimension}, using legacy collection")
            return self.collection_name

    async def create_collection(self, dimension: int = 1536):
        """
        Create the collection in Qdrant if it doesn't exist

        Args:
            dimension: Vector dimension (768 for Gemini, 1536 for OpenAI)
        """
        if not self.client:
            return  # Skip if client is not initialized

        collection_name = self._get_collection_by_dimension(dimension)

        try:
            # Check if collection exists
            await self.client.get_collection(collection_name)
            logger.info(f"Collection {collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            await self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=dimension,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {collection_name} with dimension {dimension}")

    async def create_all_collections(self):
        """
        Create both Gemini and OpenAI collections
        """
        await self.create_collection(dimension=768)   # Gemini
        await self.create_collection(dimension=1536)  # OpenAI

    async def store_embeddings(self, points: List[models.PointStruct], collection_name: Optional[str] = None):
        """
        Store embeddings in Qdrant

        Args:
            points: List of point structures with embeddings
            collection_name: Optional collection name. If not provided, auto-detects from vector dimension
        """
        if not self.client:
            return  # Skip if client is not initialized

        if not points:
            logger.warning("No points to store")
            return

        # Auto-detect collection from first point's vector dimension if not specified
        if collection_name is None:
            first_vector = points[0].vector
            dimension = len(first_vector)
            collection_name = self._get_collection_by_dimension(dimension)
            logger.info(f"Auto-detected collection {collection_name} for dimension {dimension}")

        await self.client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Stored {len(points)} embeddings in collection {collection_name}")

    async def search_similar(self, query_vector: List[float], top_k: int = 5, collection_name: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant

        Args:
            query_vector: Query embedding vector
            top_k: Number of results to return
            collection_name: Optional collection name. If not provided, auto-detects from vector dimension

        Returns:
            List of similar documents with scores
        """
        if not self.client:
            # Return empty results if client is not initialized
            logger.warning("Qdrant client not initialized")
            return []

        # Auto-detect collection from query vector dimension if not specified
        if collection_name is None:
            dimension = len(query_vector)
            collection_name = self._get_collection_by_dimension(dimension)
            logger.info(f"Auto-detected collection {collection_name} for search (dimension {dimension})")

        results = await self.client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=top_k
        )

        return [
            {
                "id": result.id,
                "content": result.payload.get("content", ""),
                "document_title": result.payload.get("title", ""),
                "source_path": result.payload.get("source_path", ""),
                "score": result.score,
                "metadata": result.payload
            }
            for result in results
        ]

    async def delete_document_vectors(self, document_id: str, collection_name: Optional[str] = None):
        """
        Delete all vectors associated with a document

        Args:
            document_id: Document ID to delete
            collection_name: Optional collection name. If not provided, deletes from both collections
        """
        if not self.client:
            return  # Skip if client is not initialized

        # If collection specified, delete only from that collection
        if collection_name:
            collections = [collection_name]
        else:
            # Delete from both Gemini and OpenAI collections
            collections = [self.gemini_collection, self.openai_collection]

        for coll in collections:
            try:
                await self.client.delete(
                    collection_name=coll,
                    points_selector=models.FilterSelector(
                        filter=models.Filter(
                            must=[
                                models.FieldCondition(
                                    key="document_id",
                                    match=models.MatchValue(value=document_id)
                                )
                            ]
                        )
                    )
                )
                logger.info(f"Deleted vectors for document {document_id} from collection {coll}")
            except Exception as e:
                logger.warning(f"Could not delete from collection {coll}: {e}")

    async def get_collection_info(self, collection_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Get information about a collection

        Args:
            collection_name: Collection name (defaults to Gemini collection)

        Returns:
            Dictionary with collection information
        """
        if not self.client:
            return {}

        if collection_name is None:
            collection_name = self.gemini_collection

        try:
            info = await self.client.get_collection(collection_name)
            return {
                "name": collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {}

    async def close(self):
        """
        Close the Qdrant client connection
        """
        if self.client:
            await self.client.close()