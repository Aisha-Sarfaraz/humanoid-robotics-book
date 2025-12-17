"""
Indexing Service
Handles document re-indexing with progress tracking
"""
import asyncio
import logging
import uuid
from pathlib import Path
from typing import Dict, Any, Optional, Callable
from datetime import datetime

from app.config import settings
from app.services.document_processor import DocumentProcessor
from app.services.llm_provider import LLMProviderFactory
from app.vector_db import VectorDB
from qdrant_client.http import models

logger = logging.getLogger(__name__)


class IndexingService:
    """Service for indexing and re-indexing documents"""

    # Shared indexing status (in-memory for now, could be Redis in production)
    _indexing_status: Dict[str, Any] = {
        "is_running": False,
        "progress": 0,
        "total": 0,
        "current_file": "",
        "status": "idle",
        "message": "",
        "start_time": None,
        "end_time": None,
        "result": None
    }

    def __init__(self):
        self.provider = LLMProviderFactory.get_provider(
            provider_name=settings.LLM_PROVIDER,
            enable_fallback=settings.ENABLE_FALLBACK
        )
        self.vector_db = VectorDB()
        self.doc_processor = DocumentProcessor()

        logger.info(f"IndexingService initialized with provider: {self.provider.get_provider_name()}")

    async def reindex_all_documents(
        self,
        source_dir: str = "docs/",
        progress_callback: Optional[Callable] = None
    ) -> Dict[str, Any]:
        """
        Re-index all markdown files from source directory

        Args:
            source_dir: Directory containing markdown files
            progress_callback: Optional callback for progress updates

        Returns:
            Indexing result with statistics
        """
        # Check if indexing is already running
        if self._indexing_status["is_running"]:
            return {
                "status": "error",
                "message": "Indexing is already in progress",
                "current_progress": self._indexing_status
            }

        # Initialize status
        self._indexing_status.update({
            "is_running": True,
            "progress": 0,
            "total": 0,
            "current_file": "",
            "status": "scanning",
            "message": "Scanning for markdown files...",
            "start_time": datetime.now().isoformat(),
            "end_time": None,
            "result": None
        })

        try:
            start_time = datetime.now()

            # Get absolute path for source directory
            base_dir = Path(__file__).parent.parent.parent  # Navigate to backend root
            docs_path = base_dir / source_dir

            if not docs_path.exists():
                error_msg = f"Source directory not found: {docs_path}"
                logger.error(error_msg)
                self._indexing_status.update({
                    "is_running": False,
                    "status": "error",
                    "message": error_msg,
                    "end_time": datetime.now().isoformat()
                })
                return {
                    "status": "error",
                    "message": error_msg,
                    "total": 0,
                    "indexed": 0,
                    "failed": 0
                }

            logger.info(f"Scanning directory: {docs_path}")

            # Find all markdown files
            md_files = list(docs_path.glob("**/*.md"))
            total = len(md_files)

            logger.info(f"Found {total} markdown files")

            if total == 0:
                self._indexing_status.update({
                    "is_running": False,
                    "status": "warning",
                    "message": "No markdown files found",
                    "end_time": datetime.now().isoformat()
                })
                return {
                    "status": "warning",
                    "message": "No markdown files found",
                    "total": 0,
                    "indexed": 0,
                    "failed": 0
                }

            # Update status with total
            self._indexing_status.update({
                "total": total,
                "status": "creating_collection",
                "message": "Creating vector collection..."
            })

            # Create Gemini collection if it doesn't exist
            await self.vector_db.create_collection(dimension=self.provider.get_embedding_dimension())

            # Process each file
            indexed = 0
            failed = 0
            failed_files = []

            self._indexing_status.update({
                "status": "indexing",
                "message": "Indexing documents..."
            })

            for i, file_path in enumerate(md_files, 1):
                current_file = file_path.name

                # Update progress
                self._indexing_status.update({
                    "progress": i,
                    "current_file": current_file
                })

                logger.info(f"Processing file {i}/{total}: {current_file}")

                try:
                    # Process document
                    await self._process_single_document(str(file_path))
                    indexed += 1
                    logger.info(f"✓ Successfully indexed: {current_file}")

                    # Call progress callback if provided
                    if progress_callback:
                        await progress_callback(i, total, current_file)

                except Exception as e:
                    failed += 1
                    failed_files.append(current_file)
                    logger.error(f"✗ Failed to index {current_file}: {e}")

            end_time = datetime.now()
            duration = (end_time - start_time).total_seconds()

            # Prepare result
            result = {
                "status": "success" if failed == 0 else "partial",
                "total": total,
                "indexed": indexed,
                "failed": failed,
                "failed_files": failed_files,
                "duration_seconds": duration,
                "provider": self.provider.get_provider_name(),
                "embedding_dimension": self.provider.get_embedding_dimension()
            }

            # Update final status
            self._indexing_status.update({
                "is_running": False,
                "status": "completed",
                "message": f"Indexed {indexed}/{total} documents",
                "end_time": end_time.isoformat(),
                "result": result
            })

            logger.info(f"Indexing completed: {indexed}/{total} documents indexed")

            return result

        except Exception as e:
            logger.error(f"Error during indexing: {e}")

            self._indexing_status.update({
                "is_running": False,
                "status": "error",
                "message": str(e),
                "end_time": datetime.now().isoformat()
            })

            raise

    async def _process_single_document(self, file_path: str):
        """
        Process a single document and store embeddings

        Args:
            file_path: Path to markdown file
        """
        # Extract title from file path
        path_obj = Path(file_path)
        title = path_obj.stem.replace("-", " ").title()

        # Process document to get chunks
        chunks = await self.doc_processor.process_document(file_path, title)

        # Extract text content from chunks
        texts = [chunk["content"] for chunk in chunks]

        # Generate embeddings
        embeddings = await self.provider.generate_embeddings_batch(texts, batch_size=10)

        # Create Qdrant points
        points = []
        document_id = path_obj.stem  # Use filename as document ID

        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Generate UUID from document_id and chunk index for Qdrant compatibility
            point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{document_id}_chunk_{i}"))

            point = models.PointStruct(
                id=point_id,
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

        # Store in Qdrant
        await self.vector_db.store_embeddings(points)

    async def reindex_single_document(self, file_path: str) -> Dict[str, Any]:
        """
        Re-index a single document by path

        Args:
            file_path: Path to markdown file

        Returns:
            Indexing result
        """
        try:
            start_time = datetime.now()

            # Create collection if needed
            await self.vector_db.create_collection(dimension=self.provider.get_embedding_dimension())

            # Process document
            await self._process_single_document(file_path)

            end_time = datetime.now()
            duration = (end_time - start_time).total_seconds()

            return {
                "status": "success",
                "file_path": file_path,
                "duration_seconds": duration,
                "provider": self.provider.get_provider_name()
            }

        except Exception as e:
            logger.error(f"Error indexing {file_path}: {e}")
            return {
                "status": "error",
                "file_path": file_path,
                "message": str(e)
            }

    async def get_indexing_status(self) -> Dict[str, Any]:
        """
        Get current indexing job status

        Returns:
            Status dictionary with progress information
        """
        return self._indexing_status.copy()

    async def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics for both collections

        Returns:
            Dictionary with collection statistics
        """
        gemini_info = await self.vector_db.get_collection_info(settings.QDRANT_COLLECTION_GEMINI)
        openai_info = await self.vector_db.get_collection_info(settings.QDRANT_COLLECTION_OPENAI)

        return {
            "gemini_collection": gemini_info,
            "openai_collection": openai_info,
            "current_provider": self.provider.get_provider_name()
        }

    async def close(self):
        """Close connections"""
        await self.vector_db.close()
