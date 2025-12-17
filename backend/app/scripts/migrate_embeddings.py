"""
Migration Script: Re-index documents with Gemini embeddings
Usage: python -m app.scripts.migrate_embeddings
"""
import asyncio
import sys
import os
import uuid
from pathlib import Path
import logging
from datetime import datetime
from typing import List, Dict, Any

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from app.config import settings
from app.services.document_processor import DocumentProcessor
from app.services.llm_provider import LLMProviderFactory
from app.vector_db import VectorDB
from qdrant_client.http import models

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DocumentMigrator:
    """Migrate documents to Gemini embeddings"""

    def __init__(self, provider_name: str = "gemini"):
        self.provider_name = provider_name
        self.provider = LLMProviderFactory.get_provider(provider_name)
        self.vector_db = VectorDB()
        self.doc_processor = DocumentProcessor()

        logger.info(f"Initialized DocumentMigrator with provider: {self.provider.get_provider_name()}")
        logger.info(f"Embedding dimension: {self.provider.get_embedding_dimension()}")

    async def migrate_all_documents(self, source_dir: str = "docs/") -> Dict[str, Any]:
        """
        Re-index all markdown documents from source directory

        Args:
            source_dir: Directory containing markdown files

        Returns:
            Migration statistics
        """
        start_time = datetime.now()

        # Get absolute path for source directory
        base_dir = Path(__file__).parent.parent.parent.parent  # Navigate to project root
        docs_path = base_dir / source_dir

        if not docs_path.exists():
            logger.error(f"Source directory not found: {docs_path}")
            return {
                "status": "error",
                "message": f"Directory not found: {docs_path}",
                "total": 0,
                "indexed": 0,
                "failed": 0
            }

        logger.info(f"Scanning directory: {docs_path}")

        # Find all markdown files
        md_files = list(docs_path.glob("**/*.md"))
        logger.info(f"Found {len(md_files)} markdown files")

        if not md_files:
            logger.warning("No markdown files found")
            return {
                "status": "warning",
                "message": "No markdown files found",
                "total": 0,
                "indexed": 0,
                "failed": 0
            }

        # Create Gemini collection if it doesn't exist
        logger.info("Creating Gemini collection...")
        await self.vector_db.create_collection(dimension=self.provider.get_embedding_dimension())

        # Process each file
        total = len(md_files)
        indexed = 0
        failed = 0
        failed_files = []

        for i, file_path in enumerate(md_files, 1):
            logger.info(f"Processing file {i}/{total}: {file_path.name}")

            try:
                # Process document
                await self._process_single_document(str(file_path))
                indexed += 1
                logger.info(f"✓ Successfully indexed: {file_path.name}")

            except Exception as e:
                failed += 1
                failed_files.append(str(file_path))
                logger.error(f"✗ Failed to index {file_path.name}: {e}")

        end_time = datetime.now()
        duration = (end_time - start_time).total_seconds()

        # Summary
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

        logger.info("=" * 60)
        logger.info("MIGRATION SUMMARY")
        logger.info("=" * 60)
        logger.info(f"Total files: {total}")
        logger.info(f"Successfully indexed: {indexed}")
        logger.info(f"Failed: {failed}")
        logger.info(f"Duration: {duration:.2f} seconds")
        logger.info(f"Provider: {self.provider.get_provider_name()}")
        logger.info(f"Embedding dimension: {self.provider.get_embedding_dimension()}")
        logger.info("=" * 60)

        if failed_files:
            logger.error(f"Failed files: {', '.join(failed_files)}")

        return result

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
        logger.info(f"  Generated {len(chunks)} chunks")

        # Extract text content from chunks
        texts = [chunk["content"] for chunk in chunks]

        # Generate embeddings
        logger.info(f"  Generating embeddings with {self.provider.get_provider_name()}...")
        embeddings = await self.provider.generate_embeddings_batch(texts, batch_size=10)
        logger.info(f"  Generated {len(embeddings)} embeddings")

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
        logger.info(f"  Storing {len(points)} points in Qdrant...")
        await self.vector_db.store_embeddings(points)
        logger.info(f"  ✓ Stored successfully")

    async def verify_migration(self) -> Dict[str, Any]:
        """
        Verify migration by checking collection stats

        Returns:
            Collection information
        """
        gemini_info = await self.vector_db.get_collection_info(settings.QDRANT_COLLECTION_GEMINI)
        openai_info = await self.vector_db.get_collection_info(settings.QDRANT_COLLECTION_OPENAI)

        return {
            "gemini_collection": gemini_info,
            "openai_collection": openai_info
        }


async def main():
    """Main migration function"""
    logger.info("Starting document migration to Gemini embeddings...")

    # Check if Gemini API key is configured
    if not settings.GEMINI_API_KEY or settings.GEMINI_API_KEY == "your_gemini_api_key_here_please_replace_this":
        logger.error("Gemini API key not configured! Please set GEMINI_API_KEY in .env file")
        return

    logger.info(f"Using LLM provider: {settings.LLM_PROVIDER}")

    # Create migrator
    migrator = DocumentMigrator(provider_name=settings.LLM_PROVIDER)

    # Run migration
    result = await migrator.migrate_all_documents(source_dir="docs/")

    # Verify migration
    logger.info("\nVerifying migration...")
    verification = await migrator.verify_migration()

    logger.info("\nCollection Statistics:")
    logger.info(f"Gemini Collection: {verification['gemini_collection']}")
    logger.info(f"OpenAI Collection: {verification['openai_collection']}")

    # Close connections
    await migrator.vector_db.close()

    logger.info("\nMigration complete!")


if __name__ == "__main__":
    asyncio.run(main())
