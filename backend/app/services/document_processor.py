import hashlib
import asyncio
from typing import List, Dict, Any, Optional
from pathlib import Path
from ..models.document import Document
from ..config import settings
from ..utils.text_splitter import AdvancedTextSplitter


class DocumentProcessor:
    def __init__(self):
        self.text_splitter = AdvancedTextSplitter(
            chunk_size=settings.CHUNK_SIZE,
            chunk_overlap=settings.CHUNK_OVERLAP,
        )

    async def process_document(self, file_path: str, title: str) -> List[Dict[str, Any]]:
        """
        Process a document file and return text chunks with metadata
        """
        content = self._read_file_content(file_path)
        checksum = self._calculate_checksum(content)

        # Split content into chunks using the advanced text splitter
        chunk_data = self.text_splitter.split_text(content)

        # Add additional metadata to each chunk
        for i, chunk_item in enumerate(chunk_data):
            chunk_item["metadata"].update({
                "title": title,
                "source_path": file_path,
                "chunk_index": i,
                "total_chunks": len(chunk_data)
            })

        return chunk_data

    def _read_file_content(self, file_path: str) -> str:
        """
        Read content from a file (MD/MDX)
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        return content

    def _calculate_checksum(self, content: str) -> str:
        """
        Calculate MD5 checksum for content to detect changes
        """
        return hashlib.md5(content.encode('utf-8')).hexdigest()

    async def extract_metadata(self, file_path: str) -> Dict[str, Any]:
        """
        Extract metadata from document file
        """
        path = Path(file_path)
        content = self._read_file_content(file_path)
        checksum = self._calculate_checksum(content)

        return {
            "title": path.stem,
            "filename": path.name,
            "content_type": path.suffix[1:],  # Remove the dot
            "source_path": str(path),
            "checksum": checksum,
            "content": content,
            "is_processed": False
        }

    async def detect_content_changes(self, db_document: Document, file_path: str) -> bool:
        """
        Compare document checksum with file content to detect changes
        """
        new_metadata = await self.extract_metadata(file_path)
        return db_document.checksum != new_metadata["checksum"]