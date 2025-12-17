from pydantic import BaseModel
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum


class ContentTypeEnum(str, Enum):
    md = "md"
    mdx = "mdx"


class DocumentBase(BaseModel):
    title: str
    filename: str
    content: str
    content_type: ContentTypeEnum
    source_path: str
    checksum: str
    is_processed: bool = False
    metadata_: Optional[Dict[str, Any]] = None


class DocumentCreate(DocumentBase):
    pass


class DocumentUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    checksum: Optional[str] = None
    is_processed: Optional[bool] = None
    metadata_: Optional[Dict[str, Any]] = None


class Document(DocumentBase):
    id: int
    embedding_vector_id: Optional[str] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


# Re-indexing schemas
class ReindexRequest(BaseModel):
    """Request schema for document re-indexing"""
    source_dir: Optional[str] = "docs/"


class ReindexResponse(BaseModel):
    """Response schema for re-indexing operations"""
    status: str
    message: str
    source_dir: Optional[str] = None
    result: Optional[Dict[str, Any]] = None


class IndexingStatusResponse(BaseModel):
    """Response schema for indexing status"""
    is_running: bool
    progress: int = 0
    total: int = 0
    current_file: str = ""
    status: str = "idle"
    message: str = ""
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    result: Optional[Dict[str, Any]] = None


class CollectionStatsResponse(BaseModel):
    """Response schema for collection statistics"""
    gemini_collection: Dict[str, Any] = {}
    openai_collection: Dict[str, Any] = {}
    current_provider: str = ""