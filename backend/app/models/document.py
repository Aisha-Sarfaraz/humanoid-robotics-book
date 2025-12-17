from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, JSON
from sqlalchemy.sql import func
from ..database import Base


class Document(Base):
    __tablename__ = "documents"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, index=True)
    filename = Column(String, unique=True, index=True)
    content = Column(Text)
    content_type = Column(String)  # md, mdx
    source_path = Column(String)
    embedding_vector_id = Column(String)  # Reference to Qdrant vector ID
    checksum = Column(String)  # For update detection
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    is_processed = Column(Boolean, default=False)
    metadata_ = Column("metadata", JSON)  # Using metadata_ to avoid conflict with metadata method