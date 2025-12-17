from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List, Dict, Any
from enum import Enum


class MessageRoleEnum(str, Enum):
    user = "user"
    assistant = "assistant"


class ChatMessageBase(BaseModel):
    session_id: int
    role: MessageRoleEnum
    content: str
    context_chunks: Optional[List[Dict[str, Any]]] = None


class ChatMessageCreate(ChatMessageBase):
    pass


class ChatMessage(ChatMessageBase):
    id: int
    timestamp: datetime

    class Config:
        from_attributes = True


class ChatSessionBase(BaseModel):
    session_id: str
    title: Optional[str] = None
    is_active: bool = True


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSessionUpdate(BaseModel):
    title: Optional[str] = None
    is_active: Optional[bool] = None


class ChatSession(ChatSessionBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None
    messages: List[ChatMessage] = []

    class Config:
        from_attributes = True


class ChatQuery(BaseModel):
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    temperature: Optional[float] = 0.7


class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: List[Dict[str, Any]]
    timestamp: datetime


class SearchQuery(BaseModel):
    query: str
    top_k: Optional[int] = 5
    selected_text: Optional[str] = None


class SearchResult(BaseModel):
    content: str
    document_title: str
    score: float
    source_path: Optional[str] = None


class SearchResponse(BaseModel):
    results: List[SearchResult]