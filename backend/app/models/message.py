from sqlalchemy import Column, Integer, String, Text, DateTime, JSON, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from ..database import Base


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("chat_sessions.id"))
    role = Column(String)  # 'user' or 'assistant'
    content = Column(Text)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    context_chunks = Column(JSON)  # Store context chunks used in the response

    # Relationship
    session = relationship("ChatSession", back_populates="messages")