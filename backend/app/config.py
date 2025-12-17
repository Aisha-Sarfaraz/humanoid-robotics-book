from pydantic_settings import BaseSettings
from typing import List, Optional
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    # Application settings
    APP_NAME: str = "Humanoid Robotics Book RAG Chatbot API"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False

    # API Settings
    API_V1_STR: str = "/api/v1"

    # Database settings - Neon Postgres
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")
    DATABASE_POOL_SIZE: int = int(os.getenv("DATABASE_POOL_SIZE", "5"))
    DATABASE_POOL_TIMEOUT: int = int(os.getenv("DATABASE_POOL_TIMEOUT", "30"))

    # Vector Database settings - Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY", "")

    # Dual collection support: Gemini (768-dim) and OpenAI (1536-dim)
    QDRANT_COLLECTION_GEMINI: str = os.getenv("QDRANT_COLLECTION_GEMINI", "humanoid-robotics-gemini")
    QDRANT_COLLECTION_OPENAI: str = os.getenv("QDRANT_COLLECTION_OPENAI", "humanoid-robotic-book")

    # For backward compatibility
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

    # LLM Provider Selection
    LLM_PROVIDER: str = os.getenv("LLM_PROVIDER", "gemini")  # Options: "gemini" | "openai"
    ENABLE_FALLBACK: bool = os.getenv("ENABLE_FALLBACK", "true").lower() == "true"

    # Gemini API settings (Primary LLM)
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_EMBEDDING_MODEL: str = os.getenv("GEMINI_EMBEDDING_MODEL", "models/text-embedding-004")  # 768 dimensions
    GEMINI_CHAT_MODEL: str = os.getenv("GEMINI_CHAT_MODEL", "gemini-2.0-flash-exp")

    # Gemini rate limits (free tier)
    GEMINI_RPM_LIMIT: int = int(os.getenv("GEMINI_RPM_LIMIT", "15"))  # 15 requests per minute
    GEMINI_TPM_LIMIT: int = int(os.getenv("GEMINI_TPM_LIMIT", "1000000"))  # 1M tokens per minute
    GEMINI_RPD_LIMIT: int = int(os.getenv("GEMINI_RPD_LIMIT", "1500"))  # 1500 requests per day

    # OpenAI settings (Fallback LLM)
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_EMBEDDING_MODEL: str = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-3-small")  # 1536 dimensions
    OPENAI_CHAT_MODEL: str = os.getenv("OPENAI_CHAT_MODEL", "gpt-3.5-turbo")
    OPENAI_API_BASE: Optional[str] = os.getenv("OPENAI_API_BASE")  # For Azure OpenAI or custom endpoints
    OPENAI_ORGANIZATION: Optional[str] = os.getenv("OPENAI_ORGANIZATION")

    # OpenAI rate limits
    OPENAI_RPM_LIMIT: int = int(os.getenv("OPENAI_RPM_LIMIT", "10"))  # Requests per minute
    OPENAI_CHAT_RATE_LIMIT: int = int(os.getenv("OPENAI_CHAT_RATE_LIMIT", "5"))  # Requests per minute

    # Legacy rate limiting settings (for backward compatibility)
    EMBEDDING_RATE_LIMIT: int = int(os.getenv("EMBEDDING_RATE_LIMIT", "10"))  # Requests per minute
    CHAT_RATE_LIMIT: int = int(os.getenv("CHAT_RATE_LIMIT", "5"))  # Requests per minute
    MAX_TOKENS_PER_REQUEST: int = int(os.getenv("MAX_TOKENS_PER_REQUEST", "2000"))
    MAX_EMBEDDING_BATCH_SIZE: int = int(os.getenv("MAX_EMBEDDING_BATCH_SIZE", "10"))

    # ChatKit settings
    CHATKIT_ENABLED: bool = os.getenv("CHATKIT_ENABLED", "true").lower() == "true"
    CHATKIT_STREAMING: bool = os.getenv("CHATKIT_STREAMING", "true").lower() == "true"

    # Document processing settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "1000"))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", "100"))
    ALLOWED_ORIGINS: List[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:8080",
        "http://localhost:5173",  # Vite default port
        "https://aisha-sarfaraz.github.io/humanoid-robotics-book/",  # Example production URL
    ]


# Create a single instance of settings
settings = Settings()