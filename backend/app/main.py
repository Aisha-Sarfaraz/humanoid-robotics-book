from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi.errors import RateLimitExceeded
from app.routers import chat, documents
from app.config import settings
from app import models  # Import models to register them with SQLAlchemy
from app.middleware.rate_limiter import (
    limiter,
    custom_rate_limit_exceeded_handler,
    rate_limit_middleware
)

# Create FastAPI application instance
app = FastAPI(
    title="Humanoid Robotics Book RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation chatbot for the Humanoid Robotics Book",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json",
    docs_url="/api/v1/docs",
    redoc_url="/api/v1/redoc"
)

# Add rate limiter state
app.state.limiter = limiter

# Add rate limit exception handler
app.add_exception_handler(RateLimitExceeded, custom_rate_limit_exceeded_handler)

# Add CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for response debugging
    expose_headers=["Access-Control-Allow-Origin"]
)

# Include API routers
app.include_router(chat.router, prefix="/api/v1/chat", tags=["chat"])
app.include_router(documents.router, prefix="/api/v1", tags=["documents"])

@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint to verify API is running"""
    return {
        "status": "healthy",
        "service": "Humanoid Robotics Book RAG Chatbot API",
        "version": "1.0.0"
    }

@app.get("/")
async def root():
    """Root endpoint with API information"""
    return {
        "message": "Welcome to the Humanoid Robotics Book RAG Chatbot API",
        "docs_url": "/api/v1/docs",
        "health_url": "/api/v1/health"
    }