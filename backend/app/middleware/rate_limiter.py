"""
Rate Limiter Middleware
Implements rate limiting for API endpoints to prevent abuse
"""
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request, Response
from typing import Callable
import logging

logger = logging.getLogger(__name__)


def get_client_identifier(request: Request) -> str:
    """
    Get client identifier for rate limiting

    Priority:
    1. X-API-Key header (for authenticated users)
    2. X-Forwarded-For header (for proxied requests)
    3. Remote address (fallback)

    Args:
        request: FastAPI request object

    Returns:
        Client identifier string
    """
    # Check for API key in headers
    api_key = request.headers.get("X-API-Key")
    if api_key:
        return f"apikey:{api_key}"

    # Check for forwarded IP (behind proxy/load balancer)
    forwarded_for = request.headers.get("X-Forwarded-For")
    if forwarded_for:
        # Take the first IP in the chain
        client_ip = forwarded_for.split(",")[0].strip()
        return f"ip:{client_ip}"

    # Fallback to remote address
    return f"ip:{get_remote_address(request)}"


# Initialize rate limiter
limiter = Limiter(
    key_func=get_client_identifier,
    default_limits=["100/hour", "20/minute"],  # Global limits
    storage_uri="memory://",  # In-memory storage (use Redis in production)
    strategy="fixed-window"
)


# Custom rate limit exceeded handler
async def custom_rate_limit_exceeded_handler(request: Request, exc: RateLimitExceeded) -> Response:
    """
    Custom handler for rate limit exceeded errors

    Args:
        request: FastAPI request
        exc: Rate limit exception

    Returns:
        JSON response with rate limit info
    """
    from fastapi.responses import JSONResponse

    # Log the rate limit violation
    client_id = get_client_identifier(request)
    logger.warning(f"Rate limit exceeded for {client_id} on {request.url.path}")

    # Get retry-after time from exception if available
    retry_after = getattr(exc, 'retry_after', None)

    response_data = {
        "error": "rate_limit_exceeded",
        "message": "Too many requests. Please slow down.",
        "detail": str(exc),
        "path": str(request.url.path)
    }

    if retry_after:
        response_data["retry_after_seconds"] = retry_after

    headers = {}
    if retry_after:
        headers["Retry-After"] = str(retry_after)

    return JSONResponse(
        status_code=429,
        content=response_data,
        headers=headers
    )


# Decorator for endpoint-specific rate limits
def rate_limit(limit: str):
    """
    Decorator to apply rate limit to specific endpoints

    Usage:
        @router.post("/expensive-operation")
        @rate_limit("5/minute")
        async def expensive_operation():
            ...

    Args:
        limit: Rate limit string (e.g., "10/minute", "100/hour")

    Returns:
        Decorated function
    """
    def decorator(func: Callable) -> Callable:
        return limiter.limit(limit)(func)
    return decorator


# Pre-configured rate limits for different endpoint types
class RateLimits:
    """Common rate limit configurations"""

    # Chat endpoints - moderate limits
    CHAT_QUERY = "30/minute"  # 30 queries per minute
    CHAT_STREAM = "20/minute"  # 20 streaming requests per minute

    # Search endpoints - higher limits
    SEARCH = "60/minute"  # 60 searches per minute

    # Document management - lower limits (expensive operations)
    REINDEX = "5/hour"  # 5 re-indexing operations per hour
    REINDEX_STATUS = "30/minute"  # 30 status checks per minute

    # Health/status checks - very high limits
    HEALTH = "120/minute"  # 120 health checks per minute

    # Session management
    SESSIONS = "60/minute"  # 60 session operations per minute


# Helper function to check if request should be exempt from rate limiting
def is_rate_limit_exempt(request: Request) -> bool:
    """
    Check if request should be exempt from rate limiting

    Args:
        request: FastAPI request

    Returns:
        True if exempt, False otherwise
    """
    # Exempt health checks
    if request.url.path.endswith("/health"):
        return True

    # Exempt requests with special admin API key
    admin_key = request.headers.get("X-Admin-Key")
    if admin_key and admin_key == "YOUR_ADMIN_KEY":  # Replace with actual key from env
        return True

    # Exempt localhost in development
    client_host = request.client.host if request.client else None
    if client_host in ["127.0.0.1", "localhost", "::1"]:
        # Only exempt in development mode
        from ..config import settings
        if hasattr(settings, 'ENVIRONMENT') and settings.ENVIRONMENT == 'development':
            return True

    return False


# Middleware to add rate limit headers to all responses
async def rate_limit_middleware(request: Request, call_next):
    """
    Middleware to add rate limit information to response headers

    Args:
        request: FastAPI request
        call_next: Next middleware in chain

    Returns:
        Response with rate limit headers
    """
    # Skip exempt requests
    if is_rate_limit_exempt(request):
        return await call_next(request)

    # Process request
    response = await call_next(request)

    # Add rate limit headers (if available from slowapi)
    # These would be populated by the limiter decorators

    return response


# Redis-based rate limiter (for production)
def get_redis_limiter(redis_url: str = None):
    """
    Create rate limiter with Redis backend

    Args:
        redis_url: Redis connection URL (e.g., "redis://localhost:6379")

    Returns:
        Limiter instance with Redis storage
    """
    if not redis_url:
        # Try to get from environment
        import os
        redis_url = os.getenv("REDIS_URL", "redis://localhost:6379")

    return Limiter(
        key_func=get_client_identifier,
        default_limits=["100/hour", "20/minute"],
        storage_uri=redis_url,
        strategy="fixed-window"
    )


# Export commonly used items
__all__ = [
    "limiter",
    "rate_limit",
    "RateLimits",
    "custom_rate_limit_exceeded_handler",
    "rate_limit_middleware",
    "get_redis_limiter",
    "is_rate_limit_exempt"
]
