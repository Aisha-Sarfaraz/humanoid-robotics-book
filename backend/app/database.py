from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from .config import settings


# Create async engine for Neon Postgres - will be initialized when needed
def get_engine():
    if not settings.DATABASE_URL or settings.DATABASE_URL == "":
        # Return a mock engine or raise a specific exception for handling
        from sqlalchemy import create_engine as sync_create_engine
        # For now, return None or create a minimal engine for testing
        # In production, this should be handled in the calling functions
        raise Exception("DATABASE_URL is not configured")

    return create_async_engine(
        settings.DATABASE_URL,
        pool_size=settings.DATABASE_POOL_SIZE,
        pool_timeout=settings.DATABASE_POOL_TIMEOUT,
        pool_recycle=300,
        echo=False  # Set to True for SQL query logging during development
    )


# Create async session - will be initialized when needed
def get_session_local():
    if not settings.DATABASE_URL or settings.DATABASE_URL == "":
        # Return a mock session maker
        class MockSessionMaker:
            def __call__(self):
                return None
        return MockSessionMaker

    engine = get_engine()
    return sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False
    )


# Create base class for models
Base = declarative_base()


# Dependency to get DB session
async def get_db():
    if not settings.DATABASE_URL or settings.DATABASE_URL == "":
        # Yield a mock session or None to avoid errors when DB is not configured
        yield None
    else:
        async with get_session_local()() as session:
            yield session