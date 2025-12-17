from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from typing import List
from ..database import get_db
from .. import models, schemas
from ..services.rag_service import RAGService
from ..services.search_service import SearchService
from ..services.document_processor import DocumentProcessor
from ..services.streaming_service import StreamingService
from ..middleware.rate_limiter import limiter, RateLimits
import uuid
from datetime import datetime


router = APIRouter()


@router.post("/query", response_model=schemas.ChatResponse)
@limiter.limit(RateLimits.CHAT_QUERY)
async def chat_query(
    request: Request,
    chat_query: schemas.ChatQuery,
    db: AsyncSession = Depends(get_db)
):
    """
    Process a chat query using RAG
    """
    try:
        rag_service = RAGService()

        # Generate response using RAG
        result = await rag_service.generate_response(
            query=chat_query.message,
            session_id=chat_query.session_id,
            selected_text=chat_query.selected_text
        )

        # Only try to save to database if we have a database connection and valid response
        if db and result["response"] and not result["response"].startswith("Error:"):
            try:
                # Create or get session
                session_id = result["session_id"]
                existing_session = await db.get(models.ChatSession, session_id)

                if not existing_session:
                    # Create new session
                    session = models.ChatSession(
                        session_id=session_id,
                        title=chat_query.message[:50] + "..." if len(chat_query.message) > 50 else chat_query.message
                    )
                    db.add(session)
                    await db.commit()
                    await db.refresh(session)
                else:
                    session = existing_session

                # Create user message
                user_message = models.ChatMessage(
                    session_id=session.id,
                    role="user",
                    content=chat_query.message,
                    context_chunks=result["context_chunks"]
                )
                db.add(user_message)

                # Create assistant message
                assistant_message = models.ChatMessage(
                    session_id=session.id,
                    role="assistant",
                    content=result["response"],
                    context_chunks=result["context_chunks"]
                )
                db.add(assistant_message)

                await db.commit()
            except Exception as db_error:
                # If database operations fail, just continue without saving
                print(f"Database error (non-fatal): {str(db_error)}")

        # Close the RAG service
        await rag_service.close()

        return schemas.ChatResponse(
            response=result["response"],
            session_id=result["session_id"],
            sources=result["sources"],
            timestamp=datetime.now()
        )
    except Exception as e:
        # Log the full error for debugging
        print(f"Chat query error: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error processing chat query: {str(e)}")


@router.post("/stream")
@limiter.limit(RateLimits.CHAT_STREAM)
async def chat_query_stream(
    request: Request,
    chat_query: schemas.ChatQuery
):
    """
    Process a chat query using RAG with Server-Sent Events streaming

    This endpoint streams the response in real-time using SSE.
    Events emitted:
    - status: Updates on processing status
    - sources: Retrieved document sources
    - chunk: Response text chunks
    - done: Completion signal
    - error: Error information
    """
    try:
        streaming_service = StreamingService()

        return StreamingResponse(
            streaming_service.stream_chat_response(
                query=chat_query.message,
                session_id=chat_query.session_id,
                selected_text=chat_query.selected_text
            ),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"  # Disable nginx buffering
            }
        )
    except Exception as e:
        print(f"Streaming error: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error in streaming: {str(e)}")


@router.post("/search", response_model=schemas.SearchResponse)
@limiter.limit(RateLimits.SEARCH)
async def search_content(
    request: Request,
    search_query: schemas.SearchQuery
):
    """
    Perform semantic search on the content
    """
    try:
        search_service = SearchService()

        results = await search_service.semantic_search(
            query=search_query.query,
            top_k=search_query.top_k,
            selected_text=search_query.selected_text
        )

        search_results = [
            schemas.SearchResult(
                content=result["content"],
                document_title=result["document_title"],
                score=result["score"],
                source_path=result["source_path"]
            )
            for result in results
        ]

        await search_service.close()

        return schemas.SearchResponse(results=search_results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing search: {str(e)}")


@router.get("/sessions", response_model=List[schemas.ChatSession])
@limiter.limit(RateLimits.SESSIONS)
async def get_sessions(
    request: Request,
    db: AsyncSession = Depends(get_db)
):
    """
    Get all chat sessions
    """
    try:
        result = await db.execute(select(models.ChatSession))
        sessions = result.scalars().all()

        return [schemas.ChatSession.model_validate(session) for session in sessions]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving sessions: {str(e)}")


@router.get("/sessions/{session_id}", response_model=schemas.ChatSession)
@limiter.limit(RateLimits.SESSIONS)
async def get_session(
    request: Request,
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Get a specific chat session with messages
    """
    try:
        result = await db.execute(
            select(models.ChatSession)
            .filter(models.ChatSession.session_id == session_id)
        )
        session = result.scalar_one_or_none()

        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return schemas.ChatSession.model_validate(session)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving session: {str(e)}")


@router.delete("/sessions/{session_id}")
@limiter.limit(RateLimits.SESSIONS)
async def delete_session(
    request: Request,
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Delete a specific chat session
    """
    try:
        result = await db.execute(
            select(models.ChatSession)
            .filter(models.ChatSession.session_id == session_id)
        )
        session = result.scalar_one_or_none()

        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        await db.delete(session)
        await db.commit()

        return {"message": "Session deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting session: {str(e)}")


# Import select from sqlalchemy
from sqlalchemy import select