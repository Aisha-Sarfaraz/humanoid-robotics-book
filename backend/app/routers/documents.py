"""
Documents Router
Endpoints for document management and re-indexing
"""
from fastapi import APIRouter, BackgroundTasks, HTTPException, Query, Request
from typing import Optional
import logging

from app.services.indexing_service import IndexingService
from app.schemas.document import (
    ReindexRequest,
    ReindexResponse,
    IndexingStatusResponse,
    CollectionStatsResponse
)
from app.middleware.rate_limiter import limiter, RateLimits

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/documents", tags=["documents"])


@router.post("/reindex", response_model=ReindexResponse)
@limiter.limit(RateLimits.REINDEX)
async def reindex_documents(
    request: Request,
    background_tasks: BackgroundTasks,
    reindex_request: ReindexRequest = None,
    source_dir: Optional[str] = Query("docs/", description="Source directory for markdown files")
):
    """
    Trigger re-indexing of all documents.
    Runs as a background task.

    Args:
        source_dir: Directory containing markdown files (default: "docs/")

    Returns:
        Status message indicating re-indexing has started
    """
    try:
        indexing_service = IndexingService()

        # Check if already running
        status = await indexing_service.get_indexing_status()
        if status["is_running"]:
            raise HTTPException(
                status_code=409,
                detail="Indexing is already in progress. Please wait for it to complete."
            )

        # Start background task
        background_tasks.add_task(
            indexing_service.reindex_all_documents,
            source_dir=source_dir
        )

        return ReindexResponse(
            status="started",
            message=f"Re-indexing started for directory: {source_dir}",
            source_dir=source_dir
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error starting re-indexing: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/reindex/status", response_model=IndexingStatusResponse)
@limiter.limit(RateLimits.REINDEX_STATUS)
async def get_reindex_status(request: Request):
    """
    Get the status of the current re-indexing job.

    Returns:
        Current indexing status with progress information
    """
    try:
        indexing_service = IndexingService()
        status = await indexing_service.get_indexing_status()

        return IndexingStatusResponse(**status)

    except Exception as e:
        logger.error(f"Error getting indexing status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/reindex/{document_id}", response_model=ReindexResponse)
@limiter.limit(RateLimits.REINDEX)
async def reindex_single_document(
    request: Request,
    document_id: str,
    file_path: Optional[str] = Query(None, description="Path to markdown file")
):
    """
    Re-index a specific document by ID or file path.

    Args:
        document_id: Document ID
        file_path: Optional file path to markdown file

    Returns:
        Indexing result for the single document
    """
    try:
        if not file_path:
            raise HTTPException(
                status_code=400,
                detail="file_path query parameter is required"
            )

        indexing_service = IndexingService()
        result = await indexing_service.reindex_single_document(file_path)

        if result["status"] == "error":
            raise HTTPException(status_code=500, detail=result.get("message", "Unknown error"))

        return ReindexResponse(
            status="success",
            message=f"Document {document_id} re-indexed successfully",
            source_dir=file_path,
            result=result
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error re-indexing document {document_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/stats", response_model=CollectionStatsResponse)
@limiter.limit(RateLimits.REINDEX_STATUS)
async def get_collection_stats(request: Request):
    """
    Get statistics for vector database collections.

    Returns:
        Collection statistics for both Gemini and OpenAI collections
    """
    try:
        indexing_service = IndexingService()
        stats = await indexing_service.get_collection_stats()

        return CollectionStatsResponse(**stats)

    except Exception as e:
        logger.error(f"Error getting collection stats: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/collection/{collection_name}")
async def clear_collection(collection_name: str):
    """
    Clear all vectors from a specific collection.
    WARNING: This operation cannot be undone!

    Args:
        collection_name: Name of the collection to clear

    Returns:
        Success message
    """
    # For now, this is not implemented for safety
    # In production, you might want to add authentication and confirmation
    raise HTTPException(
        status_code=501,
        detail="Collection deletion not implemented for safety. Use Qdrant dashboard to manage collections."
    )
