"""
Health check endpoints for monitoring service status and dependencies.
"""

import logging
from datetime import datetime

from fastapi import APIRouter, status
from fastapi.responses import JSONResponse

from ...services.vector_store import vector_store
from ...services.llm_service import llm_service
from ...services.embedder import embedder
from ...services.conversation import conversation_service
from ...core.config import settings

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/")
async def health_check():
    """
    Basic health check endpoint.
    Returns 200 OK if service is running.

    Returns:
        JSON response with service status
    """
    return {
        "status": "healthy",
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "timestamp": datetime.utcnow().isoformat(),
        "environment": settings.environment,
    }


@router.get("/dependencies")
async def health_check_dependencies():
    """
    Detailed health check for all service dependencies.
    Checks connectivity to:
    - Qdrant (vector database)
    - OpenAI (embeddings and LLM)
    - Conversation service (in-memory for MVP)

    Returns:
        JSON response with status of each dependency
    """
    dependencies = {
        "qdrant": {"status": "unknown", "details": None},
        "openai": {"status": "unknown", "details": None},
        "embedder": {"status": "unknown", "details": None},
        "conversation": {"status": "unknown", "details": None},
    }

    overall_healthy = True

    # Check Qdrant
    try:
        collection_info = vector_store.get_collection_info()
        dependencies["qdrant"]["status"] = "healthy"
        dependencies["qdrant"]["details"] = {
            "collection": collection_info["name"],
            "vectors_count": collection_info["vectors_count"],
            "collection_status": collection_info["status"],
        }
        logger.debug("Qdrant health check: PASS")
    except Exception as e:
        dependencies["qdrant"]["status"] = "unhealthy"
        dependencies["qdrant"]["details"] = {"error": str(e)}
        overall_healthy = False
        logger.error(f"Qdrant health check: FAIL - {e}")

    # Check OpenAI (LLM)
    try:
        openai_healthy = await llm_service.health_check()
        if openai_healthy:
            dependencies["openai"]["status"] = "healthy"
            dependencies["openai"]["details"] = {
                "model": llm_service.model,
                "temperature": llm_service.temperature,
            }
            logger.debug("OpenAI health check: PASS")
        else:
            dependencies["openai"]["status"] = "unhealthy"
            dependencies["openai"]["details"] = {"error": "API connection failed"}
            overall_healthy = False
            logger.error("OpenAI health check: FAIL")
    except Exception as e:
        dependencies["openai"]["status"] = "unhealthy"
        dependencies["openai"]["details"] = {"error": str(e)}
        overall_healthy = False
        logger.error(f"OpenAI health check: FAIL - {e}")

    # Check Embedder (cache statistics)
    try:
        cache_stats = embedder.get_cache_stats()
        dependencies["embedder"]["status"] = "healthy"
        dependencies["embedder"]["details"] = {
            "model": cache_stats["model"],
            "cache_size": cache_stats["cache_size"],
            "cache_hit_rate": round(cache_stats["hit_rate"], 3),
        }
        logger.debug("Embedder health check: PASS")
    except Exception as e:
        dependencies["embedder"]["status"] = "unhealthy"
        dependencies["embedder"]["details"] = {"error": str(e)}
        overall_healthy = False
        logger.error(f"Embedder health check: FAIL - {e}")

    # Check Conversation service
    try:
        session_count = conversation_service.get_session_count()
        dependencies["conversation"]["status"] = "healthy"
        dependencies["conversation"]["details"] = {
            "active_sessions": session_count,
            "storage": "in-memory (MVP)",
        }
        logger.debug("Conversation service health check: PASS")
    except Exception as e:
        dependencies["conversation"]["status"] = "unhealthy"
        dependencies["conversation"]["details"] = {"error": str(e)}
        overall_healthy = False
        logger.error(f"Conversation service health check: FAIL - {e}")

    # Return response with appropriate status code
    response_data = {
        "status": "healthy" if overall_healthy else "unhealthy",
        "timestamp": datetime.utcnow().isoformat(),
        "dependencies": dependencies,
    }

    status_code = (
        status.HTTP_200_OK
        if overall_healthy
        else status.HTTP_503_SERVICE_UNAVAILABLE
    )

    return JSONResponse(
        status_code=status_code,
        content=response_data,
    )
