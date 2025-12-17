# backend/src/api/router.py (FINAL CODE - Fixed with Threshold 0.0)

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from typing import List
import logging
from starlette.concurrency import run_in_threadpool 

from ..services.embedder import embedder
from ..services.vector_store import vector_store
from ..services.llm_service import llm_service
from ..core.config import settings
from ..models.document import ChunkWithScore

logger = logging.getLogger(__name__)

router = APIRouter()

class ChatRequest(BaseModel):
    """Schema for incoming chat request."""
    session_id: str | None = None
    query: str
    context: dict | None = None

class ChatResponse(BaseModel):
    """Schema for outgoing chat response."""
    response: str
    message_id: str | None = None
    citations: List[dict] | None = None
    context: List[ChunkWithScore]
    llm_model: str
    embedding_model: str

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main RAG Chat endpoint.
    1. Embeds query.
    2. Searches vector store for context using run_in_threadpool.
    3. Sends context and query to LLM for final answer.
    """
    try:
        logger.info(f"Processing chat request: {request.query[:50]}...")

        # 1. Embed the user query
        query_vector_list = await embedder.embed_documents([request.query])
        query_vector = query_vector_list[0]
        logger.debug("Generated query embedding")

        # 2. Search the vector store for context
        # Fixed: Use run_in_threadpool to call the synchronous search method
        context_chunks: List[ChunkWithScore] = await run_in_threadpool( 
            vector_store.search,
            query_embedding=query_vector,
            top_k=settings.vector_search_top_k,
            score_threshold=0.0 # *** CRITICAL FIX: Set threshold to 0.0 to guarantee retrieval ***
        )
        logger.info(f"Retrieved {len(context_chunks)} relevant chunks")

        # Format the chunks into a single context string for the LLM
        formatted_context = "\n---\n".join([
            f"Source: {c.chunk.metadata.source_file}, Chunk Index: {c.chunk.metadata.chunk_index}\n{c.chunk.content}"
            for c in context_chunks
        ])

        # 3. Generate the RAG response
        rag_response = await llm_service.generate_rag_response(
            query=request.query,
            context=formatted_context
        )
        logger.debug(f"Generated LLM response ({len(rag_response)} chars)")

        # Prepare citation data
        citations = [
            {
                "source_file": c.chunk.metadata.source_file,
                "chunk_index": c.chunk.metadata.chunk_index,
                "score": c.score
            }
            for c in context_chunks
        ]

        return ChatResponse(
            response=rag_response,
            context=context_chunks,
            llm_model=settings.llm_model,
            embedding_model=settings.embedding_model,
            citations=citations
        )

    except Exception as e:
        logger.error(f"Chat endpoint error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process chat request: {str(e)}"
        )