"""
Chat API routes for RAG chatbot.
Handles query processing and conversation history retrieval.
"""

from datetime import datetime
from uuid import uuid4
import time
import logging

from fastapi import APIRouter, HTTPException, status

from ...models.chat import ChatRequest, ChatResponse, Citation
from ...models.session import Message, MessageRole
from ...services.embedder import embedder
from ...services.vector_store import vector_store
from ...services.llm_service import llm_service
from ...services.conversation import conversation_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Process a chat query with RAG pipeline.

    Pipeline:
    1. Get conversation context (if exists)
    2. Generate query embedding
    3. Search vector store for relevant chunks
    4. Generate LLM response with retrieved context
    5. Store query and response in conversation history
    6. Return response with citations and timing

    Args:
        request: ChatRequest with session_id, query, optional context

    Returns:
        ChatResponse with answer, citations, and metadata
    """
    start_time = time.time()

    try:
        logger.info(
            f"Processing query for session {request.session_id}: "
            f'"{request.query[:50]}..."'
        )

        # Step 1: Get conversation context
        conversation_history = await conversation_service.get_conversation_context(
            request.session_id,
            max_pairs=5,
        )
        logger.debug(f"Retrieved {len(conversation_history)} messages from conversation history")

        # Step 2: Generate query embedding
        query_embedding = await embedder.embed_query(request.query)
        logger.debug("Generated query embedding")

        # Step 3: Vector search for relevant chunks
        retrieved_chunks = await vector_store.search(
            query_embedding=query_embedding,
            top_k=5,
            score_threshold=0.7,
        )
        logger.info(f"Retrieved {len(retrieved_chunks)} relevant chunks")

        if not retrieved_chunks:
            logger.warning("No relevant chunks found for query")

        # Step 4: Generate LLM response
        selected_text = request.context.selected_text if request.context else None
        response_text = await llm_service.generate_response(
            query=request.query,
            retrieved_chunks=retrieved_chunks,
            conversation_history=conversation_history,
            selected_text=selected_text,
        )
        logger.debug(f"Generated LLM response ({len(response_text)} chars)")

        # Step 5: Create citations from retrieved chunks
        citations = []
        for chunk_with_score in retrieved_chunks:
            chunk = chunk_with_score.chunk
            citation = Citation(
                source=f"{chunk.metadata.module_name}, {chunk.metadata.chapter_title}",
                excerpt=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                relevance=chunk_with_score.score,
            )
            citations.append(citation)

        # Step 6: Store messages in conversation history
        message_id = uuid4()

        # Store user message
        user_message = Message(
            message_id=uuid4(),
            session_id=request.session_id,
            role=MessageRole.USER,
            content=request.query,
            timestamp=datetime.utcnow(),
            source_citations=None,
        )
        await conversation_service.add_message(request.session_id, user_message)

        # Store assistant message with citations
        assistant_message = Message(
            message_id=message_id,
            session_id=request.session_id,
            role=MessageRole.ASSISTANT,
            content=response_text,
            timestamp=datetime.utcnow(),
            source_citations=[c.dict() for c in citations],
        )
        await conversation_service.add_message(request.session_id, assistant_message)

        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        # Optional: Generate suggested follow-up questions
        suggested_questions = []  # TODO: Implement in future iteration

        logger.info(
            f"Query processed successfully in {processing_time_ms}ms "
            f"(session: {request.session_id})"
        )

        return ChatResponse(
            message_id=message_id,
            response=response_text,
            citations=citations,
            suggested_questions=suggested_questions,
            processing_time_ms=processing_time_ms,
        )

    except Exception as e:
        logger.error(f"Query processing failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}",
        )


@router.get("/history/{session_id}")
async def get_conversation_history(session_id: str, limit: int = 20):
    """
    Retrieve conversation history for a session.

    Args:
        session_id: Conversation session ID
        limit: Maximum number of messages to return (default: 20)

    Returns:
        List of messages in chronological order
    """
    try:
        from uuid import UUID
        session_uuid = UUID(session_id)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid session ID format",
        )

    try:
        messages = await conversation_service.get_recent_messages(
            session_uuid,
            limit=limit,
        )

        logger.info(
            f"Retrieved {len(messages)} messages for session {session_id}"
        )

        return {
            "session_id": session_id,
            "message_count": len(messages),
            "messages": [
                {
                    "message_id": str(msg.message_id),
                    "role": msg.role.value,
                    "content": msg.content,
                    "timestamp": msg.timestamp.isoformat(),
                    "source_citations": msg.source_citations,
                }
                for msg in messages
            ],
        }

    except Exception as e:
        logger.error(f"Failed to retrieve conversation history: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve conversation history: {str(e)}",
        )
