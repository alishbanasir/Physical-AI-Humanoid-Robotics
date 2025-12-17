"""
Pydantic models for chat API requests and responses.
"""

from typing import List, Optional
from uuid import UUID
from pydantic import BaseModel, Field


class ChatContext(BaseModel):
    """Context for chat query (optional)."""
    current_page: Optional[str] = None
    selected_text: Optional[str] = Field(None, max_length=500)


class ChatRequest(BaseModel):
    """Request model for POST /api/chat/query."""
    session_id: UUID
    query: str = Field(..., min_length=1, max_length=1000)
    context: Optional[ChatContext] = None


class Citation(BaseModel):
    """Source citation for chatbot response."""
    source: str  # e.g., "Module 1, Chapter 2"
    excerpt: str
    relevance: float = Field(..., ge=0.0, le=1.0)


class ChatResponse(BaseModel):
    """Response model for POST /api/chat/query."""
    message_id: UUID
    response: str  # Markdown-formatted answer
    citations: List[Citation]
    suggested_questions: List[str] = []
    processing_time_ms: int
