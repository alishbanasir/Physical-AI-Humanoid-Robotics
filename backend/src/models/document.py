"""
Pydantic models for document chunks and metadata.
"""

from uuid import UUID
from pydantic import BaseModel, Field


class ChunkMetadata(BaseModel):
    """Metadata for a document chunk."""
    source_file: str  # e.g., "docs/textbook/module-1/chapter-1.md"
    module_name: str
    chapter_title: str
    chunk_index: int
    token_count: int = Field(..., ge=100, le=1500)


class DocumentChunk(BaseModel):
    """Represents a semantically chunked segment of textbook content."""
    id: UUID
    content: str
    metadata: ChunkMetadata


class ChunkWithScore(BaseModel):
    """Document chunk with relevance score from vector search."""
    chunk: DocumentChunk
    score: float = Field(..., ge=0.0, le=1.0)
