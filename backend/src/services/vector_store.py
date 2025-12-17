# backend/src/services/vector_store.py (FINAL CODE - with delete_collection fix)

"""
Qdrant vector store service for document chunk storage and retrieval.
Handles collection setup, vector search, and chunk management.
"""

from typing import List, Optional
from uuid import UUID
import logging

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    SearchParams,
    HnswConfigDiff,
)

from ..core.config import settings
from ..models.document import DocumentChunk, ChunkWithScore, ChunkMetadata

logger = logging.getLogger(__name__)
# Vector dimension based on the embedding model used (Gemini's text-embedding-004 is 768)
VECTOR_DIMENSION = 768

class VectorStore:
    """Manages Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client and ensure collection exists."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name
        self._ensure_collection()

    def _ensure_collection(self) -> None:
        """
        Create Qdrant collection if it doesn't exist.
        """
        collections = self.client.get_collections().collections
        collection_exists = any(
            col.name == self.collection_name for col in collections
        )

        if not collection_exists:
            logger.info(f"Creating collection: {self.collection_name}")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                size=VECTOR_DIMENSION, 
                distance=Distance.COSINE,
                ),
                hnsw_config=HnswConfigDiff(
                    m=16, 
                    ef_construct=100,
                ),
            )
            logger.info(f"Collection {self.collection_name} created successfully")
        else:
            logger.info(f"Collection {self.collection_name} already exists")
    
    # *** CRITICAL FIX: ADDED delete_collection method ***
    def delete_collection(self) -> None:
        """
        Delete the entire collection if it exists. Used for re-indexing with --clear.
        """
        collections = self.client.get_collections().collections
        if self.collection_name in [col.name for col in collections]:
            logger.warning(f"Deleting collection: {self.collection_name}")
            self.client.delete_collection(collection_name=self.collection_name)
        else:
            logger.info(f"Collection {self.collection_name} does not exist for deletion.")
    # *** END CRITICAL FIX ***

    # FIX 1: Removed 'async' keyword to make the method synchronous
    def search( 
        self,
        query_embedding: List[float],
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
    ) -> List[ChunkWithScore]:
        """
        Search for similar document chunks using vector similarity.
        """
        top_k = top_k or settings.vector_search_top_k
        score_threshold = score_threshold or settings.vector_search_threshold

        # Search with HNSW ef parameter from research.md
        # Using query_points for compatibility with qdrant-client 1.7+
        search_results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
            score_threshold=score_threshold,
            search_params=SearchParams(hnsw_ef=64),
        ).points

        # Convert Qdrant results to ChunkWithScore models
        chunks_with_scores = []
        for result in search_results:
            chunk = DocumentChunk(
                id=UUID(result.id),
                content=result.payload["content"],
                metadata=ChunkMetadata(
                    source_file=result.payload["source_file"],
                    module_name=result.payload["module_name"],
                    chapter_title=result.payload["chapter_title"],
                    chunk_index=result.payload["chunk_index"],
                    token_count=result.payload["token_count"],
                ),
            )
            chunks_with_scores.append(
                ChunkWithScore(chunk=chunk, score=result.score)
            )

        logger.info(
            f"Vector search returned {len(chunks_with_scores)} chunks "
            f"(threshold: {score_threshold})"
        )
        return chunks_with_scores

    async def upsert_chunks(self, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> None:
        """
        Insert or update document chunks in the vector store.
        """
        if len(chunks) != len(embeddings):
            raise ValueError("Number of chunks must match number of embeddings")

        points = []
        for chunk, embedding in zip(chunks, embeddings):
            point = PointStruct(
                id=str(chunk.id),
                vector=embedding,
                payload={
                    "content": chunk.content,
                    "source_file": chunk.metadata.source_file,
                    "module_name": chunk.metadata.module_name,
                    "chapter_title": chunk.metadata.chapter_title,
                    "chunk_index": chunk.metadata.chunk_index,
                    "token_count": chunk.metadata.token_count,
                },
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            points=points,
        )
        logger.info(f"Upserted {len(points)} chunks to {self.collection_name}")

    async def delete_by_source(self, source_file: str) -> None:
        """
        Delete all chunks from a specific source file.
        """
        logger.warning(
            f"Delete by source not implemented. "
            f"To re-index {source_file}, use the delete_collection method or implement filtering."
        )

    def get_collection_info(self) -> dict:
        """
        Get collection statistics for monitoring.
        """
        collection_info = self.client.get_collection(self.collection_name)
        return {
            "name": self.collection_name,
            "points_count": collection_info.points_count,
            "status": collection_info.status,
        }

# Global instance (initialized on import)
vector_store = VectorStore()