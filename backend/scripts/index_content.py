#!/usr/bin/env python3
"""
Content indexing script for RAG chatbot.
Parses docs/textbook/, chunks content, generates embeddings, and stores in Qdrant.

Usage:
    python scripts/index_content.py [--docs-dir docs/textbook] [--clear]
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

# CRITICAL FIX: Add backend/src to Python path so absolute imports work
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.indexing.markdown_parser import MarkdownParser
from src.indexing.chunker import SemanticChunker
from src.services.embedder import embedder
from src.services.vector_store import vector_store
from src.core.config import settings

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def index_documents(docs_dir: str, clear_existing: bool = False) -> None:
    """
    Index all documents from docs directory.
    """
    logger.info("=" * 80)
    logger.info("Starting content indexing process")
    logger.info(f"Docs directory: {docs_dir}")
    logger.info(f"Clear existing: {clear_existing}")
    logger.info("=" * 80)

    # Step 0: Clear collection if requested (CRITICAL FIX: ADDED RECREATION LOGIC)
    if clear_existing:
        # 1. Delete collection using the new method (yeh kaam kar gaya tha)
        vector_store.delete_collection()
        
        # 2. *** CRITICAL FIX: Re-create the collection immediately ***
        # Jab humne delete kiya to usay dobara banana zaroori hai
        vector_store._ensure_collection() 
        logger.info(f"Collection {vector_store.collection_name} has been cleared and RECREATED.")

    # Step 1: Parse markdown files
    logger.info("\n[1/4] Parsing markdown files...")
    parser = MarkdownParser(docs_dir)
    documents = parser.parse_directory()
# ... (Baaki poori file waisi hi rahegi) ...
# ... (Line 59 se aage) ...
    if not documents:
        logger.warning("No documents found to index!")
        return

    logger.info(f"Parsed {len(documents)} documents")

    # Step 2: Chunk documents
    logger.info("\n[2/4] Chunking documents...")
    chunker = SemanticChunker(
        chunk_size=settings.chunk_size,
        chunk_overlap=settings.chunk_overlap,
    )

    all_chunks = []
    for doc in documents:
        try:
            chunks = chunker.chunk_document(
                content=doc.content,
                source_file=doc.file_path,
                module_name=doc.module_name,
                chapter_title=doc.chapter_title,
            )
            all_chunks.extend(chunks)
        except Exception as e:
            logger.error(f"Failed to chunk {doc.file_path}: {e}")
            continue

    logger.info(f"Created {len(all_chunks)} chunks from {len(documents)} documents")

    if not all_chunks:
        logger.warning("No chunks created!")
        return

    # Step 3: Generate embeddings
    logger.info("\n[3/4] Generating embeddings...")
    chunk_texts = [chunk.content for chunk in all_chunks]

    # Process in batches to avoid API limits
    batch_size = 100
    all_embeddings = []

    for i in range(0, len(chunk_texts), batch_size):
        batch = chunk_texts[i : i + batch_size]
        logger.info(
            f"Processing batch {i // batch_size + 1}/{(len(chunk_texts) + batch_size - 1) // batch_size} "
            f"({len(batch)} chunks)"
        )

        try:
            embeddings = await embedder.embed_documents(batch)
            all_embeddings.extend(embeddings)
        except Exception as e:
            logger.error(f"Failed to generate embeddings for batch {i // batch_size + 1}: {e}")
            continue

    if len(all_embeddings) != len(all_chunks):
        logger.warning(
            f"Embedding count mismatch: {len(all_embeddings)} embeddings for {len(all_chunks)} chunks"
        )
        all_chunks = all_chunks[: len(all_embeddings)]

    logger.info(f"Generated {len(all_embeddings)} embeddings")

    # Step 4: Store in Qdrant
    logger.info("\n[4/4] Storing in Qdrant...")
    
    # Upload in batches
    batch_size = 100
    for i in range(0, len(all_chunks), batch_size):
        chunk_batch = all_chunks[i : i + batch_size]
        embedding_batch = all_embeddings[i : i + batch_size]

        logger.info(
            f"Uploading batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size} "
            f"({len(chunk_batch)} chunks)"
        )

        try:
            await vector_store.upsert_chunks(chunk_batch, embedding_batch)
        except Exception as e:
            logger.error(f"Failed to upload batch {i // batch_size + 1}: {e}")
            continue

    # Final statistics
    logger.info("\n" + "=" * 80)
    logger.info("Indexing complete!")
    collection_info = vector_store.get_collection_info()
    logger.info(f"Collection: {collection_info['name']}")
    logger.info(f"Total vectors: {collection_info['points_count']}") 
    logger.info(f"Status: {collection_info['status']}")
    logger.info("=" * 80)

def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Index textbook content for RAG chatbot"
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="../docs/textbook",
        help="Root directory for textbook content (default: ../docs/textbook)",
    )
    parser.add_argument(
        "--clear",
        action="store_true",
        help="Clear existing vectors before indexing",
    )

    args = parser.parse_args()

    # Validate docs directory
    docs_path = Path(args.docs_dir)
    if not docs_path.exists():
        logger.error(f"Docs directory not found: {args.docs_dir}")
        sys.exit(1)

    # Run async indexing
    try:
        asyncio.run(index_documents(args.docs_dir, args.clear))
    except KeyboardInterrupt:
        logger.info("\nIndexing interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Indexing failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()