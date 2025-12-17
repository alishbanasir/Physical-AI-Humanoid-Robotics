"""
Semantic chunker for textbook content.
Implements 800-token chunks with 100-token overlap, preserving code blocks and semantic boundaries.
"""

import re
from typing import List
from uuid import uuid4
import logging

import tiktoken

# CRITICAL FIX: Change relative import to absolute import (from ..models.document to src.models.document)
from src.models.document import DocumentChunk, ChunkMetadata 

logger = logging.getLogger(__name__)


class SemanticChunker:
    """
    Chunks markdown content into semantically meaningful segments.
    Configuration from research.md:
    - Chunk size: 800 tokens
    - Overlap: 100 tokens
    - Preserve code blocks and semantic boundaries
    """

    def __init__(
        self,
        chunk_size: int = 800,
        chunk_overlap: int = 100,
        encoding_name: str = "cl100k_base",  # GPT-4 tokenizer
    ):
        """
        Initialize chunker with token-based configuration.

        Args:
            chunk_size: Target tokens per chunk
            chunk_overlap: Overlap tokens between chunks
            encoding_name: tiktoken encoding name
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.encoding = tiktoken.get_encoding(encoding_name)

    def chunk_document(
        self,
        content: str,
        source_file: str,
        module_name: str,
        chapter_title: str,
    ) -> List[DocumentChunk]:
        """
        Chunk a document into semantically meaningful segments.

        Args:
            content: Document content
            source_file: Source file path
            module_name: Module name (e.g., "Module 1: ROS 2 Fundamentals")
            chapter_title: Chapter title

        Returns:
            List of DocumentChunks with metadata
        """
        # Split content into semantic sections
        sections = self._split_into_sections(content)

        chunks = []
        chunk_index = 0
        current_chunk_tokens = []
        current_chunk_text = []

        for section in sections:
            section_tokens = self.encoding.encode(section)

            # If section fits in current chunk, add it
            if len(current_chunk_tokens) + len(section_tokens) <= self.chunk_size:
                current_chunk_tokens.extend(section_tokens)
                current_chunk_text.append(section)
            else:
                # Save current chunk if it has content
                if current_chunk_tokens:
                    chunk = self._create_chunk(
                        current_chunk_text,
                        current_chunk_tokens,
                        chunk_index,
                        source_file,
                        module_name,
                        chapter_title,
                    )
                    chunks.append(chunk)
                    chunk_index += 1

                # Start new chunk with overlap
                if chunks:
                    # Take last chunk_overlap tokens from previous chunk
                    overlap_tokens = current_chunk_tokens[-self.chunk_overlap :]
                    overlap_text = self.encoding.decode(overlap_tokens)
                    current_chunk_tokens = overlap_tokens
                    current_chunk_text = [overlap_text]
                else:
                    current_chunk_tokens = []
                    current_chunk_text = []

                # Add current section
                # If section itself is larger than chunk_size, split it
                if len(section_tokens) > self.chunk_size:
                    section_chunks = self._split_large_section(
                        section, section_tokens
                    )
                    for i, (sec_text, sec_tokens) in enumerate(section_chunks):
                        if i == 0:
                            # First part goes into current chunk
                            current_chunk_tokens.extend(sec_tokens)
                            current_chunk_text.append(sec_text)
                        else:
                            # Subsequent parts become their own chunks
                            chunk = self._create_chunk(
                                [sec_text],
                                sec_tokens,
                                chunk_index,
                                source_file,
                                module_name,
                                chapter_title,
                            )
                            chunks.append(chunk)
                            chunk_index += 1
                else:
                    current_chunk_tokens.extend(section_tokens)
                    current_chunk_text.append(section)

        # Add final chunk
        if current_chunk_tokens:
            chunk = self._create_chunk(
                current_chunk_text,
                current_chunk_tokens,
                chunk_index,
                source_file,
                module_name,
                chapter_title,
            )
            chunks.append(chunk)

        logger.info(
            f"Chunked {source_file} into {len(chunks)} chunks "
            f"(avg {sum(c.metadata.token_count for c in chunks) // len(chunks) if chunks else 0} tokens/chunk)"
        )
        return chunks

    def _split_into_sections(self, content: str) -> List[str]:
        """
        Split content into semantic sections.
        Preserves:
        - Headers (##, ###, etc.)
        - Code blocks (```...```)
        - Paragraphs

        Args:
            content: Markdown content

        Returns:
            List of section strings
        """
        sections = []

        # Extract code blocks first (preserve them intact)
        code_block_pattern = r"```[\s\S]*?```"
        code_blocks = re.findall(code_block_pattern, content)

        # Replace code blocks with placeholders
        content_with_placeholders = content
        for i, code_block in enumerate(code_blocks):
            placeholder = f"__CODE_BLOCK_{i}__"
            content_with_placeholders = content_with_placeholders.replace(
                code_block, placeholder, 1
            )

        # Split on headers and paragraphs
        # Split on ## headers or double newlines
        parts = re.split(r"(^#{2,}.*$)", content_with_placeholders, flags=re.MULTILINE)

        # Restore code blocks
        restored_parts = []
        for part in parts:
            for i, code_block in enumerate(code_blocks):
                placeholder = f"__CODE_BLOCK_{i}__"
                part = part.replace(placeholder, code_block)
            restored_parts.append(part)

        # Combine adjacent non-header parts
        current_section = []
        for part in restored_parts:
            if part.strip():
                if part.strip().startswith("#"):
                    # Start new section with header
                    if current_section:
                        sections.append("\n".join(current_section))
                    current_section = [part]
                else:
                    current_section.append(part)

        # Add final section
        if current_section:
            sections.append("\n".join(current_section))

        # Further split large sections by paragraphs
        final_sections = []
        for section in sections:
            if self._get_token_count(section) > self.chunk_size * 1.5:
                # Split by paragraphs
                paragraphs = section.split("\n\n")
                for para in paragraphs:
                    if para.strip():
                        final_sections.append(para)
            else:
                final_sections.append(section)

        return [s for s in final_sections if s.strip()]

    def _split_large_section(
        self, section: str, section_tokens: List[int]
    ) -> List[tuple[str, List[int]]]:
        """
        Split a large section that exceeds chunk_size.

        Args:
            section: Section text
            section_tokens: Tokenized section

        Returns:
            List of (text, tokens) tuples
        """
        chunks = []
        start = 0

        while start < len(section_tokens):
            end = min(start + self.chunk_size, len(section_tokens))
            chunk_tokens = section_tokens[start:end]
            chunk_text = self.encoding.decode(chunk_tokens)
            chunks.append((chunk_text, chunk_tokens))
            start = end - self.chunk_overlap  # Add overlap

        return chunks

    def _create_chunk(
        self,
        texts: List[str],
        tokens: List[int],
        index: int,
        source_file: str,
        module_name: str,
        chapter_title: str,
    ) -> DocumentChunk:
        """
        Create a DocumentChunk from text and tokens.

        Args:
            texts: List of text segments
            tokens: Token list
            index: Chunk index
            source_file: Source file path
            module_name: Module name
            chapter_title: Chapter title

        Returns:
            DocumentChunk with metadata
        """
        content = "\n".join(texts).strip()
        token_count = len(tokens)

        return DocumentChunk(
            id=uuid4(),
            content=content,
            metadata=ChunkMetadata(
                source_file=source_file,
                module_name=module_name,
                chapter_title=chapter_title,
                chunk_index=index,
                token_count=token_count,
            ),
        )

    def _get_token_count(self, text: str) -> int:
        """Get token count for text."""
        return len(self.encoding.encode(text))


# Example usage
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    chunker = SemanticChunker(chunk_size=800, chunk_overlap=100)

    sample_text = """
# Chapter 1: ROS 2 Architecture

ROS 2 is the next generation of the Robot Operating System.

## Core Concepts

ROS 2 introduces several key improvements over ROS 1.

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')"""