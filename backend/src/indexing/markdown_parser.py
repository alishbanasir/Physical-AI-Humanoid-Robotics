"""
Markdown parser for Docusaurus textbook content.
Extracts content and metadata from markdown files.
"""

import re
from pathlib import Path
from typing import Dict, List, Optional
import logging

logger = logging.getLogger(__name__)


class MarkdownDocument:
    """Represents a parsed markdown document with metadata."""

    def __init__(
        self,
        file_path: str,
        content: str,
        frontmatter: Dict[str, str],
        module_name: str,
        chapter_title: str,
    ):
        self.file_path = file_path
        self.content = content
        self.frontmatter = frontmatter
        self.module_name = module_name
        self.chapter_title = chapter_title


class MarkdownParser:
    """
    Parses Docusaurus markdown files and extracts content with metadata.
    Handles frontmatter, module/chapter detection, and content cleaning.
    """

    def __init__(self, docs_root: str = "docs/textbook"):
        """
        Initialize parser with docs root directory.

        Args:
            docs_root: Root directory for textbook content
        """
        self.docs_root = Path(docs_root)

    def parse_file(self, file_path: str) -> MarkdownDocument:
        """
        Parse a single markdown file.

        Args:
            file_path: Path to markdown file

        Returns:
            MarkdownDocument with extracted content and metadata
        """
        path = Path(file_path)

        if not path.exists():
            raise FileNotFoundError(f"Markdown file not found: {file_path}")

        # Read file content
        with open(path, "r", encoding="utf-8") as f:
            raw_content = f.read()

        # Extract frontmatter
        frontmatter = self._extract_frontmatter(raw_content)

        # Remove frontmatter from content
        content = self._remove_frontmatter(raw_content)

        # Extract module and chapter info from path
        module_name, chapter_title = self._extract_metadata_from_path(path, frontmatter)

        # Clean content (remove Docusaurus-specific syntax)
        cleaned_content = self._clean_content(content)

        logger.info(f"Parsed {path.name}: {len(cleaned_content)} chars")

        return MarkdownDocument(
            file_path=str(path),
            content=cleaned_content,
            frontmatter=frontmatter,
            module_name=module_name,
            chapter_title=chapter_title,
        )

    def parse_directory(self, directory: Optional[str] = None) -> List[MarkdownDocument]:
        """
        Parse all markdown files in a directory recursively.

        Args:
            directory: Directory to parse (defaults to docs_root)

        Returns:
            List of parsed MarkdownDocuments
        """
        target_dir = Path(directory) if directory else self.docs_root

        if not target_dir.exists():
            raise FileNotFoundError(f"Directory not found: {target_dir}")

        # Find all markdown files
        md_files = list(target_dir.rglob("*.md"))
        logger.info(f"Found {len(md_files)} markdown files in {target_dir}")

        documents = []
        for md_file in md_files:
            try:
                doc = self.parse_file(str(md_file))
                documents.append(doc)
            except Exception as e:
                logger.error(f"Failed to parse {md_file}: {e}")
                continue

        logger.info(f"Successfully parsed {len(documents)}/{len(md_files)} files")
        return documents

    def _extract_frontmatter(self, content: str) -> Dict[str, str]:
        """
        Extract YAML frontmatter from markdown.

        Args:
            content: Raw markdown content

        Returns:
            Dict of frontmatter key-value pairs
        """
        frontmatter_pattern = r"^---\s*\n(.*?)\n---\s*\n"
        match = re.match(frontmatter_pattern, content, re.DOTALL)

        if not match:
            return {}

        frontmatter_text = match.group(1)
        frontmatter = {}

        # Simple YAML parsing (key: value pairs)
        for line in frontmatter_text.split("\n"):
            if ":" in line:
                key, value = line.split(":", 1)
                frontmatter[key.strip()] = value.strip().strip('"').strip("'")

        return frontmatter

    def _remove_frontmatter(self, content: str) -> str:
        """Remove YAML frontmatter from content."""
        frontmatter_pattern = r"^---\s*\n.*?\n---\s*\n"
        return re.sub(frontmatter_pattern, "", content, count=1, flags=re.DOTALL)

    def _extract_metadata_from_path(
        self, path: Path, frontmatter: Dict[str, str]
    ) -> tuple[str, str]:
        """
        Extract module name and chapter title from file path and frontmatter.

        Args:
            path: File path
            frontmatter: Parsed frontmatter

        Returns:
            (module_name, chapter_title) tuple
        """
        # Try to get title from frontmatter
        chapter_title = frontmatter.get("title", path.stem.replace("-", " ").title())

        # Extract module from path (e.g., docs/textbook/module-1/... -> Module 1)
        path_parts = path.parts
        module_name = "Unknown Module"

        for part in path_parts:
            if part.startswith("module-"):
                module_num = part.replace("module-", "").replace("-", " ").title()
                module_name = f"Module {module_num}"
                break

        # Try to get more descriptive module name from frontmatter or path
        if "sidebar_label" in frontmatter and "module" in frontmatter["sidebar_label"].lower():
            module_name = frontmatter["sidebar_label"]

        return module_name, chapter_title

    def _clean_content(self, content: str) -> str:
        """
        Clean markdown content by removing Docusaurus-specific syntax.

        Args:
            content: Raw markdown content

        Returns:
            Cleaned content suitable for embedding
        """
        # Remove import statements
        content = re.sub(r"^import .*?;?\s*$", "", content, flags=re.MULTILINE)

        # Remove JSX/React components (simple heuristic)
        content = re.sub(r"<[A-Z]\w+[^>]*>.*?</[A-Z]\w+>", "", content, flags=re.DOTALL)
        content = re.sub(r"<[A-Z]\w+[^>]*/?>", "", content)

        # Remove HTML comments
        content = re.sub(r"<!--.*?-->", "", content, flags=re.DOTALL)

        # Remove admonition syntax but keep content
        # :::note -> (keep content) -> :::
        content = re.sub(r":::(note|tip|info|warning|danger)\s*\n", "", content)
        content = re.sub(r":::\s*$", "", content, flags=re.MULTILINE)

        # Normalize whitespace (collapse multiple newlines)
        content = re.sub(r"\n{3,}", "\n\n", content)

        # Remove leading/trailing whitespace
        content = content.strip()

        return content


# Example usage
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    parser = MarkdownParser("docs/textbook")
    documents = parser.parse_directory()

    for doc in documents[:3]:  # Show first 3
        print(f"\n{'=' * 80}")
        print(f"File: {doc.file_path}")
        print(f"Module: {doc.module_name}")
        print(f"Chapter: {doc.chapter_title}")
        print(f"Content preview: {doc.content[:200]}...")
