"""
Text chunking service for textbook content.
Implements heading-based chunking strategy per research.md.
"""
import re
import tiktoken
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class Chunk:
    """Represents a single text chunk."""
    text: str
    token_count: int
    section_heading: Optional[str]
    chunk_index: int


class TextbookChunker:
    """Chunker for textbook markdown content using heading-based strategy."""

    def __init__(
        self,
        target_chunk_size: int = 400,
        overlap_tokens: int = 50,
        min_chunk_size: int = 100,
        max_chunk_size: int = 600
    ):
        """
        Initialize chunker with size parameters.

        Args:
            target_chunk_size: Target chunk size in tokens (default 400)
            overlap_tokens: Overlap between chunks (default 50)
            min_chunk_size: Minimum chunk size (default 100)
            max_chunk_size: Maximum chunk size (default 600)
        """
        self.target_chunk_size = target_chunk_size
        self.overlap_tokens = overlap_tokens
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size
        self.encoding = tiktoken.encoding_for_model("gpt-4")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken."""
        return len(self.encoding.encode(text))

    def split_by_headings(self, markdown_content: str) -> List[tuple[Optional[str], str]]:
        """
        Split markdown content by headings.

        Returns:
            List of (heading, content) tuples
        """
        # Match markdown headings (## or ###)
        heading_pattern = r'^(#{2,3})\s+(.+)$'
        lines = markdown_content.split('\n')

        sections = []
        current_heading = None
        current_content = []

        for line in lines:
            heading_match = re.match(heading_pattern, line)
            if heading_match:
                # Save previous section
                if current_content:
                    sections.append((current_heading, '\n'.join(current_content)))
                # Start new section
                current_heading = heading_match.group(2).strip()
                current_content = []
            else:
                current_content.append(line)

        # Add final section
        if current_content:
            sections.append((current_heading, '\n'.join(current_content)))

        return sections

    def chunk_section(self, heading: Optional[str], content: str) -> List[str]:
        """
        Chunk a single section into target-sized pieces.

        Args:
            heading: Section heading (or None)
            content: Section content

        Returns:
            List of chunk texts
        """
        content = content.strip()
        if not content:
            return []

        token_count = self.count_tokens(content)

        # If section fits in one chunk, return as-is
        if token_count <= self.max_chunk_size:
            return [content]

        # Split into sentences
        sentences = re.split(r'(?<=[.!?])\s+', content)
        chunks = []
        current_chunk = []
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = self.count_tokens(sentence)

            # If adding sentence exceeds max, save current chunk
            if current_tokens + sentence_tokens > self.target_chunk_size and current_chunk:
                chunks.append(' '.join(current_chunk))
                # Keep overlap
                overlap_text = ' '.join(current_chunk[-2:]) if len(current_chunk) >= 2 else ''
                current_chunk = [overlap_text] if overlap_text else []
                current_tokens = self.count_tokens(overlap_text)

            current_chunk.append(sentence)
            current_tokens += sentence_tokens

        # Add final chunk
        if current_chunk:
            chunks.append(' '.join(current_chunk))

        return chunks

    def chunk_page(self, content: str) -> List[Chunk]:
        """
        Chunk a textbook page into multiple chunks.

        Args:
            content: Markdown content from textbook page

        Returns:
            List of Chunk objects
        """
        sections = self.split_by_headings(content)
        all_chunks = []
        chunk_index = 0

        for heading, section_content in sections:
            chunk_texts = self.chunk_section(heading, section_content)

            for chunk_text in chunk_texts:
                token_count = self.count_tokens(chunk_text)

                # Skip chunks that are too small
                if token_count < self.min_chunk_size:
                    continue

                all_chunks.append(Chunk(
                    text=chunk_text,
                    token_count=token_count,
                    section_heading=heading,
                    chunk_index=chunk_index
                ))
                chunk_index += 1

        return all_chunks
