"""
Textbook parser for extracting markdown content with metadata.
Parses Docusaurus markdown files and extracts frontmatter.
"""
import re
import yaml
from pathlib import Path
from typing import List, Dict, Optional
from dataclasses import dataclass


@dataclass
class TextbookPage:
    """Represents a single textbook page with metadata."""
    file_path: str
    module_name: str
    chapter_name: str
    slug: str
    content: str
    frontmatter: dict


class TextbookParser:
    """Parser for Docusaurus markdown files."""

    def __init__(self, textbook_root: Path):
        """
        Initialize parser with textbook root directory.

        Args:
            textbook_root: Path to textbook/docs/ directory
        """
        self.textbook_root = Path(textbook_root)

    def parse_file(self, file_path: Path) -> Optional[TextbookPage]:
        """
        Parse a single markdown file.

        Args:
            file_path: Path to markdown file

        Returns:
            TextbookPage object or None if parsing fails
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract frontmatter
            frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)', content, re.DOTALL)
            if not frontmatter_match:
                # No frontmatter, use defaults
                frontmatter = {}
                markdown_content = content
            else:
                frontmatter_str, markdown_content = frontmatter_match.groups()
                frontmatter = yaml.safe_load(frontmatter_str) or {}

            # Extract metadata from file path
            relative_path = file_path.relative_to(self.textbook_root)
            path_parts = relative_path.parts

            # Infer module and chapter from path
            module_name = path_parts[0] if len(path_parts) > 0 else "Unknown Module"
            chapter_name = frontmatter.get('title', file_path.stem)

            # Generate slug
            slug = "/" + str(relative_path.with_suffix('')).replace("\\", "/")
            if not slug.startswith("/docs/"):
                slug = "/docs" + slug

            return TextbookPage(
                file_path=str(file_path),
                module_name=module_name.replace('-', ' ').title(),
                chapter_name=chapter_name,
                slug=slug,
                content=markdown_content.strip(),
                frontmatter=frontmatter
            )
        except Exception as e:
            print(f"Error parsing {file_path}: {e}")
            return None

    def parse_all(self) -> List[TextbookPage]:
        """
        Parse all markdown files in textbook directory.

        Returns:
            List of TextbookPage objects
        """
        pages = []
        for md_file in self.textbook_root.rglob("*.md"):
            page = self.parse_file(md_file)
            if page:
                pages.append(page)
        return pages

    def filter_by_module(self, pages: List[TextbookPage], module_name: str) -> List[TextbookPage]:
        """Filter pages by module name."""
        return [p for p in pages if p.module_name.lower() == module_name.lower()]
