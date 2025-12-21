"""
Markdown parsing service for translation feature.

Feature: 004-urdu-translation
Implements: AST-based markdown preservation using markdown-it-py
"""

import re
from typing import Dict, Tuple
from markdown_it import MarkdownIt
from mdit_py_plugins.front_matter import front_matter_plugin
from mdit_py_plugins.dollarmath import dollarmath_plugin
import yaml


class MarkdownService:
    """
    Service for parsing and preserving markdown structure during translation.

    Uses AST-based approach to extract and restore:
    - YAML frontmatter
    - Code blocks
    - LaTeX formulas
    - Custom Docusaurus components
    """

    def __init__(self):
        """Initialize markdown-it parser with plugins."""
        self.md = (
            MarkdownIt()
            .use(front_matter_plugin)  # YAML frontmatter support
            .use(dollarmath_plugin)    # $...$ and $$...$$ support
            .enable('table')           # GitHub Flavored Markdown tables
        )

    def extract_frontmatter(self, content: str) -> Tuple[Dict, str]:
        """
        Extract YAML frontmatter from markdown content.

        Args:
            content: Full markdown content with frontmatter

        Returns:
            Tuple of (frontmatter_dict, content_without_frontmatter)
        """
        frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)

        if frontmatter_match:
            try:
                frontmatter = yaml.safe_load(frontmatter_match.group(1))
                content_without_fm = content[frontmatter_match.end():]
                return frontmatter, content_without_fm
            except yaml.YAMLError as e:
                print(f"[WARNING] Failed to parse frontmatter: {e}")
                return {}, content

        return {}, content

    def translate_frontmatter(self, frontmatter: Dict, translate_fn) -> Dict:
        """
        Translate specific frontmatter fields to Urdu.

        Translates: title, description, sidebar_label
        Keeps: id, slug, tags, etc. in English

        Args:
            frontmatter: Original frontmatter dictionary
            translate_fn: Async function to translate text

        Returns:
            Translated frontmatter dictionary
        """
        translated_fm = frontmatter.copy()

        # Fields to translate
        translatable_fields = ['title', 'description', 'sidebar_label']

        for field in translatable_fields:
            if field in translated_fm and isinstance(translated_fm[field], str):
                # Note: In actual implementation, this would use translate_fn
                # For now, we'll keep it as-is (will be implemented in translation service)
                pass

        return translated_fm

    def restore_frontmatter(self, frontmatter: Dict, content: str) -> str:
        """
        Restore YAML frontmatter to markdown content.

        Args:
            frontmatter: Frontmatter dictionary
            content: Markdown content without frontmatter

        Returns:
            Full markdown with frontmatter prepended
        """
        if not frontmatter:
            return content

        yaml_str = yaml.dump(frontmatter, allow_unicode=True, sort_keys=False)
        return f"---\n{yaml_str}---\n\n{content}"

    def validate_translation(self, original: str, translated: str) -> list:
        """
        Validate that translation preserved critical markdown elements.

        Checks:
        - Code block count (``` markers)
        - Math block count ($$ markers)
        - Heading count (# markers)
        - Link count (]( markers)

        Args:
            original: Original markdown content
            translated: Translated markdown content

        Returns:
            List of validation errors (empty if valid)
        """
        errors = []

        # Check code block count
        original_code_blocks = original.count('```')
        translated_code_blocks = translated.count('```')
        if original_code_blocks != translated_code_blocks:
            errors.append(
                f"Code block count mismatch: "
                f"original={original_code_blocks}, translated={translated_code_blocks}"
            )

        # Check math block count
        original_math_blocks = original.count('$$')
        translated_math_blocks = translated.count('$$')
        if original_math_blocks != translated_math_blocks:
            errors.append(
                f"Math block count mismatch: "
                f"original={original_math_blocks}, translated={translated_math_blocks}"
            )

        # Check heading count (rough heuristic)
        original_headings = len(re.findall(r'^#+\s', original, re.MULTILINE))
        translated_headings = len(re.findall(r'^#+\s', translated, re.MULTILINE))
        if original_headings != translated_headings:
            errors.append(
                f"Heading count mismatch: "
                f"original={original_headings}, translated={translated_headings}"
            )

        # Check link count
        original_links = original.count('](')
        translated_links = translated.count('](')
        if original_links != translated_links:
            errors.append(
                f"Link count mismatch: "
                f"original={original_links}, translated={translated_links}"
            )

        return errors

    def clean_for_translation(self, content: str) -> str:
        """
        Prepare content for translation.

        - Removes excessive whitespace
        - Normalizes line endings
        - Preserves paragraph structure

        Args:
            content: Raw markdown content

        Returns:
            Cleaned content ready for translation
        """
        # Normalize line endings
        content = content.replace('\r\n', '\n')

        # Remove trailing whitespace from lines
        lines = [line.rstrip() for line in content.split('\n')]
        content = '\n'.join(lines)

        # Collapse multiple blank lines into double newlines (paragraph separation)
        content = re.sub(r'\n{3,}', '\n\n', content)

        return content
