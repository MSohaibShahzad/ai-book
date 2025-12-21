"""
Translation orchestration service.

Feature: 004-urdu-translation
Integrates: Cache service, Markdown service, OpenAI Agent service
"""

from typing import Dict, Optional
from pathlib import Path
from .translation_cache_service import TranslationCacheService
from .markdown_service import MarkdownService
from .openai_agent_service import OpenAIAgentService


class TranslationService:
    """
    Main orchestration service for translation workflow.

    Workflow:
    1. Check cache for existing translation
    2. If cache miss, read chapter markdown file
    3. Extract and preserve frontmatter, code blocks, formulas
    4. Translate content using OpenAI Agent
    5. Restore preserved elements
    6. Validate translation integrity
    7. Store in cache
    8. Return translated content
    """

    def __init__(self, textbook_docs_path: str = "textbook/docs"):
        """
        Initialize translation service with all dependencies.

        Args:
            textbook_docs_path: Path to textbook docs directory
        """
        self.cache_service = TranslationCacheService()
        self.markdown_service = MarkdownService()
        self.openai_service = OpenAIAgentService()
        self.textbook_docs_path = Path(textbook_docs_path)

    def get_chapter_path(self, chapter_slug: str) -> Path:
        """
        Resolve chapter markdown file path from URL-based slug.

        Folders now have NO NUMBERS (e.g., "preface/", "foundations-ros2/")
        URL slug matches folder structure directly.

        Example: URL "/docs/preface/how-to-use" → slug "preface/how-to-use"
        Maps to file "preface/how-to-use-this-book.md"

        Args:
            chapter_slug: URL-based slug (e.g., "preface/how-to-use", "preface", "vision-language-action/whisper-speech-recognition")

        Returns:
            Path to chapter markdown file

        Raises:
            FileNotFoundError: If chapter file doesn't exist
        """
        # Normalize slug (remove leading/trailing slashes)
        normalized_slug = chapter_slug.strip('/')

        # Search all markdown files in docs directory recursively
        for file_path in self.textbook_docs_path.rglob('*.md'):
            try:
                content = file_path.read_text(encoding='utf-8')
                frontmatter, _ = self.markdown_service.extract_frontmatter(content)

                if not frontmatter or 'slug' not in frontmatter:
                    continue

                file_slug = frontmatter['slug'].strip('/')

                # Get parent directory name (e.g., "preface", "foundations-ros2")
                parent_dir = file_path.parent.name

                # Construct full URL slug based on file's frontmatter slug
                if file_slug.startswith(parent_dir + '/'):
                    # Slug already includes parent (e.g., "vision-language-action/whisper-speech-recognition")
                    full_url_slug = file_slug
                elif file_slug == parent_dir:
                    # This is the index file (e.g., slug="/preface" for preface/index.md)
                    full_url_slug = parent_dir
                elif '/' not in file_slug:
                    # Slug is relative to parent (e.g., "how-to-use" in preface directory)
                    full_url_slug = f"{parent_dir}/{file_slug}"
                else:
                    # Slug has its own path structure
                    full_url_slug = file_slug

                # Check if this matches the requested slug
                if full_url_slug == normalized_slug:
                    return file_path

            except Exception:
                # Skip files that can't be read or parsed
                continue

        # Also search .mdx files
        for file_path in self.textbook_docs_path.rglob('*.mdx'):
            try:
                content = file_path.read_text(encoding='utf-8')
                frontmatter, _ = self.markdown_service.extract_frontmatter(content)

                if not frontmatter or 'slug' not in frontmatter:
                    continue

                file_slug = frontmatter['slug'].strip('/')

                # Get parent directory name
                parent_dir = file_path.parent.name

                # Construct full URL slug
                if file_slug.startswith(parent_dir + '/'):
                    full_url_slug = file_slug
                elif file_slug == parent_dir:
                    full_url_slug = parent_dir
                elif '/' not in file_slug:
                    full_url_slug = f"{parent_dir}/{file_slug}"
                else:
                    full_url_slug = file_slug

                # Check if this matches the requested slug
                if full_url_slug == normalized_slug:
                    return file_path

            except Exception:
                # Skip files that can't be read or parsed
                continue

        raise FileNotFoundError(f"Chapter file not found for slug: {chapter_slug}")

    async def translate_chapter(
        self,
        chapter_slug: str,
        target_language: str = "ur"
    ) -> Dict:
        """
        Translate a chapter to target language.

        Args:
            chapter_slug: Chapter identifier
            target_language: Target language code (default: "ur" for Urdu)

        Returns:
            Dictionary with:
                - translated_content: Full translated markdown
                - from_cache: Whether served from cache
                - cached_at: When originally cached (if from cache)

        Raises:
            FileNotFoundError: If chapter doesn't exist
            Exception: If translation fails
        """
        # Step 1: Read chapter content
        try:
            chapter_path = self.get_chapter_path(chapter_slug)
            original_content = chapter_path.read_text(encoding='utf-8')
        except FileNotFoundError as e:
            raise FileNotFoundError(f"Chapter not found: {chapter_slug}") from e

        # Step 2: Compute content hash for cache versioning
        content_hash = self.cache_service.compute_content_hash(original_content)

        # Step 3: Check cache
        cached_result = await self.cache_service.get_cached_translation(
            chapter_slug=chapter_slug,
            language=target_language,
            content_hash=content_hash
        )

        if cached_result:
            return cached_result

        # Step 4: Cache miss - translate content
        print(f"[TRANSLATION START] {chapter_slug} → {target_language}")

        # Extract frontmatter
        frontmatter, content_body = self.markdown_service.extract_frontmatter(
            original_content
        )

        # Clean content for translation
        cleaned_content = self.markdown_service.clean_for_translation(content_body)

        # Translate content using OpenAI Agent
        try:
            translated_body = await self.openai_service.translate_to_urdu(
                cleaned_content
            )
        except Exception as e:
            print(f"[ERROR] Translation failed for {chapter_slug}: {e}")
            raise Exception(f"Translation failed: {str(e)}") from e

        # Validate translation
        validation_errors = self.markdown_service.validate_translation(
            original=content_body,
            translated=translated_body
        )

        if validation_errors:
            print(f"[WARNING] Translation validation issues: {validation_errors}")
            # Continue despite warnings (non-critical validation)

        # Restore frontmatter (keep in English for now)
        # TODO: Translate frontmatter fields in future iteration
        final_content = self.markdown_service.restore_frontmatter(
            frontmatter,
            translated_body
        )

        # Step 5: Store in cache
        cache_stored = await self.cache_service.store_translation(
            chapter_slug=chapter_slug,
            language=target_language,
            content_hash=content_hash,
            translated_content=final_content,
            ttl_days=90  # Default TTL
        )

        if not cache_stored:
            print(f"[WARNING] Failed to cache translation for {chapter_slug}")

        print(f"[TRANSLATION COMPLETE] {chapter_slug}")

        return {
            "translated_content": final_content,
            "from_cache": False,
            "cached_at": None
        }

    async def invalidate_chapter_cache(
        self,
        chapter_slug: str,
        language: Optional[str] = None
    ) -> int:
        """
        Invalidate cached translation for a chapter.

        Args:
            chapter_slug: Chapter identifier
            language: Optional language filter

        Returns:
            Number of cache entries deleted
        """
        return await self.cache_service.invalidate_cache(
            chapter_slug=chapter_slug,
            language=language
        )

    async def get_translation_stats(self) -> Dict:
        """
        Get translation and cache statistics.

        Returns:
            Dictionary with cache metrics
        """
        return await self.cache_service.get_cache_stats()
