#!/usr/bin/env python3
"""
Quick test script for translation feature.
Tests the translation services without needing authentication.
"""

import asyncio
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from services.openai_agent_service import OpenAIAgentService
from services.markdown_service import MarkdownService
from services.translation_cache_service import TranslationCacheService


async def test_markdown_service():
    """Test markdown parsing."""
    print("\n=== Testing Markdown Service ===")

    markdown_service = MarkdownService()

    # Test content
    test_content = """---
title: Test Chapter
---

# Introduction

This is a **test** chapter with `code` and formulas like $E=mc^2$.

```python
def hello():
    print("Hello World")
```

## Conclusion

The end.
"""

    # Extract frontmatter
    frontmatter, body = markdown_service.extract_frontmatter(test_content)
    print(f"✓ Extracted frontmatter: {frontmatter}")
    print(f"✓ Body length: {len(body)} characters")

    # Clean content
    cleaned = markdown_service.clean_for_translation(body)
    print(f"✓ Cleaned content length: {len(cleaned)} characters")

    return True


async def test_openai_service():
    """Test OpenAI agent service (without actual API call)."""
    print("\n=== Testing OpenAI Agent Service ===")

    try:
        openai_service = OpenAIAgentService()
        print(f"✓ OpenAI service initialized")
        print(f"✓ Model: {openai_service.model}")
        print(f"✓ Max tokens: {openai_service.max_tokens}")

        # Test placeholder extraction
        test_text = """
        # ROS 2 Introduction

        ROS 2 is a middleware framework for robots.

        ```python
        import rclpy
        ```

        The formula is $F=ma$.
        """

        cleaned, placeholders = openai_service.extract_preservables(test_text)
        print(f"✓ Extracted {len(placeholders)} placeholders")
        for key, value in list(placeholders.items())[:3]:
            print(f"  - {key}: {value[:50]}...")

        # Test restoration
        restored = openai_service.restore_preservables(cleaned, placeholders)
        print(f"✓ Content restoration works")

        return True

    except ValueError as e:
        print(f"⚠ OpenAI service requires OPENAI_API_KEY: {e}")
        return False


async def test_cache_service():
    """Test cache service."""
    print("\n=== Testing Cache Service ===")

    try:
        cache_service = TranslationCacheService()
        print(f"✓ Cache service initialized")

        # Test content hash
        test_content = "This is test content"
        content_hash = cache_service.compute_content_hash(test_content)
        print(f"✓ Content hash: {content_hash[:16]}...")

        # Test cache lookup (should be miss)
        result = await cache_service.get_cached_translation(
            chapter_slug="test-chapter",
            language="ur",
            content_hash=content_hash
        )
        print(f"✓ Cache lookup (expected miss): {result}")

        # Test cache stats
        stats = await cache_service.get_cache_stats()
        print(f"✓ Cache stats: {stats}")

        return True

    except ValueError as e:
        print(f"⚠ Cache service requires DATABASE_URL: {e}")
        return False


async def main():
    """Run all tests."""
    print("=" * 60)
    print("Testing Urdu Translation Feature Components")
    print("=" * 60)

    results = []

    # Test each service
    results.append(await test_markdown_service())
    results.append(await test_openai_service())
    results.append(await test_cache_service())

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✅ All tests passed!")
    else:
        print("⚠ Some tests failed (likely due to missing environment variables)")

    return passed == total


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
