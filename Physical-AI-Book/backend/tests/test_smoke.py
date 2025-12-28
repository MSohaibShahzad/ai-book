"""
Smoke tests for personalization feature

Quick tests to verify core functionality works.
These tests match the actual implementation.

Feature: 005-chapter-personalization
"""

import pytest
from personalization.agent import protect_technical_elements, reconstruct_markdown


class TestProtectionSmoke:
    """Smoke tests for technical element protection"""

    def test_protects_code_blocks(self):
        """Test code blocks are protected"""
        markdown = """
```python
def hello():
    print("world")
```
"""
        protected, protection_map = protect_technical_elements(markdown)

        # Should have placeholder
        assert "{{CODE_" in protected
        # Should not have original code
        assert "def hello" not in protected
        # Protection map should have the code
        assert len(protection_map) > 0
        code_found = any("def hello" in v for v in protection_map.values())
        assert code_found

    def test_protects_math(self):
        """Test math formulas are protected"""
        markdown = "Formula: $E = mc^2$"

        protected, protection_map = protect_technical_elements(markdown)

        # Should have placeholder
        assert "{{MATH_" in protected
        # Should not have original formula
        assert "$E = mc^2$" not in protected
        # Protection map should have the formula
        math_found = any("E = mc^2" in v for v in protection_map.values())
        assert math_found

    def test_protects_frontmatter(self):
        """Test YAML frontmatter is protected"""
        markdown = """---
title: Test
---

# Content
"""
        protected, protection_map = protect_technical_elements(markdown)

        # Should have placeholder
        assert "{{FRONTMATTER_" in protected
        # Protection map should have frontmatter
        frontmatter_found = any("title: Test" in v for v in protection_map.values())
        assert frontmatter_found

    def test_reconstruct_works(self):
        """Test reconstruction replaces placeholders"""
        # First protect
        original = "Code: ```python\ntest()\n```"
        protected, protection_map = protect_technical_elements(original)

        # Then reconstruct
        reconstructed = reconstruct_markdown(protected, protection_map)

        # Should have original code back
        assert "```python" in reconstructed
        assert "test()" in reconstructed

    def test_end_to_end_preservation(self):
        """Test complete protect-reconstruct cycle"""
        original = """---
title: Chapter
---

# Test

Code: ```python
x = 1
```

Math: $E=mc^2$

$$
y = mx + b
$$
"""
        # Protect
        protected, protection_map = protect_technical_elements(original)

        # Verify protection
        assert "{{CODE_" in protected
        assert "{{MATH_" in protected
        assert "{{FRONTMATTER_" in protected
        assert "x = 1" not in protected

        # Reconstruct
        reconstructed = reconstruct_markdown(protected, protection_map)

        # Verify reconstruction
        assert "title: Chapter" in reconstructed
        assert "```python" in reconstructed
        assert "x = 1" in reconstructed
        assert "$E=mc^2$" in reconstructed
        assert "$$" in reconstructed
        assert "y = mx + b" in reconstructed


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
