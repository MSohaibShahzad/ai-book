"""
Tests for Personalization Agent

Tests verify that the AI agent:
1. Preserves code blocks 100% accurately
2. Preserves LaTeX formulas 100% accurately
3. Preserves YAML frontmatter 100% accurately
4. Preserves images and links
5. Handles placeholders correctly
6. Timeout enforcement works

Feature: Chapter Personalization (005-chapter-personalization)
Task: T039
"""

import pytest
from unittest.mock import Mock, patch, AsyncMock
from personalization.agent import (
    protect_technical_elements,
    reconstruct_markdown,
    personalize_chapter_async,
)


class TestProtectTechnicalElements:
    """Tests for protect_technical_elements function"""

    def test_preserves_code_blocks(self):
        """Test that code blocks are replaced with placeholders"""
        markdown = """# Chapter

Some text.

```python
def hello():
    print("world")
```

More text.
"""

        protected, elements = protect_technical_elements(markdown)

        # Check placeholder exists (0-indexed)
        assert "{{CODE_0}}" in protected

        # Check original code not in protected version
        assert "def hello():" not in protected
        assert 'print("world")' not in protected

        # Check element extracted correctly
        assert "{{CODE_0}}" in elements
        assert "def hello():" in elements["{{CODE_0}}"]
        assert 'print("world")' in elements["{{CODE_0}}"]

    def test_preserves_multiple_code_blocks(self):
        """Test multiple code blocks get unique placeholders"""
        markdown = """
```python
code1()
```

Text.

```bash
code2
```
"""

        protected, elements = protect_technical_elements(markdown)

        # Check both placeholders exist
        assert "{{CODE_BLOCK_1}}" in protected
        assert "{{CODE_BLOCK_2}}" in protected

        # Check both code blocks extracted
        assert len(elements["code_blocks"]) == 2
        assert "code1()" in elements["code_blocks"][0]
        assert "code2" in elements["code_blocks"][1]

    def test_preserves_inline_math(self):
        """Test inline LaTeX math preserved"""
        markdown = "The formula $E = mc^2$ is famous."

        protected, elements = protect_technical_elements(markdown)

        # Check placeholder exists
        assert "{{MATH_INLINE_1}}" in protected

        # Check original formula not in protected version
        assert "$E = mc^2$" not in protected

        # Check element extracted
        assert len(elements["math_inline"]) == 1
        assert "$E = mc^2$" in elements["math_inline"][0]

    def test_preserves_display_math(self):
        """Test display LaTeX math preserved"""
        markdown = """
$$
\\int_0^\\infty e^{-x^2} dx = \\frac{\\sqrt{\\pi}}{2}
$$
"""

        protected, elements = protect_technical_elements(markdown)

        # Check placeholder exists
        assert "{{MATH_DISPLAY_1}}" in protected

        # Check original formula not in protected version
        assert "\\int_0^\\infty" not in protected

        # Check element extracted
        assert len(elements["math_display"]) == 1
        assert "\\int_0^\\infty" in elements["math_display"][0]

    def test_preserves_yaml_frontmatter(self):
        """Test YAML frontmatter preserved"""
        markdown = """---
title: My Chapter
id: chapter-1
tags: [robotics, ai]
---

# Chapter Content
"""

        protected, elements = protect_technical_elements(markdown)

        # Check placeholder exists
        assert "{{FRONTMATTER}}" in protected

        # Check original frontmatter not in protected version
        assert "title: My Chapter" not in protected
        assert "id: chapter-1" not in protected

        # Check element extracted
        assert elements["frontmatter"] is not None
        assert "title: My Chapter" in elements["frontmatter"]
        assert "tags: [robotics, ai]" in elements["frontmatter"]

    def test_preserves_images(self):
        """Test image markdown preserved"""
        markdown = "![Robot diagram](../images/robot.png)"

        protected, elements = protect_technical_elements(markdown)

        # Check placeholder exists
        assert "{{IMAGE_1}}" in protected

        # Check original image not in protected version
        assert "![Robot diagram]" not in protected

        # Check element extracted
        assert len(elements["images"]) == 1
        assert "![Robot diagram](../images/robot.png)" in elements["images"][0]

    def test_preserves_mixed_content(self):
        """Test mixed technical content all preserved"""
        markdown = """---
title: Test
---

# Chapter

Formula: $E = mc^2$

```python
code()
```

$$
math
$$

![image](img.png)
"""

        protected, elements = protect_technical_elements(markdown)

        # Check all placeholders exist
        assert "{{FRONTMATTER}}" in protected
        assert "{{MATH_INLINE_1}}" in protected
        assert "{{CODE_BLOCK_1}}" in protected
        assert "{{MATH_DISPLAY_1}}" in protected
        assert "{{IMAGE_1}}" in protected

        # Verify counts
        assert elements["frontmatter"] is not None
        assert len(elements["code_blocks"]) == 1
        assert len(elements["math_inline"]) == 1
        assert len(elements["math_display"]) == 1
        assert len(elements["images"]) == 1

    def test_empty_markdown(self):
        """Test empty markdown handled"""
        markdown = ""

        protected, elements = protect_technical_elements(markdown)

        assert protected == ""
        assert len(elements["code_blocks"]) == 0
        assert len(elements["math_inline"]) == 0


class TestReconstructMarkdown:
    """Tests for reconstruct_markdown function"""

    def test_restores_code_blocks(self):
        """Test code blocks restored from placeholders"""
        personalized = "Text {{CODE_BLOCK_1}} more text."
        elements = {
            "code_blocks": ["```python\ncode()\n```"],
            "math_inline": [],
            "math_display": [],
            "images": [],
            "frontmatter": None,
        }

        result = reconstruct_markdown(personalized, elements)

        assert "```python\ncode()\n```" in result
        assert "{{CODE_BLOCK_1}}" not in result

    def test_restores_all_elements(self):
        """Test all elements restored correctly"""
        personalized = """{{FRONTMATTER}}

{{MATH_INLINE_1}}

{{CODE_BLOCK_1}}

{{MATH_DISPLAY_1}}

{{IMAGE_1}}"""

        elements = {
            "frontmatter": "---\ntitle: Test\n---",
            "code_blocks": ["```python\ncode()\n```"],
            "math_inline": ["$E=mc^2$"],
            "math_display": ["$$\nmath\n$$"],
            "images": ["![img](img.png)"],
        }

        result = reconstruct_markdown(personalized, elements)

        # All elements should be restored
        assert "---\ntitle: Test\n---" in result
        assert "```python\ncode()\n```" in result
        assert "$E=mc^2$" in result
        assert "$$\nmath\n$$" in result
        assert "![img](img.png)" in result

        # No placeholders remaining
        assert "{{" not in result
        assert "}}" not in result

    def test_preserves_order(self):
        """Test elements restored in correct order"""
        personalized = "First {{CODE_BLOCK_1}} then {{CODE_BLOCK_2}} done."
        elements = {
            "code_blocks": ["```A```", "```B```"],
            "math_inline": [],
            "math_display": [],
            "images": [],
            "frontmatter": None,
        }

        result = reconstruct_markdown(personalized, elements)

        # Check order preserved
        a_pos = result.index("```A```")
        b_pos = result.index("```B```")
        assert a_pos < b_pos

    def test_handles_missing_placeholders(self):
        """Test handles case where placeholder not replaced"""
        personalized = "Text {{CODE_BLOCK_1}}"
        elements = {
            "code_blocks": [],  # No code blocks to restore
            "math_inline": [],
            "math_display": [],
            "images": [],
            "frontmatter": None,
        }

        # Should not crash, placeholder may remain
        result = reconstruct_markdown(personalized, elements)
        assert isinstance(result, str)


class TestPersonalizeChapterAsync:
    """Tests for personalize_chapter_async function"""

    @pytest.mark.asyncio
    async def test_preserves_code_blocks_after_personalization(self):
        """Test code blocks preserved through full personalization"""
        chapter_content = """# Chapter

```python
def important():
    return "must preserve"
```

Some prose to personalize.
"""

        user_profile = {
            "softwareBackground": "Beginner",
            "hardwareBackground": "None",
            "interestArea": "AI",
        }

        # Mock the OpenAI agent response
        with patch("personalization.agent.get_openai_agent") as mock_agent:
            mock_agent.return_value.run = AsyncMock(
                return_value="# Chapter\n\n{{CODE_BLOCK_1}}\n\nSimpler prose for beginners."
            )

            result = await personalize_chapter_async(
                chapter_content=chapter_content,
                user_profile=user_profile,
                timeout_seconds=30,
            )

            # Code block should be preserved exactly
            assert "```python" in result
            assert "def important():" in result
            assert 'return "must preserve"' in result

            # Prose should be changed
            assert "Simpler prose for beginners" in result

    @pytest.mark.asyncio
    async def test_timeout_enforcement(self):
        """Test timeout raises TimeoutError"""
        chapter_content = "# Test"
        user_profile = {
            "softwareBackground": "Intermediate",
            "hardwareBackground": "Beginner",
            "interestArea": "Robotics",
        }

        # Mock agent that takes too long
        with patch("personalization.agent.get_openai_agent") as mock_agent:
            async def slow_response(*args, **kwargs):
                import asyncio
                await asyncio.sleep(35)  # Longer than 30s timeout
                return "result"

            mock_agent.return_value.run = slow_response

            with pytest.raises(TimeoutError):
                await personalize_chapter_async(
                    chapter_content=chapter_content,
                    user_profile=user_profile,
                    timeout_seconds=30,
                )

    @pytest.mark.asyncio
    async def test_preserves_frontmatter(self):
        """Test YAML frontmatter preserved"""
        chapter_content = """---
title: Important Chapter
id: chapter-1
---

# Chapter
Content here.
"""

        user_profile = {
            "softwareBackground": "Expert",
            "hardwareBackground": "Advanced",
            "interestArea": "Computer Vision",
        }

        with patch("personalization.agent.get_openai_agent") as mock_agent:
            mock_agent.return_value.run = AsyncMock(
                return_value="{{FRONTMATTER}}\n\n# Chapter\nAdvanced content."
            )

            result = await personalize_chapter_async(
                chapter_content=chapter_content,
                user_profile=user_profile,
                timeout_seconds=30,
            )

            # Frontmatter should be preserved exactly
            assert "---" in result
            assert "title: Important Chapter" in result
            assert "id: chapter-1" in result

    @pytest.mark.asyncio
    async def test_preserves_latex_formulas(self):
        """Test LaTeX formulas preserved"""
        chapter_content = """
Inline: $E = mc^2$

Display:
$$
\\int_0^\\infty e^{-x^2} dx
$$
"""

        user_profile = {
            "softwareBackground": "Beginner",
            "hardwareBackground": "None",
            "interestArea": "AI",
        }

        with patch("personalization.agent.get_openai_agent") as mock_agent:
            mock_agent.return_value.run = AsyncMock(
                return_value="{{MATH_INLINE_1}}\n\n{{MATH_DISPLAY_1}}"
            )

            result = await personalize_chapter_async(
                chapter_content=chapter_content,
                user_profile=user_profile,
                timeout_seconds=30,
            )

            # Formulas should be preserved exactly
            assert "$E = mc^2$" in result
            assert "\\int_0^\\infty e^{-x^2}" in result


class TestEndToEndPreservation:
    """Integration tests for complete preservation workflow"""

    @pytest.mark.asyncio
    async def test_complex_chapter_preservation(self):
        """Test realistic chapter with mixed content"""
        chapter_content = """---
title: ROS 2 Nodes
id: foundations-ros2/nodes-topics
---

# ROS 2 Nodes and Topics

## Introduction

ROS 2 uses the DDS protocol with QoS settings: $QoS = (reliability, durability)$.

## Code Example

```python
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher')
    publisher = node.create_publisher(String, 'topic', 10)
```

## Mathematical Model

The message flow rate is given by:

$$
rate = \\frac{messages}{second}
$$

![ROS 2 Architecture](../images/ros2-arch.png)
"""

        user_profile = {
            "softwareBackground": "Intermediate",
            "hardwareBackground": "Beginner",
            "interestArea": "Robotics",
        }

        with patch("personalization.agent.get_openai_agent") as mock_agent:
            # Mock returns placeholders in correct positions
            mock_agent.return_value.run = AsyncMock(
                return_value="""{{FRONTMATTER}}

# ROS 2 Nodes and Topics

## Introduction (Simplified for You)

ROS 2 uses messaging with quality settings: {{MATH_INLINE_1}}.

## Code Example

{{CODE_BLOCK_1}}

## Mathematical Model

The rate formula:

{{MATH_DISPLAY_1}}

{{IMAGE_1}}
"""
            )

            result = await personalize_chapter_async(
                chapter_content=chapter_content,
                user_profile=user_profile,
                timeout_seconds=30,
            )

            # Verify ALL technical elements preserved
            assert "title: ROS 2 Nodes" in result
            assert "id: foundations-ros2/nodes-topics" in result
            assert "import rclpy" in result
            assert "from std_msgs.msg import String" in result
            assert "$QoS = (reliability, durability)$" in result
            assert "$$\nrate = \\frac{messages}{second}\n$$" in result
            assert "![ROS 2 Architecture](../images/ros2-arch.png)" in result

            # Verify prose was personalized
            assert "Simplified for You" in result or "Introduction" in result


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
