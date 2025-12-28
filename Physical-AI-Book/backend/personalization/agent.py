"""
OpenAI Personalization Agent

Implements AI-powered chapter personalization using OpenAI API.
Preserves technical content (code blocks, LaTeX formulas, diagrams) using AST-based markdown processing.
"""

from typing import Dict, Tuple, Optional
import os
import re
from markdown_it import MarkdownIt
from markdown_it.token import Token
from openai import OpenAI
import asyncio
import logging

logger = logging.getLogger(__name__)

# Initialize OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Configuration
PERSONALIZATION_MODEL = os.getenv("PERSONALIZATION_MODEL", "gpt-4o-mini")
PERSONALIZATION_TIMEOUT_SECONDS = int(os.getenv("PERSONALIZATION_TIMEOUT_SECONDS", "30"))
MAX_PERSONALIZATION_TOKENS = int(os.getenv("MAX_PERSONALIZATION_TOKENS", "16384"))


def protect_technical_elements(markdown_content: str) -> Tuple[str, Dict[str, str]]:
    """
    Extract protected elements (code, math, frontmatter) and replace with placeholders.

    Uses markdown-it-py for AST-based parsing to ensure 100% preservation of technical content.

    Args:
        markdown_content: Original chapter markdown

    Returns:
        Tuple of (prose-only markdown, protection_map)
    """
    md = MarkdownIt('commonmark', {'html': True})
    md.enable(['table', 'strikethrough'])

    protection_map = {}
    placeholder_counter = 0
    result_lines = []

    try:
        # Split content into lines for processing
        lines = markdown_content.split('\n')

        # Detect and protect YAML frontmatter
        if lines and lines[0].strip() == '---':
            frontmatter_end = None
            for i in range(1, len(lines)):
                if lines[i].strip() == '---':
                    frontmatter_end = i
                    break

            if frontmatter_end:
                # Protect frontmatter
                frontmatter = '\n'.join(lines[:frontmatter_end + 1])
                placeholder = f"{{{{FRONTMATTER_{placeholder_counter}}}}}"
                protection_map[placeholder] = frontmatter
                result_lines.append(placeholder)
                placeholder_counter += 1
                lines = lines[frontmatter_end + 1:]

        # Process remaining content
        content_without_frontmatter = '\n'.join(lines)

        # Protect code blocks (fenced with ``` or ```)
        code_block_pattern = re.compile(r'```[\s\S]*?```|`[^`\n]+`', re.MULTILINE)
        prose_content = content_without_frontmatter

        for match in code_block_pattern.finditer(content_without_frontmatter):
            placeholder = f"{{{{CODE_{placeholder_counter}}}}}"
            protection_map[placeholder] = match.group(0)
            prose_content = prose_content.replace(match.group(0), placeholder, 1)
            placeholder_counter += 1

        # Protect LaTeX math blocks ($$...$$) and inline ($...$)
        math_block_pattern = re.compile(r'\$\$[\s\S]*?\$\$|\$[^\$\n]+\$', re.MULTILINE)

        # Need to process in order, replacing as we go
        temp_content = prose_content
        for match in math_block_pattern.finditer(prose_content):
            placeholder = f"{{{{MATH_{placeholder_counter}}}}}"
            protection_map[placeholder] = match.group(0)
            temp_content = temp_content.replace(match.group(0), placeholder, 1)
            placeholder_counter += 1

        prose_content = temp_content

        # Protect images
        image_pattern = re.compile(r'!\[([^\]]*)\]\(([^\)]+)\)')

        temp_content = prose_content
        for match in image_pattern.finditer(prose_content):
            placeholder = f"{{{{IMAGE_{placeholder_counter}}}}}"
            protection_map[placeholder] = match.group(0)
            temp_content = temp_content.replace(match.group(0), placeholder, 1)
            placeholder_counter += 1

        prose_content = temp_content

        # Combine frontmatter placeholder (if exists) with prose content
        if result_lines:
            final_content = '\n'.join(result_lines) + '\n\n' + prose_content
        else:
            final_content = prose_content

        logger.info(f"Protected {len(protection_map)} technical elements")
        return final_content, protection_map

    except Exception as e:
        logger.error(f"Error protecting technical elements: {e}")
        # Fallback: return original content with empty protection map
        return markdown_content, {}


def reconstruct_markdown(personalized_prose: str, protection_map: Dict[str, str]) -> str:
    """
    Replace placeholders with original protected elements.

    Args:
        personalized_prose: AI-generated personalized content with placeholders
        protection_map: Dictionary mapping placeholders to original content

    Returns:
        Reconstructed markdown with preserved technical elements
    """
    try:
        result = personalized_prose

        # Replace placeholders with original content
        # Process in reverse order to handle nested placeholders correctly
        for placeholder, original_content in sorted(protection_map.items(), reverse=True):
            result = result.replace(placeholder, original_content)

        logger.info(f"Reconstructed markdown with {len(protection_map)} protected elements")
        return result

    except Exception as e:
        logger.error(f"Error reconstructing markdown: {e}")
        # Fallback: return personalized prose as-is
        return personalized_prose


def personalize_chapter(
    chapter_content: str,
    user_profile: Dict[str, str],
    timeout_seconds: Optional[int] = None
) -> str:
    """
    Generate personalized chapter content using OpenAI API.

    Uses protection/reconstruction pattern to preserve technical content.

    Args:
        chapter_content: Original chapter markdown
        user_profile: User's background (softwareBackground, hardwareBackground, interestArea)
        timeout_seconds: Optional timeout override (default from env)

    Returns:
        Personalized chapter markdown

    Raises:
        TimeoutError: If personalization exceeds timeout
        Exception: On OpenAI API errors
    """
    timeout = timeout_seconds or PERSONALIZATION_TIMEOUT_SECONDS

    try:
        # Step 1: Protect technical elements
        logger.info("Protecting technical elements...")
        logger.info(f"Original content length: {len(chapter_content)} chars")
        prose_only, protection_map = protect_technical_elements(chapter_content)
        logger.info(f"Prose-only content length: {len(prose_only)} chars")
        logger.info(f"Protected elements: {len(protection_map)}")

        # Debug: Log first 200 chars of prose_only
        logger.info(f"Prose preview: {prose_only[:200]}...")

        # Step 2: Build personalization prompt
        software_bg = user_profile.get('softwareBackground', 'Intermediate')
        hardware_bg = user_profile.get('hardwareBackground', 'Beginner')
        interest_area = user_profile.get('interestArea', 'General')

        system_prompt = f"""You are a technical writing assistant that adapts explanation complexity for different skill levels.

User Profile:
- Software Background: {software_bg}
- Hardware Background: {hardware_bg}
- Interest Area: {interest_area}

YOUR TASK (TWO STEPS):

STEP 1: READ AND UNDERSTAND
Read the chapter content carefully. Identify:
- What is the main topic? (e.g., robotics, simulation, ROS2, vision, etc.)
- What are the chapter title and section headings?
- What are the key technical concepts being taught?

STEP 2: PERSONALIZE EXPLANATION STYLE ONLY
Rewrite the chapter for a {software_bg} software background and {hardware_bg} hardware background user.

CRITICAL RULES:
✓ KEEP: Same topic, same title, same headings, same concepts, same technical terms
✓ KEEP: All placeholders like {{{{CODE_X}}}}, {{{{MATH_X}}}}, {{{{IMAGE_X}}}}, {{{{FRONTMATTER_X}}}} unchanged
✓ CHANGE: Only HOW concepts are explained (simpler/deeper based on user level)

WHAT TO ADJUST FOR {software_bg} LEVEL:
- Beginner: Use simple language, add brief definitions, use analogies
- Intermediate: Normal technical writing, assume basic knowledge
- Advanced: Technical depth, assume strong foundation
- Expert: Concise, advanced terminology, assume expertise

WHAT NEVER TO CHANGE:
- The main subject/topic of the chapter
- Chapter titles or section headings
- Technical terms, concepts, or examples mentioned in the original
- The structure, organization, or flow
- Any placeholder text

Remember: You are adjusting explanation DEPTH, not changing CONTENT."""

        user_message = f"""Here is the chapter content to personalize:

{prose_only}

INSTRUCTIONS:
1. First, identify what this chapter is about (the main topic)
2. Then rewrite it for a {software_bg} software background user
3. Keep the SAME topic, SAME titles, SAME headings, SAME concepts
4. Only adjust explanation complexity and style

Output the complete personalized chapter with all placeholders intact."""

        # Step 3: Call OpenAI API with timeout
        logger.info(f"Calling OpenAI API (model: {PERSONALIZATION_MODEL}, timeout: {timeout}s)...")
        logger.info(f"User message length: {len(user_message)} chars")
        logger.info(f"User message preview: {user_message[:300]}...")

        response = client.chat.completions.create(
            model=PERSONALIZATION_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            max_tokens=MAX_PERSONALIZATION_TOKENS,
            temperature=0.7,
            timeout=timeout
        )

        personalized_prose = response.choices[0].message.content

        if not personalized_prose:
            raise Exception("OpenAI API returned empty response")

        logger.info(f"Received personalized content ({len(personalized_prose)} chars)")

        # Step 4: Reconstruct with protected elements
        logger.info("Reconstructing markdown with protected elements...")
        final_content = reconstruct_markdown(personalized_prose, protection_map)

        logger.info("Personalization complete")
        return final_content

    except asyncio.TimeoutError:
        logger.error(f"Personalization timed out after {timeout} seconds")
        raise TimeoutError(f"Personalization exceeded {timeout} second timeout")

    except Exception as e:
        logger.error(f"Personalization failed: {e}")
        raise


async def personalize_chapter_async(
    chapter_content: str,
    user_profile: Dict[str, str],
    timeout_seconds: Optional[int] = None
) -> str:
    """
    Async wrapper for personalize_chapter with enforced timeout.

    Args:
        chapter_content: Original chapter markdown
        user_profile: User's background
        timeout_seconds: Optional timeout override

    Returns:
        Personalized chapter markdown

    Raises:
        TimeoutError: If personalization exceeds timeout
    """
    timeout = timeout_seconds or PERSONALIZATION_TIMEOUT_SECONDS

    try:
        # Run in thread pool to avoid blocking event loop
        import concurrent.futures

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as pool:
            result = await asyncio.wait_for(
                loop.run_in_executor(
                    pool,
                    personalize_chapter,
                    chapter_content,
                    user_profile,
                    timeout
                ),
                timeout=timeout
            )
            return result

    except asyncio.TimeoutError:
        logger.error(f"Async personalization timed out after {timeout} seconds")
        raise TimeoutError(f"Personalization exceeded {timeout} second timeout")
