"""
OpenAI Agents SDK service for Urdu translation.

Feature: 004-urdu-translation
Implements: GPT-4o agent with placeholder substitution for technical terms
"""

import os
import re
from typing import Dict, Tuple
from tenacity import retry, stop_after_attempt, wait_random_exponential
import tiktoken


class OpenAIAgentService:
    """
    Service for translating content to Urdu using OpenAI Agents SDK.

    Implements placeholder substitution pattern to preserve:
    - Code blocks
    - LaTeX formulas
    - Technical terms
    """

    def __init__(self):
        """Initialize OpenAI agent with configuration from environment."""
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        self.model = os.getenv("TRANSLATION_MODEL", "gpt-4o")
        self.max_tokens = int(os.getenv("MAX_TRANSLATION_TOKENS", "128000"))

        # System prompt for Urdu translation agent
        self.system_prompt = """You are a technical translation agent specializing in translating English educational content to Urdu (اردو).

Your task:
1. Translate all explanatory text from English to Urdu
2. Keep code, formulas, and placeholders (like __CODE_*, __LATEX_*, __TERM_*) EXACTLY as they appear
3. Use simple, clear Urdu appropriate for undergraduate students
4. Preserve markdown formatting (headings, lists, bold, italic, links)

Critical rules:
- NEVER translate content inside placeholders (__CODE_*, __LATEX_*, __TERM_*)
- NEVER modify code blocks or mathematical formulas
- Keep technical terms in English (robotics, neural network, actuator, etc.)
- Use simple Urdu sentences (avoid complex literary language)
- Maintain the same paragraph structure as the original

Example:
Input: "ROS 2 provides a __TERM_middleware__ for robot communication."
Output: "ROS 2 روبوٹ مواصلات کے لیے __TERM_middleware__ فراہم کرتا ہے۔"
"""

        # Placeholder patterns
        self.code_pattern = re.compile(r'```[\s\S]*?```', re.MULTILINE)
        self.latex_block_pattern = re.compile(r'\$\$[\s\S]*?\$\$', re.MULTILINE)
        self.latex_inline_pattern = re.compile(r'\$[^\$\n]+\$')
        self.technical_terms = [
            "ROS", "ROS 2", "API", "GPU", "CPU", "Docker", "Kubernetes",
            "neural network", "kinematics", "actuator", "sensor", "URDF",
            "middleware", "framework", "algorithm", "pipeline", "workflow"
        ]

    def extract_preservables(self, text: str) -> Tuple[str, Dict[str, str]]:
        """
        Extract code blocks, LaTeX, and technical terms for preservation.

        Args:
            text: Original markdown text

        Returns:
            Tuple of (cleaned_text_with_placeholders, placeholder_map)
        """
        placeholders = {}
        counter = 0

        # Extract code blocks
        def replace_code(match):
            nonlocal counter
            placeholder = f"__CODE_{counter:04d}__"
            placeholders[placeholder] = match.group(0)
            counter += 1
            return placeholder

        text = self.code_pattern.sub(replace_code, text)

        # Extract LaTeX blocks ($$...$$)
        def replace_latex_block(match):
            nonlocal counter
            placeholder = f"__LATEX_BLOCK_{counter:04d}__"
            placeholders[placeholder] = match.group(0)
            counter += 1
            return placeholder

        text = self.latex_block_pattern.sub(replace_latex_block, text)

        # Extract inline LaTeX ($...$)
        def replace_latex_inline(match):
            nonlocal counter
            placeholder = f"__LATEX_INLINE_{counter:04d}__"
            placeholders[placeholder] = match.group(0)
            counter += 1
            return placeholder

        text = self.latex_inline_pattern.sub(replace_latex_inline, text)

        # Extract technical terms (case-insensitive)
        for term in self.technical_terms:
            def replace_term(match):
                nonlocal counter
                placeholder = f"__TERM_{term.replace(' ', '_').upper()}__"
                placeholders[placeholder] = match.group(0)
                return placeholder

            text = re.sub(
                re.escape(term),
                replace_term,
                text,
                flags=re.IGNORECASE
            )

        return text, placeholders

    def restore_preservables(self, text: str, placeholders: Dict[str, str]) -> str:
        """
        Restore original content from placeholders.

        Args:
            text: Translated text with placeholders
            placeholders: Map of placeholder -> original content

        Returns:
            Text with placeholders replaced by original content
        """
        for placeholder, original in placeholders.items():
            text = text.replace(placeholder, original)
        return text

    @retry(
        wait=wait_random_exponential(min=1, max=60),
        stop=stop_after_attempt(6)
    )
    async def translate_to_urdu(self, text: str) -> str:
        """
        Translate English text to Urdu with retry logic.

        Args:
            text: English markdown content

        Returns:
            Urdu markdown content

        Raises:
            Exception: If translation fails after retries
        """
        # For MVP, we'll use OpenAI's standard completion API
        # In production, this would use the OpenAI Agents SDK
        # Note: Agents SDK integration will be completed in subsequent tasks

        # Step 1: Extract preservables
        cleaned_text, placeholders = self.extract_preservables(text)

        # Step 2: Translate (placeholder for actual OpenAI Agents SDK call)
        # TODO: Replace with actual Agents SDK implementation
        try:
            from openai import OpenAI
            client = OpenAI(api_key=self.api_key)

            response = client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": cleaned_text}
                ],
                max_tokens=self.max_tokens,
                temperature=0.3  # Lower temperature for consistent technical translation
            )

            translated_text = response.choices[0].message.content

            # Step 3: Restore preservables
            final_text = self.restore_preservables(translated_text, placeholders)

            # Log cost estimation and token usage
            input_tokens = response.usage.prompt_tokens
            output_tokens = response.usage.completion_tokens
            cost = self.estimate_cost(input_tokens, output_tokens)
            print(f"[COST] {self.model}: {input_tokens} in + {output_tokens} out = ${cost:.4f}")

            # Store cost tracking in a simple log file for admin monitoring
            try:
                import os
                log_dir = os.path.join(os.path.dirname(__file__), '../../logs')
                os.makedirs(log_dir, exist_ok=True)
                log_file = os.path.join(log_dir, 'translation_costs.log')

                with open(log_file, 'a') as f:
                    from datetime import datetime
                    timestamp = datetime.now().isoformat()
                    f.write(f"{timestamp},{self.model},{input_tokens},{output_tokens},{cost:.4f}\n")
            except Exception as e:
                print(f"[WARNING] Failed to log cost: {e}")

            return final_text

        except Exception as e:
            print(f"[ERROR] Translation failed: {e}")
            raise

    def estimate_cost(self, input_tokens: int, output_tokens: int) -> float:
        """
        Estimate cost of translation based on token usage.

        GPT-4o pricing (as of Dec 2024):
        - Input: $2.50 per 1M tokens
        - Output: $10.00 per 1M tokens

        Args:
            input_tokens: Number of input tokens
            output_tokens: Number of output tokens

        Returns:
            Estimated cost in USD
        """
        if self.model == "gpt-4o":
            input_cost = (input_tokens / 1_000_000) * 2.50
            output_cost = (output_tokens / 1_000_000) * 10.00
        elif self.model == "gpt-4o-mini":
            input_cost = (input_tokens / 1_000_000) * 0.150
            output_cost = (output_tokens / 1_000_000) * 0.600
        else:
            # Default to GPT-4o pricing
            input_cost = (input_tokens / 1_000_000) * 2.50
            output_cost = (output_tokens / 1_000_000) * 10.00

        return input_cost + output_cost

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in text using tiktoken.

        Args:
            text: Text to count tokens for

        Returns:
            Number of tokens
        """
        try:
            encoding = tiktoken.encoding_for_model(self.model)
            return len(encoding.encode(text))
        except Exception:
            # Fallback: rough estimate (1 token ≈ 4 characters)
            return len(text) // 4
