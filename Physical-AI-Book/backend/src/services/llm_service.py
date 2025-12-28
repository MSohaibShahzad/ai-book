"""
LLM service for RAG Chatbot using OpenAI GPT-4o-mini.
Handles prompt generation, LLM calls, and response parsing with safety checks.
"""
import time
from typing import List, Dict, Optional
from openai import OpenAI
import openai

from src.config import settings


class LLMService:
    """Service for generating responses using OpenAI LLM."""

    def __init__(self):
        """Initialize OpenAI client."""
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.llm_model
        self.temperature = settings.llm_temperature
        self.max_tokens = settings.llm_max_tokens

    def _get_system_prompt(self) -> str:
        """
        Get system prompt with constitution principles.
        Incorporates principles VII-X from constitution.
        """
        return """You are an expert AI tutor for a Physical-AI and Robotics textbook. Your role is to help students learn robotics concepts, theory, and practical applications.

**Key Principles:**

1. **Educational Focus**: Explain concepts clearly with appropriate technical depth. Break down complex topics into digestible explanations.

2. **Source Attribution**: Always cite the specific textbook module and chapter where information comes from. Use format: "Source: Module X, Chapter Y"

3. **Factual Accuracy**: Only answer based on the provided context from the textbook. If information isn't in the context, clearly state: "This topic is not covered in the sections I have access to."

4. **Safety & Ethics**:
   - Refuse harmful or unethical queries (e.g., weaponizing robots, bypassing safety systems)
   - Redirect to "Ethics & Safety" module when appropriate
   - Emphasize responsible robotics development

5. **Practical Application**: When relevant, connect theory to practical implementation, real-world examples, or industry standards.

**Response Format:**
- Provide clear, concise explanations (aim for 2-5 sentences for simple questions, up to 2-3 paragraphs for complex topics)
- Include specific citations to sources
- Use technical terminology accurately
- Format mathematical equations clearly when needed"""

    def _check_safety(self, query: str) -> Optional[str]:
        """
        Check if query contains harmful intent.
        Returns safety message if query should be blocked, None otherwise.
        """
        harmful_keywords = [
            "weaponize", "weapon", "harm", "attack", "exploit",
            "bypass safety", "disable safety", "malicious",
            "injure", "damage", "destroy"
        ]

        query_lower = query.lower()
        if any(keyword in query_lower for keyword in harmful_keywords):
            return (
                "I cannot provide assistance with potentially harmful or unethical applications of robotics. "
                "For discussions on responsible robotics development, safety systems, and ethical considerations, "
                "please refer to the Ethics & Safety module in the textbook."
            )

        return None

    def generate_response(
        self,
        query: str,
        context_chunks: List[Dict[str, any]],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> str:
        """
        Generate LLM response for query using provided context.

        Args:
            query: User's question
            context_chunks: List of relevant chunks with metadata
            conversation_history: Optional previous messages [{role, content}]

        Returns:
            Generated response string
        """
        # Safety check
        safety_message = self._check_safety(query)
        if safety_message:
            return safety_message

        # Build context from chunks
        context_text = self._format_context(context_chunks)

        # Build user prompt
        user_prompt = f"""Based on the following excerpts from the Physical-AI textbook, answer the student's question.

**Textbook Context:**
{context_text}

**Student Question:**
{query}

Please provide a clear, educational response with proper source citations."""

        # Build messages
        messages = [{"role": "system", "content": self._get_system_prompt()}]

        # Add conversation history if provided (limit to last 3 exchanges)
        if conversation_history:
            messages.extend(conversation_history[-6:])  # Last 3 exchanges = 6 messages

        messages.append({"role": "user", "content": user_prompt})

        # Call OpenAI with exponential backoff
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=self.temperature,
                    max_tokens=self.max_tokens,
                    top_p=0.9,
                )

                return response.choices[0].message.content.strip()

            except openai.RateLimitError:
                if attempt < max_retries - 1:
                    wait_time = 2 ** attempt  # Exponential backoff: 1s, 2s, 4s
                    time.sleep(wait_time)
                    continue
                raise

            except openai.APIError as e:
                if attempt < max_retries - 1:
                    wait_time = 2 ** attempt
                    time.sleep(wait_time)
                    continue
                raise Exception(f"OpenAI API error after {max_retries} retries: {str(e)}")

            except Exception as e:
                raise Exception(f"LLM generation error: {str(e)}")

        raise Exception("Failed to generate response after retries")

    def _format_context(self, chunks: List[Dict[str, any]]) -> str:
        """
        Format context chunks for LLM prompt.

        Args:
            chunks: List of chunk dictionaries with metadata

        Returns:
            Formatted context string
        """
        formatted_chunks = []

        for i, chunk in enumerate(chunks, 1):
            module = chunk.get("module_name", "Unknown")
            chapter = chunk.get("chapter_name", "Unknown")
            section = chunk.get("section_heading", "")
            text = chunk.get("chunk_text", "")

            section_info = f" - {section}" if section else ""

            formatted = f"""[Excerpt {i}] Module: {module}, Chapter: {chapter}{section_info}
{text}
"""
            formatted_chunks.append(formatted)

        return "\n".join(formatted_chunks)


# Global service instance
llm_service = LLMService()
