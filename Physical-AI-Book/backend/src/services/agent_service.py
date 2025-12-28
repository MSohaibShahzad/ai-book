"""
Agent service using OpenAI Agents SDK for RAG Chatbot.
Replaces the direct LLM service with an agent-based approach.
"""
from typing import List, Dict, Optional, Annotated
from pydantic import BaseModel, Field
from agents import Agent, Runner, function_tool, RunContextWrapper, ModelSettings
from uuid import UUID

from src.services.embeddings import embedding_service
from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service
from src.config import settings


class RetrievedContext(BaseModel):
    """Retrieved context from RAG pipeline."""
    chunks: List[Dict] = Field(description="Retrieved text chunks with metadata")
    query: str = Field(description="The original user query")


class RAGAgentService:
    """Service for RAG using OpenAI Agents SDK."""

    SIMILARITY_THRESHOLD = 0.35  # Minimum cosine similarity for relevant results

    def __init__(self):
        """Initialize agent service with RAG retrieval tool."""
        self.embeddings = embedding_service
        self.qdrant = qdrant_service
        self.postgres = postgres_service
        self.max_chunks = settings.max_retrieval_chunks

        # Create the retrieval tool
        @function_tool
        async def retrieve_textbook_context(
            ctx: RunContextWrapper,
            query: Annotated[str, "The student's question to find relevant textbook content for"]
        ) -> str:
            """
            Retrieve relevant context from the Physical-AI textbook based on the student's question.
            This tool searches the textbook and returns the most relevant excerpts.
            """
            # Step 1: Generate embedding for query
            try:
                query_embedding = self.embeddings.generate_embedding(query)
            except Exception as e:
                return f"Error generating embedding: {str(e)}"

            # Step 2: Search Qdrant for similar vectors
            try:
                search_results = self.qdrant.search(
                    query_vector=query_embedding,
                    limit=self.max_chunks,
                    score_threshold=self.SIMILARITY_THRESHOLD
                )
            except Exception as e:
                return f"Error searching vector database: {str(e)}"

            # Step 3: Check if results are relevant
            if not search_results or all(result.score < self.SIMILARITY_THRESHOLD for result in search_results):
                return "NO_RELEVANT_CONTENT_FOUND"

            # Step 4: Retrieve metadata from Postgres
            vector_ids = [UUID(str(result.id)) for result in search_results]
            try:
                chunks_metadata = await self.postgres.get_chunks_by_vector_ids(vector_ids)
            except Exception as e:
                return f"Error retrieving metadata: {str(e)}"

            # Step 5: Merge results
            enriched_chunks = self._merge_results(search_results, chunks_metadata)

            if not enriched_chunks:
                return "NO_RELEVANT_CONTENT_FOUND"

            # Step 6: Format context for agent
            formatted_context = self._format_context(enriched_chunks)
            return formatted_context

        # Store retrieval tool
        self.retrieve_tool = retrieve_textbook_context

        # Create the RAG agent with API key
        import os
        os.environ["OPENAI_API_KEY"] = settings.openai_api_key

        self.agent = Agent(
            name="Physical-AI Tutor",
            instructions=self._get_system_instructions(),
            model=settings.llm_model,
            tools=[retrieve_textbook_context],
            model_settings=ModelSettings(tool_choice="retrieve_textbook_context"),
        )

    def _get_system_instructions(self) -> str:
        """Get agent instructions (system prompt)."""
        return """You are an expert AI tutor for a Physical-AI and Robotics textbook. Your role is to help students learn robotics concepts, theory, and practical applications.

**Key Principles:**

1. **Educational Focus**: Explain concepts clearly with appropriate technical depth. Break down complex topics into digestible explanations.

2. **Source Attribution**: Always cite the specific textbook module and chapter where information comes from. Use format: "Source: Module X, Chapter Y"

3. **Factual Accuracy**:
   - ALWAYS use the retrieve_textbook_context tool FIRST to get relevant textbook content
   - Only answer based on the retrieved context from the textbook
   - If the tool returns "NO_RELEVANT_CONTENT_FOUND", clearly state: "This topic is not covered in the sections of the textbook I have access to."

4. **Safety & Ethics**:
   - Refuse harmful or unethical queries (e.g., weaponizing robots, bypassing safety systems)
   - Redirect to "Ethics & Safety" module when appropriate
   - Emphasize responsible robotics development

5. **Practical Application**: When relevant, connect theory to practical implementation, real-world examples, or industry standards.

**Response Format:**
- Provide clear, concise explanations (aim for 2-5 sentences for simple questions, up to 2-3 paragraphs for complex topics)
- Include specific citations to sources
- Use technical terminology accurately
- Format mathematical equations clearly when needed

**CRITICAL**: You MUST call retrieve_textbook_context tool before answering any question. Never answer from your general knowledge without retrieving textbook content first."""

    def _merge_results(
        self,
        vector_results: List,
        metadata_results: List[Dict]
    ) -> List[Dict]:
        """Merge Qdrant search results with Postgres metadata."""
        # Create lookup map for metadata
        metadata_map = {
            str(chunk["qdrant_vector_id"]): chunk
            for chunk in metadata_results
        }

        enriched = []
        for result in vector_results:
            vector_id = str(result.id)
            metadata = metadata_map.get(vector_id)

            if metadata:
                enriched.append({
                    "vector_id": vector_id,
                    "score": result.score,
                    "chunk_text": metadata["chunk_text"],
                    "module_name": metadata["module_name"],
                    "chapter_name": metadata["chapter_name"],
                    "section_heading": metadata.get("section_heading"),
                    "file_path": metadata["file_path"],
                    "slug": metadata["slug"],
                    "chunk_index": metadata["chunk_index"],
                })

        return enriched

    def _format_context(self, chunks: List[Dict]) -> str:
        """Format context chunks for agent prompt."""
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

    async def generate_response(
        self,
        query: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
        user_context: Optional[str] = None
    ) -> str:
        """
        Generate agent response for query.

        Args:
            query: User's question
            conversation_history: Optional previous messages [{role, content}]
            user_context: Optional user background context for personalization

        Returns:
            Generated response string
        """
        # Safety check
        safety_message = self._check_safety(query)
        if safety_message:
            return safety_message

        # Prepend user context to query if provided
        final_query = query
        if user_context:
            final_query = f"{user_context}\n\nStudent Question: {query}"

        # Run agent with query
        try:
            result = await Runner.run(
                self.agent,
                input=final_query
            )
            return result.final_output

        except Exception as e:
            raise Exception(f"Agent execution failed: {str(e)}")

    async def generate_response_stream(
        self,
        query: str,
        conversation_history: Optional[List[Dict[str, str]]] = None
    ):
        """
        Generate streaming agent response for query.

        Args:
            query: User's question
            conversation_history: Optional previous messages [{role, content}]

        Yields:
            Text deltas as they arrive from the agent
        """
        # Safety check
        safety_message = self._check_safety(query)
        if safety_message:
            yield safety_message
            return

        # Run agent with streaming
        try:
            from openai.types.responses import ResponseTextDeltaEvent

            result = Runner.run_streamed(
                self.agent,
                input=query
            )

            async for event in result.stream_events():
                # Stream text deltas from LLM response
                if event.type == "raw_response_event" and isinstance(event.data, ResponseTextDeltaEvent):
                    yield event.data.delta

        except Exception as e:
            raise Exception(f"Agent streaming failed: {str(e)}")


# Global service instance
agent_service = RAGAgentService()
