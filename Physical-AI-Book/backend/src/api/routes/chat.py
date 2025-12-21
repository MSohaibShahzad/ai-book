"""
Chat endpoint for RAG-based question answering.
"""
from fastapi import APIRouter, HTTPException, status, Request, Response
from fastapi.responses import StreamingResponse
from typing import Optional
import logging
import json

from src.api.models.request import ChatRequest
from src.middleware.rate_limit import limiter
from src.api.models.response import ChatResponse, SourceReference
from src.services.rag_pipeline import rag_pipeline
from src.middleware.jwt_auth import get_current_user, get_user_context_for_llm

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/chat", response_model=ChatResponse, status_code=status.HTTP_200_OK)
@limiter.limit("10/minute")
async def chat(request: Request, response: Response, chat_request: ChatRequest) -> ChatResponse:
    """
    Process a chat message and return AI response with source references.

    Args:
        request: FastAPI Request object for rate limiting
        chat_request: ChatRequest with message, optional highlighted_text, session_id, and conversation_history

    Returns:
        ChatResponse with answer, sources, and metadata

    Raises:
        HTTPException: If processing fails
    """
    try:
        # Validate request
        if not chat_request.message or not chat_request.message.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message cannot be empty"
            )

        # Get user context for personalization
        user = get_current_user(request)
        user_context = get_user_context_for_llm(user)

        if user:
            logger.info(f"Chat request from authenticated user: {user.name} ({user.email})")
            logger.info(f"User background: SW={user.software_background}, HW={user.hardware_background}, Interest={user.interest_area}")

        # Process query through RAG pipeline with user context
        response_text, sources, processing_time = await rag_pipeline.process_query(
            query=chat_request.message,
            highlighted_text=chat_request.highlighted_text,
            conversation_history=chat_request.conversation_history,
            filters=None,  # Could be extended for module/chapter filtering
            user_context=user_context  # Pass user context for personalization
        )

        # Convert sources to SourceReference models
        source_refs = [
            SourceReference(
                module_name=src["module_name"],
                chapter_name=src["chapter_name"],
                slug=src["slug"],
                chunk_text_preview=src["preview"]
            )
            for src in sources
        ]

        # Build response
        chat_response = ChatResponse(
            response=response_text,
            sources=source_refs,
            session_id=chat_request.session_id,
            retrieval_count=len(source_refs),
            processing_time_ms=processing_time
        )

        logger.info(
            f"Chat request processed: session={chat_request.session_id}, "
            f"sources={len(source_refs)}, time={processing_time}ms"
        )

        return chat_response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        logger.error(f"Chat endpoint error: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process chat request: {str(e)}"
        )


@router.post("/chat/stream", status_code=status.HTTP_200_OK)
@limiter.limit("10/minute")
async def chat_stream(request: Request, response: Response, chat_request: ChatRequest):
    """
    Process a chat message and return streaming AI response with Server-Sent Events (SSE).

    Args:
        request: FastAPI Request object for rate limiting
        chat_request: ChatRequest with message, optional highlighted_text, session_id, and conversation_history

    Returns:
        StreamingResponse with text/event-stream content type

    Raises:
        HTTPException: If processing fails
    """
    try:
        # Validate request
        if not chat_request.message or not chat_request.message.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message cannot be empty"
            )

        async def event_stream():
            """Generator for Server-Sent Events."""
            try:
                # Stream response from RAG pipeline
                async for chunk in rag_pipeline.process_query_stream(
                    query=chat_request.message,
                    highlighted_text=chat_request.highlighted_text,
                    conversation_history=chat_request.conversation_history
                ):
                    # Send each chunk as SSE data event
                    yield f"data: {json.dumps({'chunk': chunk})}\n\n"

                # Send completion event
                yield f"data: {json.dumps({'done': True})}\n\n"

            except Exception as e:
                logger.error(f"Streaming error: {str(e)}", exc_info=True)
                # Send error event
                yield f"data: {json.dumps({'error': str(e)})}\n\n"

        return StreamingResponse(
            event_stream(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"  # Disable nginx buffering
            }
        )

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        logger.error(f"Chat stream endpoint error: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process streaming chat request: {str(e)}"
        )
