"""
Pydantic models for conversation sessions and messages.
"""

from datetime import datetime
from typing import List, Optional
from uuid import UUID
from pydantic import BaseModel
from enum import Enum


class MessageRole(str, Enum):
    """Role of message sender."""
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"


class Message(BaseModel):
    """Represents a single message in a conversation."""
    message_id: UUID
    session_id: UUID
    role: MessageRole
    content: str
    timestamp: datetime
    source_citations: Optional[List[dict]] = None


class ConversationSession(BaseModel):
    """Represents a chat session with message history."""
    session_id: UUID
    created_at: datetime
    last_activity: datetime
    messages: List[Message] = []
