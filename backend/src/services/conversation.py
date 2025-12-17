"""
Conversation service for managing chat sessions and message history.
Note: Database schema (T010) is deferred, so this uses in-memory storage for MVP.
Production version will use Neon Postgres.
"""

from datetime import datetime
from typing import Dict, List, Optional
from uuid import UUID
import logging

from ..models.session import ConversationSession, Message, MessageRole

logger = logging.getLogger(__name__)


class ConversationService:
    """
    Manages conversation sessions and message history.

    MVP Implementation: In-memory storage (session lost on restart)
    Production: Will use Neon Postgres (T010 schema)
    """

    def __init__(self):
        """Initialize in-memory storage for conversations."""
        # session_id -> ConversationSession
        self._sessions: Dict[UUID, ConversationSession] = {}
        logger.info("ConversationService initialized with in-memory storage (MVP)")

    async def create_session(self, session_id: UUID) -> ConversationSession:
        """
        Create a new conversation session.

        Args:
            session_id: Unique session identifier

        Returns:
            Created ConversationSession
        """
        now = datetime.utcnow()
        session = ConversationSession(
            session_id=session_id,
            created_at=now,
            last_activity=now,
            messages=[],
        )
        self._sessions[session_id] = session
        logger.info(f"Created session {session_id}")
        return session

    async def get_session(self, session_id: UUID) -> Optional[ConversationSession]:
        """
        Retrieve a conversation session.

        Args:
            session_id: Session identifier

        Returns:
            ConversationSession if found, None otherwise
        """
        return self._sessions.get(session_id)

    async def get_or_create_session(self, session_id: UUID) -> ConversationSession:
        """
        Get existing session or create new one.

        Args:
            session_id: Session identifier

        Returns:
            ConversationSession
        """
        session = await self.get_session(session_id)
        if session is None:
            session = await self.create_session(session_id)
        return session

    async def add_message(
        self,
        session_id: UUID,
        message: Message,
    ) -> None:
        """
        Add a message to a conversation session.

        Args:
            session_id: Session identifier
            message: Message to add
        """
        session = await self.get_or_create_session(session_id)
        session.messages.append(message)
        session.last_activity = datetime.utcnow()
        logger.debug(
            f"Added {message.role.value} message to session {session_id} "
            f"(total: {len(session.messages)} messages)"
        )

    async def get_recent_messages(
        self,
        session_id: UUID,
        limit: int = 10,
    ) -> List[Message]:
        """
        Get recent messages from a session.
        Returns messages in chronological order (oldest first).

        Args:
            session_id: Session identifier
            limit: Maximum number of messages to return

        Returns:
            List of recent messages
        """
        session = await self.get_session(session_id)
        if session is None:
            return []

        # Return last N messages
        return session.messages[-limit:] if session.messages else []

    async def get_conversation_context(
        self,
        session_id: UUID,
        max_pairs: int = 5,
    ) -> List[Message]:
        """
        Get conversation context for LLM prompt.
        Returns last N message pairs (user + assistant).

        Args:
            session_id: Session identifier
            max_pairs: Maximum number of message pairs to include

        Returns:
            List of messages for context (last N pairs)
        """
        # Get last max_pairs * 2 messages (each pair = user + assistant)
        recent_messages = await self.get_recent_messages(
            session_id,
            limit=max_pairs * 2,
        )

        # Filter to only user and assistant messages (exclude system)
        conversation_messages = [
            msg for msg in recent_messages
            if msg.role in [MessageRole.USER, MessageRole.ASSISTANT]
        ]

        return conversation_messages

    async def delete_session(self, session_id: UUID) -> bool:
        """
        Delete a conversation session.

        Args:
            session_id: Session identifier

        Returns:
            True if session was deleted, False if not found
        """
        if session_id in self._sessions:
            del self._sessions[session_id]
            logger.info(f"Deleted session {session_id}")
            return True
        return False

    def get_session_count(self) -> int:
        """
        Get total number of active sessions.

        Returns:
            Number of sessions in memory
        """
        return len(self._sessions)


# Global instance
conversation_service = ConversationService()
