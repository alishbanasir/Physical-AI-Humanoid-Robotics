"""
Database connection pooling and dependency injection for FastAPI.
Uses SQLAlchemy async engine with connection pooling.
"""

import logging
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import (
    AsyncSession,
    AsyncEngine,
    create_async_engine,
    async_sessionmaker,
)
from sqlalchemy.pool import NullPool, QueuePool

from .config import settings

logger = logging.getLogger(__name__)


class DatabaseManager:
    """
    Manages database connections with async SQLAlchemy engine.
    Implements connection pooling for efficient resource usage.
    """

    def __init__(self):
        """Initialize async engine with connection pooling."""
        self.engine: AsyncEngine | None = None
        self.session_factory: async_sessionmaker[AsyncSession] | None = None
        self._initialize_engine()

    def _initialize_engine(self) -> None:
        """
        Create async SQLAlchemy engine with connection pooling.

        Configuration:
        - Pool size: 5 connections (suitable for pilot scale)
        - Max overflow: 10 additional connections under load
        - Pool recycle: 3600s (1 hour) to prevent stale connections
        - Pool pre-ping: True to verify connections before use
        """
        # Convert Neon Postgres URL to async format
        # postgresql:// -> postgresql+asyncpg://
        database_url = settings.neon_database_url
        if database_url.startswith("postgresql://"):
            database_url = database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
        elif not database_url.startswith("postgresql+asyncpg://"):
            raise ValueError(
                "Invalid database URL. Expected postgresql:// or postgresql+asyncpg://"
            )

        # Choose pooling strategy based on environment
        if settings.environment == "development":
            # Development: Use NullPool for easier debugging
            poolclass = NullPool
            logger.info("Using NullPool (no connection pooling) for development")
        else:
            # Production: Use QueuePool for connection reuse
            poolclass = QueuePool
            logger.info("Using QueuePool with pool_size=5, max_overflow=10")

        self.engine = create_async_engine(
            database_url,
            echo=settings.log_level.upper() == "DEBUG",  # Log SQL queries in debug mode
            poolclass=poolclass,
            pool_size=5 if poolclass == QueuePool else None,
            max_overflow=10 if poolclass == QueuePool else None,
            pool_recycle=3600,  # Recycle connections after 1 hour
            pool_pre_ping=True,  # Verify connection health before using
        )

        # Create async session factory
        self.session_factory = async_sessionmaker(
            bind=self.engine,
            class_=AsyncSession,
            expire_on_commit=False,  # Don't expire objects after commit
            autocommit=False,
            autoflush=False,
        )

        logger.info("Database engine initialized successfully")

    async def get_session(self) -> AsyncGenerator[AsyncSession, None]:
        """
        Dependency for FastAPI route handlers.
        Provides an async database session with automatic cleanup.

        Usage in routes:
            @router.post("/api/endpoint")
            async def endpoint(db: AsyncSession = Depends(get_db)):
                # Use db session
                pass

        Yields:
            AsyncSession: SQLAlchemy async session
        """
        if self.session_factory is None:
            raise RuntimeError("Database session factory not initialized")

        async with self.session_factory() as session:
            try:
                yield session
                await session.commit()
            except Exception:
                await session.rollback()
                raise
            finally:
                await session.close()

    async def close(self) -> None:
        """
        Close database engine and dispose of connection pool.
        Should be called during application shutdown.
        """
        if self.engine:
            await self.engine.dispose()
            logger.info("Database engine disposed")

    async def health_check(self) -> bool:
        """
        Check database connectivity for health endpoints.

        Returns:
            bool: True if database is reachable, False otherwise
        """
        if not self.engine:
            return False

        try:
            async with self.engine.connect() as conn:
                await conn.execute("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Database health check failed: {e}")
            return False


# Global database manager instance
db_manager = DatabaseManager()


# Dependency function for FastAPI
async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    FastAPI dependency for database sessions.

    Usage:
        from backend.src.core.dependencies import get_db

        @router.post("/api/endpoint")
        async def endpoint(db: AsyncSession = Depends(get_db)):
            # Use db session
            pass
    """
    async for session in db_manager.get_session():
        yield session
