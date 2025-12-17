"""
Configuration management for RAG Chatbot backend.
Loads settings from environment variables using Pydantic Settings.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional # <-- Add Optional type hint

class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # --- LLM Provider Configuration ---
    
    # FIX: Make OpenAI key optional and add Gemini key for flexibility
    # OpenAI Configuration (Now Optional, will fail only if explicitly used)
    openai_api_key: Optional[str] = None # <-- Made Optional and Default None
    
    # GEMINI Configuration (New fields)
    gemini_api_key: Optional[str] = None # <-- Added Gemini Key field

    # LLM and Embedding Model Names (Can be changed if using different models)
    embedding_model: str = "text-embedding-004" # <-- Changed default to Gemini's model
    llm_model: str = "gemini-2.5-flash"         # <-- Changed default to Gemini's model
    
    llm_temperature: float = 0.3
    max_context_messages: int = 5

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook_chunks"

    # Neon Serverless Postgres
    neon_database_url: Optional[str] = None # Assuming this might be optional for MVP Indexing test

    # Application Settings
    environment: str = "development"
    log_level: str = "INFO"
    api_rate_limit: int = 15

    # Vector Search Configuration
    vector_search_top_k: int = 5
    vector_search_threshold: float = 0.7

    # Chunking Configuration
    chunk_size: int = 800
    chunk_overlap: int = 100

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )


# Global settings instance
settings = Settings()