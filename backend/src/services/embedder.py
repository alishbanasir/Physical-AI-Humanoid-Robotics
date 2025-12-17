# backend/src/services/embedder.py

import os
import sys  # CRITICAL FIX: Add this import
import logging
from typing import List
from google import genai
from google.genai import types
from google.genai.errors import APIError

logger = logging.getLogger(__name__)

# --- Configuration ---
# Check if GEMINI_API_KEY is available and use it
if "GEMINI_API_KEY" not in os.environ:
    # This check will now only trigger if load_dotenv() fails in index_content.py
    logger.error("GEMINI_API_KEY is not set in environment variables.")
    sys.exit(1)

# Use Google GenAI client (uses GEMINI_API_KEY from environment)
client = genai.Client()

# Model names (Gemini's embedding model)
EMBEDDING_MODEL = "text-embedding-004" 
MAX_BATCH_SIZE = 100 

class EmbedderService:
    """Handles embedding generation using Gemini API."""

    def __init__(self):
        logger.info(f"Embedder initialized with model: {EMBEDDING_MODEL}")
        self._cache = {} # Simple in-memory cache for repeated embeddings

    async def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of text documents.

        Args:
            texts: List of strings to embed.

        Returns:
            List of embedding vectors.
        """
        embeddings = []
        texts_to_embed = []
        
        # 1. Check cache
        for text in texts:
            if text in self._cache:
                embeddings.append(self._cache[text])
            else:
                texts_to_embed.append(text)

        if not texts_to_embed:
            return embeddings

        # 2. Call Gemini API for new texts
# 2. Call Gemini API for new texts
        # 2. Call Gemini API for new texts
        try:
            # CRITICAL FIX 1: Using the correct method (models endpoint) and removing task_type
            response = client.models.embed_content( 
                model=EMBEDDING_MODEL,
                contents=texts_to_embed, 
                # task_type="RETRIEVAL_DOCUMENT" # Removed: Fixes 'task_type' error
            )
            
# CRITICAL FIX 2: Access embeddings using the correct plural attribute: .embeddings
            new_embeddings = response.embeddings 
            
            # CRITICAL FIX 3: Extract the vector list (the 'values' attribute) from each object
            final_vectors = []
            for item in new_embeddings:
                 # Har ContentEmbedding object mein 'values' naam ka attribute hota hai jismein vector hota hai.
                 final_vectors.append(item.values) # <--- YEH LINE BADLEIN! (item.values)

            # Ab final_vectors mein woh list of floats hogi jo Qdrant ko chahiye.
            
            for text, embedding in zip(texts_to_embed, final_vectors):
                self._cache[text] = embedding
                embeddings.append(embedding)

            return embeddings

        except APIError as e:
            logger.error(f"Gemini API Error during embedding: {e}")
            raise RuntimeError(f"Failed to generate embeddings: {e}")
        except Exception as e:
            # Catch other potential errors, like incorrect response structure if using non-standard SDK
            logger.error(f"General Embedding Error: {e}")
            raise RuntimeError(f"Failed to generate embeddings: {e}")

embedder = EmbedderService()