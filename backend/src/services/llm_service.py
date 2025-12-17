# backend/src/services/llm_service.py

import logging
from google import genai
from google.genai import types
from ..core.config import settings

logger = logging.getLogger(__name__)

class LLMService:
    def __init__(self):
        self.client = genai.Client(api_key=settings.gemini_api_key)
        self.model_id = settings.llm_model
        logger.info(f"LLM Service initialized with model: {self.model_id}")

    async def generate_rag_response(self, query: str, context: str) -> str:
        """
        Generates a response using the Gemini API based on provided context.
        """
        # IMPROVED PROMPT: Adding flexibility so it doesn't say "I don't know" too easily
        system_prompt = f"""
        You are an expert AI assistant for the 'Humanoid Robotics' textbook.
        Your goal is to provide accurate, helpful, and technical information.

        CONTEXT FROM TEXTBOOK:
        {context}

        USER QUESTION: 
        {query}

        INSTRUCTIONS:
        1. Use the 'CONTEXT FROM TEXTBOOK' provided above to answer the question.
        2. If the context contains the answer, explain it clearly using that data.
        3. If the context is very brief or doesn't fully answer the question, use your internal 
           knowledge about Robotics to complete the answer, but prioritize the textbook context.
        4. If the question is a greeting (like 'hi' or 'hello'), respond politely.
        5. Maintain a professional and educational tone.
        """

        try:
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=system_prompt,
                config=types.GenerateContentConfig(
                    temperature=0.7, # Thora creative taake answer behtar ho
                    max_output_tokens=1000,
                ),
            )
            
            if not response.text:
                return "I'm sorry, I couldn't generate a response. Please try again."
                
            return response.text

        except Exception as e:
            logger.error(f"Error generating LLM response: {e}")
            raise e

# Global instance
llm_service = LLMService()