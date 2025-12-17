"""
FastAPI application entry point for RAG Chatbot backend.
Configures CORS, exception handlers, and routes.
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request, status
# NEW IMPORT: Add dotenv library
from dotenv import load_dotenv # <--- YEH LINE ADD KAREIN
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
# CRITICAL FIX 1: Add missing import for validation exception
from fastapi.exceptions import RequestValidationError # <--- YEH LINE ADD KAREIN
# ... baaki imports wahi rahenge

# Load environment variables at the start of the script
load_dotenv() # <--- YEH LINE ADD KAREIN

from .core.config import settings

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    Initializes services and cleans up resources.
    """
    # Startup
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Qdrant collection: {settings.qdrant_collection_name}")

    # Import services to trigger initialization
    from .services.vector_store import vector_store
    from .services.embedder import embedder, EMBEDDING_MODEL # <--- YEH LINE BADLEIN

    # Log service initialization
    collection_info = vector_store.get_collection_info()
    logger.info(f"Vector store initialized: {collection_info}")
    
    # CRITICAL FIX 1: Access the correct module-level constant EMBEDDING_MODEL
    logger.info(f"Embedder initialized with model: {EMBEDDING_MODEL}") # <--- YEH LINE BADLEIN
    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
)


# CORS Middleware Configuration
# Allow Docusaurus frontend to access the API
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",    # Docusaurus dev server (default port)
        "http://localhost:3001",    # Docusaurus dev server (alternate port)
        "http://localhost:8000",    # Backend dev server
        # Add production origins when deployed
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)


# Exception Handlers

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """
    Handle Pydantic validation errors with detailed error messages.
    Returns 400 Bad Request with field-level error details.
    """
    errors = []
    for error in exc.errors():
        errors.append({
            "field": " -> ".join(str(loc) for loc in error["loc"]),
            "message": error["msg"],
            "type": error["type"],
        })

    logger.warning(f"Validation error on {request.url.path}: {errors}")

    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={
            "error": "Validation Error",
            "message": "Request data is invalid",
            "details": errors,
        },
    )


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler for unhandled errors.
    Returns 500 Internal Server Error with error details in development.
    """
    logger.error(f"Unhandled exception on {request.url.path}: {exc}", exc_info=True)

    # In production, hide implementation details
    if settings.environment == "production":
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": "Internal Server Error",
                "message": "An unexpected error occurred. Please try again later.",
            },
        )

    # In development, show full error details
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "error": "Internal Server Error",
            "message": str(exc),
            "type": type(exc).__name__,
        },
    )


# Root endpoint
@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "online",
        "docs": "/docs",
    }


# Import and register route modules
# CRITICAL FIX 2: Import the unified router and use its router instance
from .api.router import router as api_router 

app.include_router(api_router, prefix="/api", tags=["chat", "health"])


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True if settings.environment == "development" else False,
        log_level=settings.log_level.lower(),
    )