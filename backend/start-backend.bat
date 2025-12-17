@echo off
REM FastAPI Backend Startup Script
REM This script starts the RAG chatbot backend server

echo ========================================
echo Starting RAG Chatbot Backend...
echo ========================================
echo.

cd /d "%~dp0"

echo Activating virtual environment...
call venv\Scripts\activate.bat

echo.
echo Starting uvicorn server on http://localhost:8000...
echo Press Ctrl+C to stop the server
echo.

python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

pause
