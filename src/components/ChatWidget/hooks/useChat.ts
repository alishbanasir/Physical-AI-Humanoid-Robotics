/**
 * useChat hook - Manages chat state and API integration.
 * Handles session management with localStorage, message state, and API calls.
 * FIX: API endpoint corrected from /api/chat/query to /api/chat
 */

import { useState, useEffect, useCallback } from 'react';
import axios from 'axios';
import { v4 as uuidv4 } from 'uuid';
import type { Message, ChatRequest, ChatResponse } from '../types';

// API_BASE_URL: Ensures it points to the local FastAPI server
// API_BASE_URL: Ensures it points to the Hugging Face backend
const API_BASE_URL = 'https://alishba-nasir-humanoid-backend.hf.space';
const SESSION_STORAGE_KEY = 'chatbot_session_id';

interface UseChatReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (query: string, selectedText?: string) => Promise<void>;
  clearError: () => void;
}

export function useChat(): UseChatReturn {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string>('');

  // Initialize session ID from localStorage or create new one
  useEffect(() => {
    let storedSessionId = localStorage.getItem(SESSION_STORAGE_KEY);

    if (!storedSessionId) {
      storedSessionId = uuidv4();
      localStorage.setItem(SESSION_STORAGE_KEY, storedSessionId);
    }

    setSessionId(storedSessionId);

    // Note: Assuming you have implemented the /api/chat/history endpoint in backend.
    loadHistory(storedSessionId); 
  }, []);

  // Load conversation history from backend
  const loadHistory = async (sid: string) => {
    try {
      // NOTE: History endpoint call is kept as is.
      const response = await axios.get(`${API_BASE_URL}/api/chat/history/${sid}`);
      if (response.data.messages && Array.isArray(response.data.messages)) {
        setMessages(response.data.messages);
      }
    } catch (err) {
      console.error('Failed to load conversation history:', err);
      // Don't show error to user - just start with empty conversation
    }
  };

  // Send a message to the chatbot
  const sendMessage = useCallback(
    async (query: string, selectedText?: string) => {
      if (!sessionId || !query.trim()) {
        return;
      }

      setIsLoading(true);
      setError(null);

      // Add user message to UI immediately (optimistic update)
      const userMessage: Message = {
        message_id: uuidv4(),
        role: 'user',
        content: query,
        timestamp: new Date().toISOString(),
      };

      setMessages((prev) => [...prev, userMessage]);

      try {
        // Prepare request
        const request: ChatRequest = {
          session_id: sessionId,
          query: query.trim(),
          context: selectedText
            ? {
                selected_text: selectedText,
                current_page: window.location.pathname,
              }
            : {
                current_page: window.location.pathname,
              },
        };

// Call backend API - *** FINAL FIX APPLIED HERE: Changed to /api/chat ***
const response = await axios.post<ChatResponse>(
    // FINAL FIX: '/api' prefix add karein taake woh router se mil sake
    `${API_BASE_URL}/api/chat`, 
    request,
    {
        timeout: 30000, // 30 second timeout
    }
);

        // Add assistant response to messages
        const assistantMessage: Message = {
          message_id: response.data.message_id,
          role: 'assistant',
          content: response.data.response,
          timestamp: new Date().toISOString(),
          // Ensure your backend returns 'citations' field in ChatResponse
          source_citations: response.data.citations, 
        };

        setMessages((prev) => [...prev, assistantMessage]);
      } catch (err) {
        console.error('Failed to send message:', err);

        // Handle different error types
        if (axios.isAxiosError(err)) {
          // Simplified error detail handling for clarity
          setError(
            err.response?.data?.detail ||
            'Failed to get response. Please check your backend connection and logs.'
          );
        } else {
          setError('An unexpected error occurred. Please try again.');
        }
      } finally {
        setIsLoading(false);
      }
    },
    [sessionId]
  );

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    clearError,
  };
}