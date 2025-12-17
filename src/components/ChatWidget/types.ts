/**
 * TypeScript interfaces for ChatWidget components.
 * Defines data structures for messages, citations, and chat state.
 */

export interface Citation {
  source: string; // e.g., "Module 1, Chapter 2"
  excerpt: string;
  relevance: number; // 0.0 to 1.0
}

export interface Message {
  message_id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: string; // ISO 8601 format
  source_citations?: Citation[];
}

export interface ChatState {
  isOpen: boolean;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sessionId: string;
}

export interface ChatRequest {
  session_id: string;
  query: string;
  context?: {
    current_page?: string;
    selected_text?: string;
  };
}

export interface ChatResponse {
  message_id: string;
  response: string;
  citations: Citation[];
  suggested_questions: string[];
  processing_time_ms: number;
}
