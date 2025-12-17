/**
 * ChatWindow component - Modal/drawer container for chat interface.
 * Contains MessageList and InputBar, with header and close button.
 */

import React from 'react';
import MessageList from './MessageList';
import InputBar from './InputBar';
import type { Message } from './types';
import styles from './styles.module.css';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  onSendMessage: (message: string) => void;
}

export default function ChatWindow({
  isOpen,
  onClose,
  messages,
  isLoading,
  error,
  onSendMessage,
}: ChatWindowProps): JSX.Element | null {
  if (!isOpen) {
    return null;
  }

  return (
    <div className={styles.chatWindow}>
      {/* Header */}
      <div className={styles.chatHeader}>
        <h3 className={styles.chatTitle}>
          🤖 Physical AI & Robotics Assistant
        </h3>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chat"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        </button>
      </div>

      {/* Content Area */}
      <div className={styles.chatContent}>
        {error && (
          <div className={styles.errorMessage}>
            <strong>Error:</strong> {error}
          </div>
        )}
        <MessageList messages={messages} isLoading={isLoading} />
      </div>

      {/* Input Bar */}
      <div className={styles.chatFooter}>
        <InputBar
          onSendMessage={onSendMessage}
          isLoading={isLoading}
          disabled={!!error}
        />
      </div>
    </div>
  );
}
