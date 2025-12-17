/**
 * InputBar component - Text input and send button for chat.
 * Handles user input with Enter key support and loading state.
 */

import React, { useState, KeyboardEvent } from 'react';
import styles from './styles.module.css';

interface InputBarProps {
  onSendMessage: (message: string) => void;
  isLoading: boolean;
  disabled?: boolean;
}

export default function InputBar({
  onSendMessage,
  isLoading,
  disabled = false,
}: InputBarProps): JSX.Element {
  const [input, setInput] = useState('');

  const handleSend = () => {
    const trimmedInput = input.trim();
    if (trimmedInput && !isLoading && !disabled) {
      onSendMessage(trimmedInput);
      setInput('');
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className={styles.inputBar}>
      <textarea
        className={styles.input}
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder={
          disabled
            ? 'Chat is unavailable'
            : isLoading
            ? 'Waiting for response...'
            : 'Ask a question... (Enter to send)'
        }
        disabled={isLoading || disabled}
        rows={1}
        maxLength={1000}
      />
      <button
        className={styles.sendButton}
        onClick={handleSend}
        disabled={!input.trim() || isLoading || disabled}
        aria-label="Send message"
      >
        {isLoading ? (
          // Loading spinner
          <svg
            className={styles.spinner}
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
          >
            <circle
              cx="12"
              cy="12"
              r="10"
              strokeWidth="3"
              strokeDasharray="32"
              strokeDashoffset="32"
            >
              <animate
                attributeName="stroke-dashoffset"
                values="32;0"
                dur="1s"
                repeatCount="indefinite"
              />
            </circle>
          </svg>
        ) : (
          // Send icon
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
            <line x1="22" y1="2" x2="11" y2="13"></line>
            <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
          </svg>
        )}
      </button>
    </div>
  );
}
