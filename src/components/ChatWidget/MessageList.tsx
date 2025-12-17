/**
 * MessageList component - Displays conversation messages with citations.
 * Supports markdown rendering and auto-scrolls to latest message.
 */

import React, { useEffect, useRef } from 'react';
import type { Message } from './types';
import styles from './styles.module.css';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

export default function MessageList({ messages, isLoading }: MessageListProps): JSX.Element {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  return (
    <div className={styles.messageList}>
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          <p>👋 Welcome! Ask me anything about Physical AI & Humanoid Robotics.</p>
          <p className={styles.emptyStateHint}>
            Try: "What is ROS 2?" or "Explain Isaac Sim"
          </p>
        </div>
      )}

      {messages.map((message) => (
        <div
          key={message.message_id}
          className={
            message.role === 'user'
              ? styles.userMessage
              : styles.assistantMessage
          }
        >
          {message.role === 'assistant' && (
            <div className={styles.messageAvatar}>🤖</div>
          )}

          <div className={styles.messageContent}>
            {/* Message text with basic markdown support */}
            <div
              className={styles.messageText}
              dangerouslySetInnerHTML={{
                __html: formatMarkdown(message.content),
              }}
            />

            {/* Citations (assistant messages only) */}
            {message.role === 'assistant' && message.source_citations && message.source_citations.length > 0 && (
              <div className={styles.citations}>
                <details>
                  <summary>
                    📚 Sources ({message.source_citations.length})
                  </summary>
                  <ul className={styles.citationList}>
                    {message.source_citations.map((citation, idx) => (
                      <li key={idx} className={styles.citationItem}>
                        <strong>{citation.source}</strong>
                        <p className={styles.citationExcerpt}>
                          {citation.excerpt}
                        </p>
                        <span className={styles.citationRelevance}>
                          Relevance: {(citation.relevance * 100).toFixed(0)}%
                        </span>
                      </li>
                    ))}
                  </ul>
                </details>
              </div>
            )}
          </div>

          {message.role === 'user' && (
            <div className={styles.messageAvatar}>👤</div>
          )}
        </div>
      ))}

      {/* Loading indicator */}
      {isLoading && (
        <div className={styles.assistantMessage}>
          <div className={styles.messageAvatar}>🤖</div>
          <div className={styles.messageContent}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        </div>
      )}

      {/* Scroll anchor */}
      <div ref={messagesEndRef} />
    </div>
  );
}

/**
 * Basic markdown formatting (simple implementation for MVP).
 * Handles: bold, italic, code blocks, inline code, lists.
 */
function formatMarkdown(text: string): string {
  let html = text;

  // Code blocks (```...```)
  html = html.replace(/```(.*?)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>');

  // Inline code (`...`)
  html = html.replace(/`([^`]+)`/g, '<code>$1</code>');

  // Bold (**...**)
  html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');

  // Italic (*...*)
  html = html.replace(/\*([^*]+)\*/g, '<em>$1</em>');

  // Links ([text](url))
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener noreferrer">$1</a>');

  // Line breaks
  html = html.replace(/\n/g, '<br />');

  return html;
}
