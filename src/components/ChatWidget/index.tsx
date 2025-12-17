/**
 * ChatWidget - Main component that composes all chat UI elements.
 * Provides floating chat button and modal window for Q&A interaction.
 */

import React, { useState } from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';
import { useChat } from './hooks/useChat';

export default function ChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const { messages, isLoading, error, sendMessage, clearError } = useChat();

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (error) {
      clearError();
    }
  };

  const handleSendMessage = async (message: string) => {
    await sendMessage(message);
  };

  return (
    <>
      <ChatButton onClick={toggleChat} isOpen={isOpen} />
      <ChatWindow
        isOpen={isOpen}
        onClose={toggleChat}
        messages={messages}
        isLoading={isLoading}
        error={error}
        onSendMessage={handleSendMessage}
      />
    </>
  );
}
