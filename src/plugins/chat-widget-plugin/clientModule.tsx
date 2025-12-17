/**
 * Client module for chat widget plugin.
 * Injects ChatButton component into Docusaurus runtime.
 */

import React from 'react';
// CORRECT PATH & COMPONENT: humein ChatButton chahiye, jo chat window ko khole ga.
// Path: plugins folder se bahar nikal kar components/ChatWidget/ChatButton
import ChatButton from '../../components/ChatWidget/ChatButton';

// Render the ChatButton component on all pages
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      {/* Hum sirf ChatButton ko render karte hain, jo ChatWindow ko manage karta hai */}
      <ChatButton />
    </>
  );
}