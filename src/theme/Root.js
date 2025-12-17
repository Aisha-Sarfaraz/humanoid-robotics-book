import React from 'react';
import ChatInterface from '../components/ChatInterface';

// Root component that wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatInterface />
    </>
  );
}