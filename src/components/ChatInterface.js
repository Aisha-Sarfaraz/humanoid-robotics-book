import React, { useState, useEffect, useRef } from 'react';
import './ChatInterface.css';

const ChatInterface = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [streamingMessage, setStreamingMessage] = useState('');
  const [sources, setSources] = useState([]);
  const [streamStatus, setStreamStatus] = useState('');
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);
  const eventSourceRef = useRef(null);

  // Function to get selected text
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    const currentInput = inputValue;

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setStreamingMessage('');
    setSources([]);
    setStreamStatus('Connecting...');

    try {
      // Prepare the request payload
      const requestBody = {
        message: currentInput,
        selected_text: selectedText || null,
      };

      if (sessionId) {
        requestBody.session_id = sessionId;
      }

      // Create fetch request for streaming
      const response = await fetch('http://localhost:8000/api/v1/chat/stream', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      let fullResponse = '';
      let responseSources = [];
      let responseSessionId = null;

      while (true) {
        const { done, value } = await reader.read();

        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop(); // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('event:')) {
            const eventType = line.substring(6).trim();
            const nextLine = lines[lines.indexOf(line) + 1];

            if (nextLine && nextLine.startsWith('data:')) {
              const data = JSON.parse(nextLine.substring(5).trim());

              if (eventType === 'status') {
                setStreamStatus(data.message);
              } else if (eventType === 'sources') {
                responseSources = data.sources;
                setSources(data.sources);
              } else if (eventType === 'chunk') {
                fullResponse += data.content;
                setStreamingMessage(fullResponse);
              } else if (eventType === 'done') {
                responseSessionId = data.session_id;
                if (responseSessionId && !sessionId) {
                  setSessionId(responseSessionId);
                }
              } else if (eventType === 'error') {
                throw new Error(data.message);
              }
            }
          }
        }
      }

      // Add final assistant message
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: fullResponse || 'No response received.',
        sources: responseSources,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
      setStreamingMessage('');
      setStreamStatus('');
      setSources([]);

    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error.message}. Please try again.`,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
      setStreamingMessage('');
      setStreamStatus('');
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const clearChat = () => {
    setMessages([]);
    setSessionId(null);
  };

  return (
    <div className="chat-container">
      {/* Chat toggle button */}
      <button
        className={`chat-toggle-btn ${isChatOpen ? 'open' : ''}`}
        onClick={toggleChat}
      >
        {isChatOpen ? '×' : '🤖'}
      </button>

      {/* Chat window */}
      {isChatOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Humanoid Robotics Book Assistant</h3>
            <div className="header-actions">
              <button onClick={clearChat} className="clear-btn" title="Clear chat">
                Clear
              </button>
              <button
                onClick={toggleChat}
                className="close-btn"
                title="Close chat"
              >
                ×
              </button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Humanoid Robotics Book assistant.</p>
                <p>You can ask me questions about the book content.</p>
                {selectedText && (
                  <div className="selected-text-preview">
                    <p><strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                >
                  <div className="message-content">
                    {message.content}

                    {message.sources && message.sources.length > 0 && (
                      <div className="sources">
                        <details>
                          <summary>Sources</summary>
                          <ul>
                            {message.sources.map((source, index) => (
                              <li key={index}>
                                <a href={source.path} target="_blank" rel="noopener noreferrer">
                                  {source.title || 'Source'}
                                </a>
                              </li>
                            ))}
                          </ul>
                        </details>
                      </div>
                    )}
                  </div>
                  <div className="message-timestamp">
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  {streamStatus && (
                    <div className="stream-status">
                      <em>{streamStatus}</em>
                    </div>
                  )}
                  {streamingMessage ? (
                    <>
                      <div className="streaming-content">{streamingMessage}</div>
                      <div className="streaming-indicator">●</div>
                    </>
                  ) : (
                    <div className="typing-indicator">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  )}
                  {sources.length > 0 && (
                    <div className="sources">
                      <details>
                        <summary>Sources ({sources.length})</summary>
                        <ul>
                          {sources.map((source, index) => (
                            <li key={index}>
                              {source.title} (score: {source.score.toFixed(3)})
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                  )}
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="selected-text-notification">
              Using selected text as context: "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"
            </div>
          )}

          <form onSubmit={handleSubmit} className="chat-input-form">
            <textarea
              ref={textareaRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask about the Humanoid Robotics Book..."}
              disabled={isLoading}
              rows={2}
              className="chat-input"
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className="send-btn"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatInterface;