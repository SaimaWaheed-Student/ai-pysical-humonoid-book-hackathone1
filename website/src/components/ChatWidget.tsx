import React, { useState, useRef, useEffect } from 'react';
import { askQuestion, ChatMessage } from '../lib/api/chat';
import clsx from 'clsx';
import styles from './ChatWidget.module.css'; // We will create this CSS module

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMsg: ChatMessage = { role: 'user', content: input };
    setMessages((prev) => [...prev, userMsg]);
    setInput('');
    setIsLoading(true);

    try {
      const history = messages.slice(-5); // Keep last 5 messages context
      const response = await askQuestion(userMsg.content, history);
      
      const botMsg: ChatMessage = { role: 'assistant', content: response.answer };
      setMessages((prev) => [...prev, botMsg]);
    } catch (error) {
      console.error('Chat error:', error);
      setMessages((prev) => [...prev, { role: 'assistant', content: 'Sorry, I encountered an error. Please try again.' }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatContainer}>
      {!isOpen && (
        <button className={styles.chatButton} onClick={() => setIsOpen(true)}>
          💬 Ask AI
        </button>
      )}

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Book Assistant</h3>
            <button className={styles.closeButton} onClick={() => setIsOpen(false)}>×</button>
          </div>
          
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                Hi! I can answer questions about the book content. Ask me anything!
              </div>
            )}
            {messages.map((msg, idx) => (
              <div key={idx} className={clsx(styles.message, msg.role === 'user' ? styles.userMessage : styles.botMessage)}>
                <div className={styles.messageContent}>{msg.content}</div>
              </div>
            ))}
            {isLoading && <div className={styles.loading}>Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.chatInputForm} onSubmit={handleSubmit}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Type your question..."
              disabled={isLoading}
              className={styles.chatInput}
            />
            <button type="submit" disabled={isLoading || !input.trim()} className={styles.sendButton}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
