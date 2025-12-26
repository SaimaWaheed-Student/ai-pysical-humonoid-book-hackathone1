from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship, declarative_base
from sqlalchemy.ext.asyncio import async_sessionmaker
from datetime import datetime
import os

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://neondb_owner:npg_x3h1BUEIRfWT@ep-little-silence-ahkavqz6-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require")

# Create async engine and session
engine = create_async_engine(DATABASE_URL)
AsyncSessionLocal = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

Base = declarative_base()

class Book(Base):
    __tablename__ = "books"

    id = Column(String, primary_key=True, index=True)
    title = Column(String, index=True)
    author = Column(String, index=True)
    total_pages = Column(Integer)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(String, primary_key=True, index=True)
    book_id = Column(String, ForeignKey("books.id"))
    chapter_number = Column(Integer)
    title = Column(String)
    start_page = Column(Integer)
    end_page = Column(Integer)
    created_at = Column(DateTime, default=datetime.utcnow)

class DocumentChunk(Base):
    __tablename__ = "document_chunks"

    id = Column(String, primary_key=True, index=True)
    book_id = Column(String, ForeignKey("books.id"))
    chapter_id = Column(String, ForeignKey("chapters.id"))
    chunk_index = Column(Integer)
    content = Column(Text)
    page_number = Column(Integer)
    qdrant_point_id = Column(String)
    token_count = Column(Integer)
    created_at = Column(DateTime, default=datetime.utcnow)

class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True)
    book_id = Column(String, ForeignKey("books.id"))
    title = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class Message(Base):
    __tablename__ = "messages"

    id = Column(String, primary_key=True, index=True)
    conversation_id = Column(String, ForeignKey("conversations.id"))
    role = Column(String)  # 'user' or 'assistant'
    content = Column(Text)
    sources = Column(JSON)  # Array of source chunks
    created_at = Column(DateTime, default=datetime.utcnow)

class UserSelection(Base):
    __tablename__ = "user_selections"

    id = Column(String, primary_key=True, index=True)
    conversation_id = Column(String, ForeignKey("conversations.id"))
    selected_text = Column(Text)
    selection_metadata = Column(JSON)
    created_at = Column(DateTime, default=datetime.utcnow)

# Create all tables
# Note: For async operations, we would use async functions
# This is kept for reference but typically handled differently in async applications

# Dependency to get async DB session
async def get_async_db():
    async with AsyncSessionLocal() as session:
        yield session