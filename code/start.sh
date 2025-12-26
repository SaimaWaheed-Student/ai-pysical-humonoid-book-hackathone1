#!/bin/bash

# Start the RAG Chatbot API
echo "Starting RAG Chatbot API..."

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Install dependencies if not already installed
pip install -r requirements.txt

# Start the application
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload